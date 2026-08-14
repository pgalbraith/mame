// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

  Heathkit H-17 Floppy controller

    This was an option for both the Heathkit H8 and H89 computer systems.

  TODO
    - The USRT receive clock is synthesised rather than recovered from the
      drive.  rx_timer_cb() re-arms at a fixed fm_cell_time() and steps
      fdc_pll_t by hand, splitting FM clock and data half-cells with the
      m_rx_clock_cell flag.  Two consequences:

        * get_next_bit() is passed a limit of machine().time() + fm_cell_time(),
          exactly one nominal cell ahead of the PLL's ctime, so the PLL can
          never adopt a delay longer than nominal.  When it needs to stretch to
          track slower media it returns -1 instead, and only makes progress
          because the fixed timer lets machine time drift ahead of ctime on the
          following call.  It converges and sectors decode byte-exact, but the
          usual MAME idiom is to schedule the next event from the PLL's own
          ctime (compare wd_fdc/upd765) instead of a free-running timer.

        * The clock/data phase is recovered heuristically: a 0 seen where a
          clock half-cell was expected is treated as data and the phase is left
          unchanged.  That resynchronises within a bit or two on the zero-byte
          preambles, but it is a guess rather than a decode.

      Doing this properly means letting the incoming floppy data drive the
      2350's receive clock, which is the long-standing TODO in h17_fdc.h.

    - The media data rate is not settled; see the note on BITCELL_SIZE in
      formats/h8d_dsk.cpp.

****************************************************************************/

#include "emu.h"
#include "h17_fdc.h"

#include "imagedev/floppy.h"
#include "machine/fdc_pll.h"
#include "machine/s2350.h"

#include "formats/h17disk.h"
#include "formats/h8d_dsk.h"
#include "formats/mfi_dsk.h"

#include "softlist_dev.h"


#define LOG_REG   (1U << 1) // Register setup
#define LOG_LINES (1U << 2) // Control lines
#define LOG_DRIVE (1U << 3) // Drive select
#define LOG_FUNC  (1U << 4) // Function calls
#define LOG_SETUP (1U << 5)

//#define VERBOSE (LOG_GENERAL | LOG_REG | LOG_LINES | LOG_DRIVE | LOG_FUNC)

#include "logmacro.h"

#define LOGREG(...)        LOGMASKED(LOG_REG, __VA_ARGS__)
#define LOGLINES(...)      LOGMASKED(LOG_LINES, __VA_ARGS__)
#define LOGDRIVE(...)      LOGMASKED(LOG_DRIVE, __VA_ARGS__)
#define LOGFUNC(...)       LOGMASKED(LOG_FUNC, __VA_ARGS__)
#define LOGSETUP(...)      LOGMASKED(LOG_SETUP, __VA_ARGS__)

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

namespace {

class heath_h17_fdc_device : public device_t, public device_h89bus_right_card_interface
{
public:
	heath_h17_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	[[maybe_unused]] void side_select_w(int state);

protected:
	static constexpr u8 MAX_FLOPPY_DRIVES = 3;

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	void write(offs_t offset, u8 data);
	u8 read(offs_t offset);

	void ctrl_w(u8 val);
	u8 floppy_status_r();

	static void floppy_formats(format_registration &fr);
	void set_floppy(floppy_image_device *floppy);
	void step_w(int state);
	void dir_w(int state);
	void set_motor(bool motor_on);
	void reset_rx_pll();
	void start_rx_timer();
	void set_write_gate(bool write_gate);
	void start_tx_write();
	void stop_tx_write();
	void write_tx_cell(bool bit);
	void tx_w(int state);

	void sync_character_received(int state);

	TIMER_CALLBACK_MEMBER(rx_timer_cb);
	TIMER_DEVICE_CALLBACK_MEMBER(tx_timer_cb);

	required_device<s2350_device> m_s2350;
	required_device_array<floppy_connector, MAX_FLOPPY_DRIVES> m_floppies;
	required_device<timer_device> m_tx_timer;
	emu_timer *m_rx_timer;
	fdc_pll_t m_rx_pll;
	fdc_pll_t m_tx_pll;

	bool m_installed;

	bool m_motor_on;
	bool m_write_gate;
	bool m_tx_write_active;
	bool m_sync_char_received;
	bool m_rx_clock_cell;
	u8   m_step_direction;
	u8   m_side;

	floppy_image_device *m_floppy;

	/// write bit control port
	static constexpr u8 CTRL_WRITE_GATE       = 0;
	static constexpr u8 CTRL_DRIVE_SELECT_0   = 1;
	static constexpr u8 CTRL_DRIVE_SELECT_1   = 2;
	static constexpr u8 CTRL_DRIVE_SELECT_2   = 3;
	static constexpr u8 CTRL_MOTOR_ON         = 4; // Controls all the drives
	static constexpr u8 CTRL_DIRECTION        = 5; // (1 = away from track 0, 0 = out)
	static constexpr u8 CTRL_STEP_COMMAND     = 6; // (Active high)
	static constexpr u8 CTRL_WRITE_ENABLE_RAM = 7; // 0 - write protected

	// USRT clock
	static constexpr XTAL USRT_BASE_CLOCK = XTAL(12'288'000) / 6 / 16;
	static constexpr u32  USRT_TX_CLOCK   = USRT_BASE_CLOCK.value();
	static attotime fm_cell_time() { return attotime::from_hz(USRT_TX_CLOCK * 2); }
};


heath_h17_fdc_device::heath_h17_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, H89BUS_H_17_FDC, tag, owner, 0)
	, device_h89bus_right_card_interface(mconfig, *this)
	, m_s2350(*this, "s2350")
	, m_floppies(*this, "floppy%u", 0U)
	, m_tx_timer(*this, "tx_timer")
	, m_rx_timer(nullptr)
	, m_floppy(nullptr)
{
}

void heath_h17_fdc_device::write(offs_t offset, u8 data)
{
	LOGFUNC("%s: reg: %d val: 0x%02x\n", FUNCNAME, offset, data);

	switch (offset & 3)
	{
		case 0: // data port
			m_s2350->transmitter_holding_reg_w(data);
			break;
		case 1: // fill character
			m_s2350->transmit_fill_reg_w(data);
			break;
		case 2: // sync port
			m_s2350->receiver_sync_reg_w(data);
			break;
		case 3: // control port
			ctrl_w(data);
			break;
	}
}

void heath_h17_fdc_device::set_floppy(floppy_image_device *floppy)
{
	if (m_floppy == floppy)
	{
		return;
	}

	LOGDRIVE("%s: selecting new drive\n", FUNCNAME);

	stop_tx_write();

	m_floppy = floppy;

	// set any latched signals
	if (m_floppy)
	{
		m_floppy->ss_w(m_side);
	}

	reset_rx_pll();
	start_tx_write();
}

void heath_h17_fdc_device::side_select_w(int state)
{
	m_side = BIT(state, 0);

	if (m_floppy)
	{
		stop_tx_write();
		m_floppy->ss_w(m_side);
		reset_rx_pll();
		start_tx_write();
	}
}

void heath_h17_fdc_device::dir_w(int state)
{
	m_step_direction = state;

	if (m_floppy)
	{
		LOGFUNC("%s: step dir: 0x%02x\n", FUNCNAME, state);

		m_floppy->dir_w(state);
	}
}

void heath_h17_fdc_device::step_w(int state)
{
	if (m_floppy)
	{
		LOGFUNC("%s: step: 0x%02x\n", FUNCNAME, state);

		stop_tx_write();
		m_floppy->stp_w(state);
		reset_rx_pll();
		start_tx_write();
	}
}

void heath_h17_fdc_device::set_motor(bool motor_on)
{
	if (m_motor_on == motor_on)
	{
		return;
	}

	if (!motor_on)
	{
		stop_tx_write();
	}

	m_motor_on = motor_on;

	for (auto &elem : m_floppies)
	{
		floppy_image_device *floppy = elem->get_device();
		if (floppy)
		{
			LOGFUNC("%s: motor: %d\n", FUNCNAME, motor_on);

			floppy->mon_w(!motor_on);
		}
	}

	reset_rx_pll();
	start_tx_write();
}

void heath_h17_fdc_device::ctrl_w(u8 val)
{
	set_write_gate(bool(BIT(val, CTRL_WRITE_GATE)));

	set_motor(bool(BIT(val, CTRL_MOTOR_ON)));

	if (BIT(val, CTRL_DRIVE_SELECT_0))
	{
		LOGFUNC("%s: set drive 0\n", FUNCNAME);

		set_floppy(m_floppies[0]->get_device());
	}
	else if (BIT(val, CTRL_DRIVE_SELECT_1))
	{
		LOGFUNC("%s: set drive 1\n", FUNCNAME);

		set_floppy(m_floppies[1]->get_device());
	}
	else if (BIT(val, CTRL_DRIVE_SELECT_2))
	{
		LOGFUNC("%s: set drive 2\n", FUNCNAME);

		set_floppy(m_floppies[2]->get_device());
	}
	else
	{
		LOGFUNC("%s: set drive none\n", FUNCNAME);

		set_floppy(nullptr);
	}

	// CTRL_DIRECTION bit 5: 1 steps away from track 0, 0 steps toward it.  The
	// boot ROM's seek loop at $1D95 reaches its "step" entry point at $1DA9
	// (which sets bit 5) only after incrementing its track counter, and $1DAC
	// (which clears bit 5) only after decrementing it.  floppy_image_device
	// uses the opposite sense, decrementing the cylinder when m_dir is 1.
	dir_w(!BIT(val, CTRL_DIRECTION));

	step_w(!BIT(val, CTRL_STEP_COMMAND));

	// FMWE write-enables the controller's own RAM in the ROM address space
	// ($1400-$17FF on the H89).  It is a P506 bus signal, so it has to be
	// driven through the slot interface - the h89bus routes it to the driver's
	// memory view selection.
	set_slot_fmwe(BIT(val, CTRL_WRITE_ENABLE_RAM));
}

u8 heath_h17_fdc_device::read(offs_t offset)
{
	u8 val = 0;

	switch (offset & 3)
	{
		case 0: // data port
			val = m_s2350->receiver_output_reg_r();
			break;
		case 1: // status port
			val = m_s2350->status_word_r();
			break;
		case 2: // sync port
			val = m_s2350->receiver_sync_search();
			break;
		case 3: // floppy status port
			val = floppy_status_r();
			break;
	}

	LOGREG("%s: reg: %d val: 0x%02x\n", FUNCNAME, offset, val);

	return val;
}

u8 heath_h17_fdc_device::floppy_status_r()
{
	u8 val = 0;

	// statuses from the floppy drive
	if (m_floppy)
	{
		// index/sector hole (idx_r is active-high: 1 = hole present = DF.HD set)
		val |= m_floppy->idx_r() ? 0x01 : 0x00;

		// TK00 is active low from the drive and inverted on the H17 board, so
		// bit 1 reads 1 at track 0.  The boot ROM's home loop at $1EAC keeps
		// stepping while bit 1 is clear.
		val |= m_floppy->trk00_r() ? 0x00 : 0x02;

		// disk is write-protected
		val |= m_floppy->wpt_r() ? 0x00 : 0x04;
	}
	else
	{
		LOGREG("%s: no drive selected\n", FUNCNAME);
	}

	// status from USRT
	val |= m_sync_char_received ? 0x08 : 0x00;

	LOGFUNC("%s: val: 0x%02x\n", FUNCNAME, val);

	return val;
}

void heath_h17_fdc_device::device_start()
{
	m_installed = false;
	m_rx_timer = timer_alloc(FUNC(heath_h17_fdc_device::rx_timer_cb), this);

	save_item(NAME(m_installed));
	save_item(NAME(m_motor_on));
	save_item(NAME(m_write_gate));
	save_item(NAME(m_tx_write_active));
	save_item(NAME(m_sync_char_received));
	save_item(NAME(m_rx_clock_cell));
	save_item(NAME(m_step_direction));
	save_item(NAME(m_side));
}

void heath_h17_fdc_device::device_reset()
{
	if (!m_installed)
	{
		h89bus::addr_ranges  addr_ranges = h89bus().get_address_ranges(h89bus::IO_FLPY);

		if (addr_ranges.size() == 1)
		{
			h89bus::addr_range range = addr_ranges.front();

			h89bus().install_io_device(range.first, range.second,
				read8sm_delegate(*this, FUNC(heath_h17_fdc_device::read)),
				write8sm_delegate(*this, FUNC(heath_h17_fdc_device::write)));
		}

		m_installed = true;
	}

	m_motor_on           = false;
	m_write_gate         = false;
	m_tx_write_active    = false;
	m_sync_char_received = false;
	m_rx_clock_cell      = true;
	m_step_direction     = 0;
	m_side               = 0;

	m_tx_timer->adjust(attotime::from_hz(USRT_TX_CLOCK), 0, attotime::from_hz(USRT_TX_CLOCK));
	reset_rx_pll();
	m_tx_pll.set_clock(fm_cell_time());
	m_tx_pll.reset(machine().time());
}

static void h17_floppies(device_slot_interface &device)
{
	// H-17-1
	device.option_add("ssdd", FLOPPY_525_SSDD);

	// Future plans - test and verify higher capacity drives with LLC's BIOS-80 for CP/M and an HUG's enhanced HDOS driver
	//  - FLOPPY_525_SSQD
	//  - FLOPPY_525_DD
	//  - FLOPPY_525_QD (H-17-4)
}

TIMER_DEVICE_CALLBACK_MEMBER(heath_h17_fdc_device::tx_timer_cb)
{
	m_s2350->tcp_w();
}

void heath_h17_fdc_device::set_write_gate(bool write_gate)
{
	if (m_write_gate == write_gate)
	{
		return;
	}

	if (!write_gate)
	{
		stop_tx_write();
	}

	m_write_gate = write_gate;

	if (write_gate)
	{
		start_tx_write();
	}
}

void heath_h17_fdc_device::start_tx_write()
{
	if (m_tx_write_active || !m_write_gate || !m_motor_on || !m_floppy)
	{
		return;
	}

	m_tx_pll.set_clock(fm_cell_time());
	m_tx_pll.reset(machine().time());
	m_tx_pll.start_writing(machine().time());
	m_tx_write_active = true;
}

void heath_h17_fdc_device::stop_tx_write()
{
	if (!m_tx_write_active)
	{
		return;
	}

	m_tx_pll.stop_writing(m_floppy, m_tx_pll.ctime);
	m_tx_write_active = false;
}

void heath_h17_fdc_device::write_tx_cell(bool bit)
{
	attotime tm;
	m_tx_pll.write_next_bit(bit, tm, m_floppy, m_tx_pll.ctime + fm_cell_time());

	if (m_tx_pll.write_position >= 30)
	{
		m_tx_pll.commit(m_floppy, m_tx_pll.ctime);
	}
}

void heath_h17_fdc_device::tx_w(int state)
{
	if (!m_tx_write_active)
	{
		return;
	}

	write_tx_cell(true);
	write_tx_cell(BIT(state, 0));
}

void heath_h17_fdc_device::start_rx_timer()
{
	if (m_motor_on && m_floppy)
	{
		m_rx_timer->adjust(fm_cell_time());
	}
	else
	{
		m_rx_timer->adjust(attotime::never);
	}
}

void heath_h17_fdc_device::reset_rx_pll()
{
	m_rx_pll.set_clock(fm_cell_time());
	m_rx_pll.read_reset(machine().time());
	m_rx_clock_cell = true;
	start_rx_timer();
}

TIMER_CALLBACK_MEMBER(heath_h17_fdc_device::rx_timer_cb)
{
	if (!m_motor_on || !m_floppy)
	{
		m_rx_timer->adjust(attotime::never);
		return;
	}

	attotime tm;
	int const bit = m_rx_pll.get_next_bit(tm, m_floppy, machine().time() + fm_cell_time());

	if (bit >= 0)
	{
		if (m_rx_clock_cell)
		{
			if (bit == 0)
			{
				// FM clock cells are always 1. Getting 0 here means phase is
				// inverted — we're in a data-0 half-cell, not a clock half-cell.
				// Send it as data and stay expecting a clock on the next call.
				m_s2350->rx_w(0);
				m_s2350->rcp_w();
			}
			else
			{
				m_rx_clock_cell = false;
			}
		}
		else
		{
			m_s2350->rx_w(bit);
			m_s2350->rcp_w();
			m_rx_clock_cell = true;
		}
	}

	m_rx_timer->adjust(fm_cell_time());
}

void heath_h17_fdc_device::floppy_formats(format_registration &fr)
{
	fr.add(FLOPPY_H17D_FORMAT);
	fr.add(FLOPPY_H8D_FORMAT);
	fr.add(FLOPPY_MFI_FORMAT);
}

void heath_h17_fdc_device::device_add_mconfig(machine_config &config)
{
	S2350(config, m_s2350);
	m_s2350->tx_handler().set(FUNC(heath_h17_fdc_device::tx_w));
	m_s2350->sync_character_received_cb().set(FUNC(heath_h17_fdc_device::sync_character_received));

	for (int i = 0; i < MAX_FLOPPY_DRIVES; i++)
	{
		FLOPPY_CONNECTOR(config, m_floppies[i], h17_floppies, "ssdd", heath_h17_fdc_device::floppy_formats);
		m_floppies[i]->set_sectoring_type(floppy_image::H10);
		m_floppies[i]->enable_sound(true);
	}

	TIMER(config, m_tx_timer).configure_generic(FUNC(heath_h17_fdc_device::tx_timer_cb));

	// the list belongs to the controller rather than to any one machine: the
	// H-88-1 came standard on the H-89 and was an option for the H-88
	SOFTWARE_LIST(config, "flop_list").set_original("h89_flop");
}

void heath_h17_fdc_device::sync_character_received(int state)
{
	LOGFUNC("%s: state: %d\n", FUNCNAME, state);

	m_sync_char_received = bool(BIT(state, 0));
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H89BUS_H_17_FDC, device_h89bus_right_card_interface, heath_h17_fdc_device, "h89_h17_fdc", "Heath H-17 Hard-sectored Controller (H-88-1)");
