// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

  Heathkit H-17 Floppy controller

    This was an option for both the Heathkit H8 and H89 computer systems.

  TODO
    - The USRT receive clock is synthesised rather than recovered from the
      drive.  rx_timer_cb() runs an analogue fdc_pll_t and splits FM clock and
      data half-cells with the m_rx_clock_cell flag.  The board does it
      digitally instead; from the H-88-1 schematic (595-2195-01):

        READ DATA L is buffered and fires U808 (443-22), a retriggerable
        one-shot with an internal timing resistor and C811 = 22 pF, so every
        flux transition becomes a short pulse.  That pulse drives the ENP and
        ENT count enables of U805 (443-757), a 4-bit synchronous counter whose
        LOAD is tied high and whose CLR comes from the cross-coupled NAND latch
        U811 (443-798).  U805 is clocked from a reference divider - U810, a
        second 443-757 with all its control pins strapped to +5V, feeding
        decoder U813 (443-807) and gates U812 (443-728).  U805's QC and QD,
        recombined through U811 and the second latch U809 (443-728), produce
        the S2350's RCP (U802 pin 37) and RSI (pin 23).

      So the real separator is a window generator re-anchored by the incoming
      transitions, not a free-running sampler and not a PLL: the phase question
      the m_rx_clock_cell flag answers by guesswork is answered in hardware by
      which count a pulse lands on.  What is not yet traced is exactly which
      counter output forms RCP versus the data window, which is what a faithful
      reimplementation would need.

      The guess is cheap to live with in the meantime.  Over a full HDOS 1.6
      boot and CAT (5,000,000 half-cells) the phase moved 587 times and every
      one of those was while the receiver was hunting for sync, none after a
      sync character had been matched.  So it re-locks in the gaps and holds
      phase across every header and data field the ROM actually consumes -
      behaviourally what the circuit above does.  Replacing it is a fidelity
      improvement, not a fix for anything observable.

    - The media data rate is not settled; see the note on BITCELL_SIZE in
      formats/h17_common.h.

****************************************************************************/

#include "emu.h"
#include "h17_fdc_base.h"

#include "formats/h17disk.h"
#include "formats/h8d_dsk.h"
#include "formats/mfi_dsk.h"


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


heath_h17_fdc_base_device::heath_h17_fdc_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_s2350(*this, "s2350")
	, m_floppies(*this, "floppy%u", 0U)
	, m_tx_timer(*this, "tx_timer")
	, m_rx_timer(nullptr)
	, m_floppy(nullptr)
{
}

void heath_h17_fdc_base_device::write(offs_t offset, u8 data)
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

void heath_h17_fdc_base_device::set_floppy(floppy_image_device *floppy)
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

void heath_h17_fdc_base_device::side_select_w(int state)
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

void heath_h17_fdc_base_device::dir_w(int state)
{
	m_step_direction = state;

	if (m_floppy)
	{
		LOGFUNC("%s: step dir: 0x%02x\n", FUNCNAME, state);

		m_floppy->dir_w(state);
	}
}

void heath_h17_fdc_base_device::step_w(int state)
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

void heath_h17_fdc_base_device::set_motor(bool motor_on)
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

void heath_h17_fdc_base_device::ctrl_w(u8 val)
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

	// FMWE write-enables the controller's 1k of RAM ($1400-$17FF) in the ROM
	// address space.  Where that RAM lives depends on the card, so leave it to
	// the bus-specific side.
	set_ram_write_enable(BIT(val, CTRL_WRITE_ENABLE_RAM));
}

u8 heath_h17_fdc_base_device::read(offs_t offset)
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

u8 heath_h17_fdc_base_device::floppy_status_r()
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

		// Write protect, like TK00 above, is active low from the drive and
		// inverted on the board, so bit 2 reads 1 for a protected disk.
		// wpt_r() is already in that sense - it is set when the image is read
		// only or its format cannot be saved - so it passes straight through.
		val |= m_floppy->wpt_r() ? 0x04 : 0x00;
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

void heath_h17_fdc_base_device::device_start()
{
	m_rx_timer = timer_alloc(FUNC(heath_h17_fdc_base_device::rx_timer_cb), this);

	save_item(NAME(m_motor_on));
	save_item(NAME(m_write_gate));
	save_item(NAME(m_tx_write_active));
	save_item(NAME(m_sync_char_received));
	save_item(NAME(m_rx_clock_cell));
	save_item(NAME(m_step_direction));
	save_item(NAME(m_side));
}

void heath_h17_fdc_base_device::device_reset()
{
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

TIMER_DEVICE_CALLBACK_MEMBER(heath_h17_fdc_base_device::tx_timer_cb)
{
	m_s2350->tcp_w();
}

void heath_h17_fdc_base_device::set_write_gate(bool write_gate)
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

void heath_h17_fdc_base_device::start_tx_write()
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

void heath_h17_fdc_base_device::stop_tx_write()
{
	if (!m_tx_write_active)
	{
		return;
	}

	m_tx_pll.stop_writing(m_floppy, m_tx_pll.ctime);
	m_tx_write_active = false;
}

void heath_h17_fdc_base_device::write_tx_cell(bool bit)
{
	attotime tm;
	m_tx_pll.write_next_bit(bit, tm, m_floppy, m_tx_pll.ctime + fm_cell_time());

	if (m_tx_pll.write_position >= 30)
	{
		m_tx_pll.commit(m_floppy, m_tx_pll.ctime);
	}
}

void heath_h17_fdc_base_device::tx_w(int state)
{
	if (!m_tx_write_active)
	{
		return;
	}

	write_tx_cell(true);
	write_tx_cell(BIT(state, 0));
}

// Wake when the PLL says the next half-cell ends.  feed_read_data() computes
// that boundary as ctime + period + phase_adjust and will not cross it, so any
// other choice either burns a callback that cannot make progress or, if it
// lands early, caps how far the PLL is allowed to stretch.
void heath_h17_fdc_base_device::schedule_rx_cell()
{
	if (!m_motor_on || !m_floppy)
	{
		m_rx_timer->adjust(attotime::never);
		return;
	}

	attotime const next = m_rx_pll.ctime + m_rx_pll.period + m_rx_pll.phase_adjust;
	attotime const now  = machine().time();

	m_rx_timer->adjust(next > now ? next - now : attotime::zero);
}

void heath_h17_fdc_base_device::reset_rx_pll()
{
	m_rx_pll.set_clock(fm_cell_time());
	m_rx_pll.read_reset(machine().time());
	m_rx_clock_cell = true;
	schedule_rx_cell();
}

void heath_h17_fdc_base_device::rx_cell(int bit)
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

TIMER_CALLBACK_MEMBER(heath_h17_fdc_base_device::rx_timer_cb)
{
	if (!m_motor_on || !m_floppy)
	{
		m_rx_timer->adjust(attotime::never);
		return;
	}

	// Take every half-cell that has ended.  Stopping at the current time rather
	// than a fixed cell ahead lets the PLL settle on whatever period the media
	// actually has; the old fixed limit made it return -1 whenever it wanted to
	// stretch, and it only got anywhere because the free-running timer let
	// machine time drift past ctime on a later call.  Normally this takes
	// exactly one cell, since the timer was armed at that cell's boundary.
	attotime tm;
	int bit;

	while ((bit = m_rx_pll.get_next_bit(tm, m_floppy, machine().time())) >= 0)
	{
		rx_cell(bit);
	}

	schedule_rx_cell();
}

void heath_h17_fdc_base_device::floppy_formats(format_registration &fr)
{
	fr.add(FLOPPY_H17D_FORMAT);
	fr.add(FLOPPY_H17D_V1_FORMAT);
	fr.add(FLOPPY_H8D_FORMAT);
	fr.add(FLOPPY_MFI_FORMAT);
}

void heath_h17_fdc_base_device::device_add_mconfig(machine_config &config)
{
	S2350(config, m_s2350);
	m_s2350->tx_handler().set(FUNC(heath_h17_fdc_base_device::tx_w));
	m_s2350->sync_character_received_cb().set(FUNC(heath_h17_fdc_base_device::sync_character_received));

	for (int i = 0; i < MAX_FLOPPY_DRIVES; i++)
	{
		FLOPPY_CONNECTOR(config, m_floppies[i], h17_floppies, "ssdd", heath_h17_fdc_base_device::floppy_formats);
		m_floppies[i]->set_sectoring_type(floppy_image::H10);
		m_floppies[i]->enable_sound(true);
	}

	TIMER(config, m_tx_timer).configure_generic(FUNC(heath_h17_fdc_base_device::tx_timer_cb));
}

void heath_h17_fdc_base_device::sync_character_received(int state)
{
	LOGFUNC("%s: state: %d\n", FUNCNAME, state);

	m_sync_char_received = bool(BIT(state, 0));
}
