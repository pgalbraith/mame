// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-DCDD floppy disk controller

    The Altair's 8-inch floppy system: two controller boards in the Altair
    and a Pertec FD-400 drive in a cabinet of its own, up to 16 drives on one
    cable. Disks are hard sectored, 32 sectors and an index hole, turn at
    360 RPM, and are recorded in FM at 250,000 bits a second. There is no
    sector header and no CRC: software is handed a byte every 32us and does
    everything else itself.

    The controller takes three ports, fixed at 010-012 octal. Status bits
    read 0 when true, and every bit reads 1 unless a drive is enabled.
    - OUT 010: D3-D0 select the drive, D7 disables the controller.
    - IN 010: D0 ENWD (ready for a byte to write), D1 MH (the head may
      move), D2 HS (the head is loaded and settled), D5 INTE, D6 track 0,
      D7 NRDA (a byte has been read). D3 and D4 read 0; MITS software takes
      D3 at 0 to mean the controller is enabled.
    - OUT 011: D0 step in, D1 step out, D2 load the head, D3 unload it,
      D4 enable the interrupt, D5 disable it, D6 head current switch,
      D7 write enable.
    - IN 011: D0 is 0 for 30us at the start of each sector, D5-D1 are the
      sector number. It reads all 1s until the head has settled.
    - 012: data, read on NRDA and written on ENWD.

    A sector starts at its hole. The read circuit is held clear for a short
    delay, then takes the first data 1 as the sync bit, which is also the top
    bit of the first byte, and passes on a byte every eight bits after it. If
    write enable has been set by the end of a longer delay, the write circuit
    starts instead: ENWD asks for a byte every 32us, and the controller goes
    on writing whichever byte it was given last until the next hole.

    MITS lengthened those two delays in 1977, from 140us and 280us to 214us
    and 389us, to put the data nearer the middle of the sector. Disks written
    that way are marked NWD and read on either board. Both timings are here as
    a configuration setting.

    Not emulated
    - INTE status, D5 of IN 010, which comes from an Altair bus line this
      S-100 bus does not carry; it reads as interrupts disabled
    - the sector interrupt is a 30us pulse from the hole; how the board
      latches and clears it has not been traced
    - the head current switch, which only changes the write current
    - the drive's five seconds of spin-up after its door closes
    - drive addresses 4 to 15

    References
    - Altair 88-DCDD manual: controller I/O information and read/write timing
      on printed pages 133-138, one-shot timings on page 128, schematics
      [https://deramp.com/downloads/altair/hardware/8_inch_floppy/Altair%20Floppy%20(88-DCDD)%20Manual.pdf]
    - MITS memo, System Timing Modification for Altair 88-DCDD Floppy Disk,
      2 September 1977, in the same manual
    - Mike Douglas, Disk IO Timing, on how Altair software reads and writes
      a sector [https://deramp.com/downloads/altair/software/8_inch_floppy/Disk%20IO%20Timing.pdf]

**********************************************************************/

#include "emu.h"
#include "mitsdcdd.h"

#include "formats/mits_dsk.h"
#include "imagedev/floppy.h"

#include "softlist_dev.h"

#define LOG_REG (1U << 1)

//#define VERBOSE (LOG_GENERAL | LOG_REG)
#include "logmacro.h"

#define LOGREG(...) LOGMASKED(LOG_REG, __VA_ARGS__)


namespace {

class s100_mits_dcdd_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_dcdd_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_sinp_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;

private:
	static constexpr unsigned DRIVES = 4;

	// FM at 250,000 bits a second, a byte every eight cells
	static attotime cell_time() { return attotime::from_usec(4); }
	static attotime half_cell_time() { return attotime::from_usec(2); }
	static attotime byte_time() { return attotime::from_usec(32); }

	// Board 1's read clock mask and read data window one-shots, 0.7-1.2us and
	// 2.6-2.9us: a pulse inside the window after a clock is a data 1, and the
	// next one after it is the next clock.
	static attotime read_clock_mask() { return attotime::from_usec(1); }
	static attotime read_data_window() { return attotime::from_nsec(2750); }

	static attotime sector_true_time() { return attotime::from_usec(30); }
	static attotime index_window() { return attotime::from_nsec(3'900'000); }
	static attotime head_settle_time() { return attotime::from_msec(40); }
	static attotime trim_erase_end_time() { return attotime::from_usec(475); }

	static void floppy_formats(format_registration &fr);

	bool enabled() const { return m_floppy && m_floppy->exists(); }
	bool head_settled() const;
	bool head_may_move() const;
	bool sector_true() const;

	u8 status_r();
	u8 sector_r();
	u8 data_r();
	void select_w(u8 data);
	void control_w(u8 data);
	void data_w(u8 data);

	void floppy_load(floppy_image_device *floppy);
	void index_pulse(floppy_image_device *floppy, int state);
	void start_sector(attotime const &when);
	void stop_reading();
	void write_byte(attotime const &when);
	void stop_writing(attotime const &when);
	void set_irq(bool state);

	TIMER_CALLBACK_MEMBER(read_cb);
	TIMER_CALLBACK_MEMBER(write_start_cb);
	TIMER_CALLBACK_MEMBER(write_byte_cb);
	TIMER_CALLBACK_MEMBER(irq_end_cb);

	required_device_array<floppy_connector, DRIVES> m_connectors;
	required_ioport m_timing;
	required_ioport m_irq_jumper;

	emu_timer *m_read_timer;
	emu_timer *m_write_start_timer;
	emu_timer *m_write_byte_timer;
	emu_timer *m_irq_timer;

	floppy_image_device *m_floppy;  // the enabled drive
	s8 m_drive;                     // its number, or -1

	bool m_nwd;                     // board 1 has the 1977 read and write delays
	u8 m_irq_route;                 // 0 not connected, 1 PINT, 2-9 VI0-VI7
	bool m_irq_state;

	// sector and index circuits, with a count for each drive
	u8 m_sector[DRIVES];
	bool m_index_seen[DRIVES];      // the drive's next sector hole is sector 0
	attotime m_hole_time[DRIVES];   // when its last sector hole passed
	attotime m_sector_time;         // when the enabled drive's sector started

	// head and drive control
	bool m_head_loaded;
	bool m_unload_after_write;
	bool m_int_enable;
	attotime m_head_settle_time;    // HS goes true
	attotime m_head_load_time;
	attotime m_step_time;
	attotime m_trim_erase_end;

	// read circuit
	attotime m_rx_scan;             // how far the flux has been decoded
	attotime m_rx_clock;            // the last clock pulse
	bool m_rx_have_clock;
	bool m_rx_data_pulse;           // a data pulse followed that clock
	bool m_rx_synced;
	u8 m_rx_bits;
	u8 m_rx_shift;
	bool m_rx_pending;              // a byte is decoded and due at the read timer
	u8 m_rx_pending_byte;
	u8 m_rx_data;
	bool m_nrda;

	// write circuit
	bool m_write_enable;
	bool m_writing;
	u8 m_tx_data;
	bool m_enwd;
};


s100_mits_dcdd_device::s100_mits_dcdd_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_DCDD, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_connectors(*this, "floppy%u", 0U)
	, m_timing(*this, "TIMING")
	, m_irq_jumper(*this, "IRQ")
	, m_read_timer(nullptr)
	, m_write_start_timer(nullptr)
	, m_write_byte_timer(nullptr)
	, m_irq_timer(nullptr)
	, m_floppy(nullptr)
	, m_drive(-1)
	, m_nwd(true)
	, m_irq_route(0)
	, m_irq_state(false)
	, m_head_loaded(false)
	, m_unload_after_write(false)
	, m_int_enable(false)
	, m_rx_have_clock(false)
	, m_rx_data_pulse(false)
	, m_rx_synced(false)
	, m_rx_bits(0)
	, m_rx_shift(0)
	, m_rx_pending(false)
	, m_rx_pending_byte(0)
	, m_rx_data(0)
	, m_nrda(false)
	, m_write_enable(false)
	, m_writing(false)
	, m_tx_data(0)
	, m_enwd(false)
{
}


void s100_mits_dcdd_device::device_start()
{
	m_read_timer = timer_alloc(FUNC(s100_mits_dcdd_device::read_cb), this);
	m_write_start_timer = timer_alloc(FUNC(s100_mits_dcdd_device::write_start_cb), this);
	m_write_byte_timer = timer_alloc(FUNC(s100_mits_dcdd_device::write_byte_cb), this);
	m_irq_timer = timer_alloc(FUNC(s100_mits_dcdd_device::irq_end_cb), this);

	for (auto &connector : m_connectors)
	{
		if (floppy_image_device *const floppy = connector->get_device())
		{
			floppy->setup_load_cb(floppy_image_device::load_cb(&s100_mits_dcdd_device::floppy_load, this));
			floppy->setup_index_pulse_cb(floppy_image_device::index_pulse_cb(&s100_mits_dcdd_device::index_pulse, this));
		}
	}

	for (unsigned drive = 0; drive < DRIVES; drive++)
	{
		m_sector[drive] = 0;
		m_index_seen[drive] = false;
		m_hole_time[drive] = attotime::never;
	}
	m_sector_time = attotime::never;
	m_head_settle_time = attotime::never;
	m_head_load_time = attotime::never;
	m_step_time = attotime::never;
	m_trim_erase_end = attotime::zero;
	m_rx_scan = attotime::zero;
	m_rx_clock = attotime::zero;

	save_item(NAME(m_drive));
	save_item(NAME(m_nwd));
	save_item(NAME(m_irq_route));
	save_item(NAME(m_irq_state));
	save_item(NAME(m_sector));
	save_item(NAME(m_index_seen));
	save_item(NAME(m_hole_time));
	save_item(NAME(m_sector_time));
	save_item(NAME(m_head_loaded));
	save_item(NAME(m_unload_after_write));
	save_item(NAME(m_int_enable));
	save_item(NAME(m_head_settle_time));
	save_item(NAME(m_head_load_time));
	save_item(NAME(m_step_time));
	save_item(NAME(m_trim_erase_end));
	save_item(NAME(m_rx_scan));
	save_item(NAME(m_rx_clock));
	save_item(NAME(m_rx_have_clock));
	save_item(NAME(m_rx_data_pulse));
	save_item(NAME(m_rx_synced));
	save_item(NAME(m_rx_bits));
	save_item(NAME(m_rx_shift));
	save_item(NAME(m_rx_pending));
	save_item(NAME(m_rx_pending_byte));
	save_item(NAME(m_rx_data));
	save_item(NAME(m_nrda));
	save_item(NAME(m_write_enable));
	save_item(NAME(m_writing));
	save_item(NAME(m_tx_data));
	save_item(NAME(m_enwd));
}

void s100_mits_dcdd_device::device_reset()
{
	if (m_writing)
		stop_writing(machine().time());

	m_nwd = BIT(m_timing->read(), 0);

	if (m_irq_state)
		set_irq(false);
	m_irq_route = m_irq_jumper->read();

	// power on clear leaves the controller disabled
	m_floppy = nullptr;
	m_drive = -1;
	m_head_loaded = false;
	m_unload_after_write = false;
	m_int_enable = false;
	m_write_enable = false;
	m_enwd = false;
	m_sector_time = attotime::never;
	stop_reading();
	m_write_start_timer->adjust(attotime::never);
	m_irq_timer->adjust(attotime::never);
}

void s100_mits_dcdd_device::device_post_load()
{
	m_floppy = (m_drive >= 0) ? m_connectors[m_drive]->get_device() : nullptr;
}


u8 s100_mits_dcdd_device::s100_sinp_r(offs_t offset)
{
	switch (offset & 0xff)
	{
	case 0x08: return status_r();
	case 0x09: return sector_r();
	case 0x0a: return data_r();
	default:   return 0xff;
	}
}

void s100_mits_dcdd_device::s100_sout_w(offs_t offset, u8 data)
{
	switch (offset & 0xff)
	{
	case 0x08: select_w(data); break;
	case 0x09: control_w(data); break;
	case 0x0a: data_w(data); break;
	}
}


bool s100_mits_dcdd_device::head_settled() const
{
	return m_head_loaded && (machine().time() >= m_head_settle_time);
}

// After a step MH is false for 10ms, true for 1ms so the head can be stepped
// again straight away, then false for 20ms while the head settles.
bool s100_mits_dcdd_device::head_may_move() const
{
	attotime const now = machine().time();

	if (m_writing || (now < m_trim_erase_end))
		return false;

	if (!m_head_load_time.is_never() && (now < m_head_load_time + head_settle_time()))
		return false;

	if (!m_step_time.is_never())
	{
		attotime const since = now - m_step_time;
		if (since < attotime::from_msec(10))
			return false;
		if ((since >= attotime::from_msec(11)) && (since < attotime::from_msec(31)))
			return false;
	}

	return true;
}

bool s100_mits_dcdd_device::sector_true() const
{
	return !m_sector_time.is_never() && ((machine().time() - m_sector_time) < sector_true_time());
}


u8 s100_mits_dcdd_device::status_r()
{
	if (!enabled())
		return 0xff;

	// D3 and D4 read 0, and INTE reads as disabled
	u8 data = 0x20;
	if (!m_enwd)
		data |= 0x01;
	if (!head_may_move())
		data |= 0x02;
	if (!head_settled())
		data |= 0x04;
	if (m_floppy->trk00_r())
		data |= 0x40;
	if (!m_nrda)
		data |= 0x80;
	return data;
}

u8 s100_mits_dcdd_device::sector_r()
{
	if (!enabled() || !head_settled())
		return 0xff;

	return 0xc0 | (m_sector[m_drive] << 1) | (sector_true() ? 0x00 : 0x01);
}

u8 s100_mits_dcdd_device::data_r()
{
	if (!enabled())
		return 0xff;

	if (!machine().side_effects_disabled())
		m_nrda = false;
	return m_rx_data;
}

void s100_mits_dcdd_device::select_w(u8 data)
{
	LOGREG("select %02x\n", data);

	s8 drive = -1;
	floppy_image_device *floppy = nullptr;
	if (!BIT(data, 7) && ((data & 0x0f) < DRIVES))
	{
		// a drive with no disk, or its door open, cannot be enabled
		drive = data & 0x0f;
		floppy = m_connectors[drive]->get_device();
		if (!floppy || !floppy->exists())
		{
			drive = -1;
			floppy = nullptr;
		}
	}

	if (floppy == m_floppy)
		return;

	// clearing disk control unloads the head and turns the interrupt off
	if (m_writing)
		stop_writing(machine().time());
	m_floppy = floppy;
	m_drive = drive;
	m_head_loaded = false;
	m_unload_after_write = false;
	m_int_enable = false;
	m_write_enable = false;
	m_enwd = false;
	m_sector_time = attotime::never;
	stop_reading();
	m_write_start_timer->adjust(attotime::never);
	if (m_irq_state)
		set_irq(false);
}

void s100_mits_dcdd_device::control_w(u8 data)
{
	LOGREG("control %02x\n", data);

	if (!enabled())
		return;

	attotime const now = machine().time();

	if (BIT(data, 0) || BIT(data, 1))
	{
		// D1 steps out, towards track 0
		m_floppy->dir_w(BIT(data, 1));
		m_floppy->stp_w(1);
		m_floppy->stp_w(0);
		m_step_time = now;
		if (m_head_loaded)
			m_head_settle_time = now + head_settle_time();
		stop_reading();
	}

	if (BIT(data, 2) && !m_head_loaded)
	{
		m_head_loaded = true;
		m_head_load_time = now;
		m_head_settle_time = now + head_settle_time();
	}

	// a write in progress holds the head loaded until it finishes
	if (BIT(data, 3))
	{
		if (m_writing)
		{
			m_unload_after_write = true;
		}
		else
		{
			m_head_loaded = false;
			stop_reading();
		}
	}

	if (BIT(data, 4))
		m_int_enable = true;
	if (BIT(data, 5))
	{
		m_int_enable = false;
		if (m_irq_state)
			set_irq(false);
	}

	if (BIT(data, 7))
		m_write_enable = true;
}

void s100_mits_dcdd_device::data_w(u8 data)
{
	if (!enabled())
		return;

	m_tx_data = data;
	m_enwd = false;
}


// MAME starts a newly loaded disk turning at sector 0's hole, so its count
// starts there rather than a revolution later at the index hole.
void s100_mits_dcdd_device::floppy_load(floppy_image_device *floppy)
{
	for (unsigned drive = 0; drive < DRIVES; drive++)
	{
		if (m_connectors[drive]->get_device() == floppy)
		{
			m_sector[drive] = 0;
			m_index_seen[drive] = false;
			m_hole_time[drive] = machine().time();
		}
	}
}

// Every hole gives a pulse. The index hole sits halfway between sectors 31
// and 0, so a hole inside the index window after a sector hole is the index.
// A drive's count goes on while the controller is disabled or another drive
// is selected: DBL takes the first sector numbered 0 once it has enabled the
// controller, and the RTC driver disk disables the controller before booting
// through DBL again, so a count that stopped would hand DBL the wrong sectors.
// How the board itself keeps count across a change of drive is not traced.
void s100_mits_dcdd_device::index_pulse(floppy_image_device *floppy, int state)
{
	if (!state)
		return;

	unsigned drive = 0;
	while ((drive < DRIVES) && (m_connectors[drive]->get_device() != floppy))
		drive++;
	if (drive == DRIVES)
		return;

	attotime const now = machine().time();
	if (!m_hole_time[drive].is_never() && ((now - m_hole_time[drive]) < index_window()))
	{
		m_index_seen[drive] = true;
		return;
	}

	m_sector[drive] = m_index_seen[drive] ? 0 : ((m_sector[drive] + 1) & 0x1f);
	m_index_seen[drive] = false;
	m_hole_time[drive] = now;

	if (floppy == m_floppy)
		start_sector(now);
}

void s100_mits_dcdd_device::start_sector(attotime const &when)
{
	m_sector_time = when;

	// the write circuit stops at the hole, and write enable is cleared at the
	// start of every sector
	if (m_writing)
		stop_writing(when);
	m_write_enable = false;

	// the read circuit is held clear for the read delay, then looks for sync
	stop_reading();
	m_rx_scan = when + attotime::from_usec(m_nwd ? 214 : 140);
	if (m_head_loaded)
		m_read_timer->adjust(m_rx_scan - machine().time());

	m_write_start_timer->adjust(attotime::from_usec(m_nwd ? 389 : 280));

	if (m_int_enable)
	{
		set_irq(true);
		m_irq_timer->adjust(sector_true_time());
	}
}

void s100_mits_dcdd_device::stop_reading()
{
	m_read_timer->adjust(attotime::never);
	m_rx_have_clock = false;
	m_rx_data_pulse = false;
	m_rx_synced = false;
	m_rx_bits = 0;
	m_rx_shift = 0;
	m_rx_pending = false;
	m_nrda = false;
}


// Decodes the flux from where it left off, until a byte is complete. The
// timer is set for when that byte is complete, so NRDA rises at the right
// moment however far ahead the decoding has looked.
TIMER_CALLBACK_MEMBER(s100_mits_dcdd_device::read_cb)
{
	if (!enabled() || !m_head_loaded || m_writing)
		return;

	attotime const now = machine().time();

	if (m_rx_pending)
	{
		m_rx_pending = false;
		m_rx_data = m_rx_pending_byte;
		m_nrda = true;
	}

	for (;;)
	{
		attotime const when = m_floppy->get_next_transition(m_rx_scan);
		if (when.is_never())
			return;
		m_rx_scan = when;

		if (!m_rx_have_clock)
		{
			m_rx_have_clock = true;
			m_rx_clock = when;
			m_rx_data_pulse = false;
			continue;
		}

		attotime const since = when - m_rx_clock;
		if (since < read_clock_mask())
			continue;
		if (since < read_data_window())
		{
			m_rx_data_pulse = true;
			continue;
		}

		// a clock pulse, which settles the cell before it
		bool const bit = m_rx_data_pulse;
		m_rx_clock = when;
		m_rx_data_pulse = false;

		if (!m_rx_synced)
		{
			if (!bit)
				continue;
			m_rx_synced = true;
			m_rx_shift = 1;
			m_rx_bits = 1;
			continue;
		}

		m_rx_shift = (m_rx_shift << 1) | (bit ? 1 : 0);
		if (++m_rx_bits == 8)
		{
			m_rx_bits = 0;
			if (when <= now)
			{
				m_rx_data = m_rx_shift;
				m_nrda = true;
			}
			else
			{
				m_rx_pending = true;
				m_rx_pending_byte = m_rx_shift;
				m_read_timer->adjust(when - now);
				return;
			}
		}
	}
}


TIMER_CALLBACK_MEMBER(s100_mits_dcdd_device::write_start_cb)
{
	if (!enabled() || !m_head_loaded || !m_write_enable)
		return;

	attotime const now = machine().time();

	stop_reading();
	m_writing = true;
	m_floppy->write_start(now);
	write_byte(now);
	m_write_byte_timer->adjust(byte_time());
}

TIMER_CALLBACK_MEMBER(s100_mits_dcdd_device::write_byte_cb)
{
	if (!m_writing)
		return;

	write_byte(machine().time());
	m_write_byte_timer->adjust(byte_time());
}

// Shifts out the byte last written, a clock pulse at the start of each cell
// and a data pulse in the middle of it for a 1, and asks for the next byte.
void s100_mits_dcdd_device::write_byte(attotime const &when)
{
	for (int bit = 0; bit < 8; bit++)
	{
		attotime const cell = when + (cell_time() * bit);
		m_floppy->write_flux_change(cell);
		if (BIT(m_tx_data, 7 - bit))
			m_floppy->write_flux_change(cell + half_cell_time());
	}
	m_enwd = true;
}

void s100_mits_dcdd_device::stop_writing(attotime const &when)
{
	m_write_byte_timer->adjust(attotime::never);
	if (m_floppy)
		m_floppy->write_end(when);
	m_writing = false;
	m_enwd = false;
	m_write_enable = false;
	m_trim_erase_end = when + trim_erase_end_time();

	if (m_unload_after_write)
	{
		m_unload_after_write = false;
		m_head_loaded = false;
	}
}


void s100_mits_dcdd_device::set_irq(bool state)
{
	m_irq_state = state;
	switch (m_irq_route)
	{
	case 1: m_bus->irq_w(state); break;
	case 2: m_bus->vi0_w(state); break;
	case 3: m_bus->vi1_w(state); break;
	case 4: m_bus->vi2_w(state); break;
	case 5: m_bus->vi3_w(state); break;
	case 6: m_bus->vi4_w(state); break;
	case 7: m_bus->vi5_w(state); break;
	case 8: m_bus->vi6_w(state); break;
	case 9: m_bus->vi7_w(state); break;
	}
}

TIMER_CALLBACK_MEMBER(s100_mits_dcdd_device::irq_end_cb)
{
	if (m_irq_state)
		set_irq(false);
}


static void mits_dcdd_floppies(device_slot_interface &device)
{
	device.option_add("8sssd", FLOPPY_8_SSSD);
}

void s100_mits_dcdd_device::floppy_formats(format_registration &fr)
{
	fr.add_fm_containers();
	fr.add(FLOPPY_MITS_DSK_FORMAT);
}

void s100_mits_dcdd_device::device_add_mconfig(machine_config &config)
{
	// the 88-DCDD came with one drive; each 88-DISC added another
	for (unsigned i = 0; i < DRIVES; i++)
	{
		FLOPPY_CONNECTOR(config, m_connectors[i], mits_dcdd_floppies, i ? nullptr : "8sssd", floppy_formats).enable_sound(true);
		m_connectors[i]->set_sectoring_type(floppy_image::H32);
	}

	SOFTWARE_LIST(config, "flop_list").set_original("88dcdd_flop");
}


static INPUT_PORTS_START( mits_dcdd )
	PORT_START("TIMING")
	PORT_CONFNAME(0x01, 0x01, "Board 1 read and write delays")
	PORT_CONFSETTING(0x00, "Original, 140us and 280us")
	PORT_CONFSETTING(0x01, "NWD, 214us and 389us")

	PORT_START("IRQ")
	PORT_CONFNAME(0x0f, 0x00, "Sector interrupt")
	PORT_CONFSETTING(0x00, "Not connected")
	PORT_CONFSETTING(0x01, "PINT")
	PORT_CONFSETTING(0x02, "VI0")
	PORT_CONFSETTING(0x03, "VI1")
	PORT_CONFSETTING(0x04, "VI2")
	PORT_CONFSETTING(0x05, "VI3")
	PORT_CONFSETTING(0x06, "VI4")
	PORT_CONFSETTING(0x07, "VI5")
	PORT_CONFSETTING(0x08, "VI6")
	PORT_CONFSETTING(0x09, "VI7")
INPUT_PORTS_END

ioport_constructor s100_mits_dcdd_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_dcdd);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_DCDD, device_s100_card_interface, s100_mits_dcdd_device, "s100_mits_dcdd", "MITS 88-DCDD Floppy Disk Controller")
