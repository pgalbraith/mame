// license:BSD-3-Clause
// copyright-holders:Paul Galbraith, AJR
/***************************************************************************

    MITS Altair 8800b and Altair 8800

    The original Altair, sold from January 1975: an 8080 CPU board, a front
    panel of toggle switches and LEDs, and S-100 slots for everything else.
    There is no ROM. Programs are toggled in at the panel, or a short loader
    is toggled in to read the rest from a serial port or cassette.

    Using the front panel
    - An address/data switch is up for 1. The top eight are also the sense
      switches, which a program reads with IN 377 (octal).
    - EXAMINE loads the address switches and shows that location; EXAMINE
      NEXT moves on one. DEPOSIT stores the low eight switches at the
      current location; DEPOSIT NEXT moves on one first.
    - RUN starts at the current location. SINGLE STEP, EXAMINE, DEPOSIT and
      PROTECT only work while the machine is stopped. RESET works any time,
      and restarts a running program from 000000.
    - STOP cannot stop a CPU that has executed HLT, on this machine or the
      real one. Hold STOP, press and release RESET, then release STOP.

    Default cards
    - an 88-2SIO at 020 octal, with a terminal on port 0 at 9600 baud
    - an 88-16MCS answering at 000000-037777
    Altair BASIC 4.0 picks its terminal from the sense switches, and all of
    them down means an 88-2SIO with two stop bits, which suits the defaults.
    For the Teletype ASR-33 that MITS sold with the machine, choose asr33
    for the 88-2SIO's port 0 and set that port's baud rate jumper to 110.

    The 88-DCDD 8-inch floppy controller is the dcdd card. It boots from the
    DBL PROM on the pmc card, an 88-PMC: set the address switches to 177400,
    EXAMINE, then RUN. RAM must leave the PMC's 174000-177777 alone, and
    Altair DOS stops with INSUFFICIENT MEMORY when RAM fills all of the rest;
    three 88-16MCS set to 000000, 040000 and 100000 suit it.

    The 88-HDSK hard disk is the hdsk card, an 88-4PIO at 240 octal with the
    Datakeeper controller and a Pertec D3422 behind it: hard1 is the
    removable cartridge and hard2 the fixed platter. Its HD-TBL boot loader
    is also on the pmc card: set the address switches to 176000, EXAMINE,
    put the switches down, then RUN.

    Where this differs from the real panel
    - SINGLE STEP runs one instruction, not one machine cycle, because the
      8080 core cannot pause partway through an instruction.
    - The real panel does not reset its RUN/STOP flip-flop at power-on, so
      the operator has to raise STOP and RESET before doing anything. Here
      the machine powers on already stopped at 000000.

    Altair 8800b
    - The 1976 redesign, on the same S-100 bus. Its CPU board adds an 8224
      clock generator, and its front panel is run by microcode in a 1702A
      PROM that the 8080 cannot see.
    - The 8800b panel stops the CPU and puts data onto it in a different
      way, so neither panel works with the other machine's CPU board.
    - SINGLE STEP's down position is SLOW. Hold it and the machine single
      steps until it is let go: 7.6, 1.9 or 0.48 times a second, set by the
      SLOW speed jumper. Those rates come from the display/control
      schematic, where the steps are taken from a counter clocked by phi2
      at 2 MHz. The manual's text says 786 ms, and "approximately 2 cycles
      per second" in its switch table; neither fits the counter as drawn.
    - The two AUX switches are its accumulator switches. DISPLAY shows the
      accumulator on the data LEDs and LOAD sets it from A7-A0. INPUT and
      OUTPUT move it from or to the I/O channel set on A15-A8.
    - Its layout is drawn from a photograph of an 8800b panel.
    - Not working: EXAMINE, DEPOSIT and the data LEDs behave as they do on
      the original panel, not through the 8800b's PROM and its interface
      card's data latch.

    References
    - Altair 8800 Theory of Operation, pages 3-8 cover the CPU board and the
      panel [https://altairclone.com/downloads/manuals/Altair%208800%20Theory%20of%20Operation.pdf]
    - MITS schematics 880-101 (CPU), 880-105 (panel display) and 880-106
      (panel control) [https://deramp.com/downloads/altair/hardware/altair_8800_computer/Altair%20Schematics.pdf]
    - Altair 8800 Operator's Manual [https://altairclone.com/downloads/manuals/Altair%208800%20Operator's%20Manual.pdf]
    - Altair BASIC manual, January 1977, appendix B for the sense switches
      [https://altairclone.com/downloads/manuals/BASIC%20Manual%2077.pdf]
    - Altair 8800c Front Panel Manual, pages 1-2, for how the 8800b panel and
      CPU board differ [https://deramp.com/downloads/altair/hardware/altair_8800c/Front%20Panel%20Manual.pdf]
    - MITS Altair 8800b documentation, April 1977: Table 2-1 for the
      switches, section 3-32 on page 3-72 for SLOW, section 3-33 on page
      3-73 for RESET/EXT CLR, Table 3-2 on pages 3-76 to 3-78 for the
      panel PROM's programs, section 3-40 on page 3-88 for the SLOW speed
      jumpers, and Figure 3-16 sheet 1 for the counter those jumpers tap
      [https://www.manualslib.com/manual/1574204/Mits-Altair-8800b.html]

***************************************************************************/

#include "emu.h"

#include "bus/s100/s100.h"
#include "bus/s100/mits2sio.h"
#include "bus/s100/mits4pio.h"
#include "bus/s100/mitsc700.h"
#include "bus/s100/mitsdcdd.h"
#include "bus/s100/mitslpc.h"
#include "bus/s100/mitspio.h"
#include "bus/s100/mitspmc.h"
#include "bus/s100/mitsram.h"
#include "bus/s100/mitssio.h"
#include "bus/s100/mitsvi.h"
#include "cpu/i8085/i8085.h"
#include "imagedev/snapquik.h"

#include "al8800.lh"
#include "al8800b.lh"


namespace {

class al8800b_state : public driver_device
{
public:
	al8800b_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_bus(*this, "s100")
		, m_switches(*this, "SWITCHES")
		, m_jumpers(*this, "JUMPERS")
		, m_leds(*this, "%u.%u", 0U, 0U)
		, m_levers(*this, "lever%u", 0U)
	{
	}

	void al8800(machine_config &config) ATTR_COLD;
	void al8800b(machine_config &config) ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(control_changed);
	DECLARE_INPUT_CHANGED_MEMBER(accumulator_changed);

	// Bits of the CONTROLS port. The eight control switches take two bits
	// each, up position then down, so a bit number divided by two is the
	// switch it belongs to. SINGLE STEP's down position is SLOW on the
	// 8800b and does nothing on the original 8800.
	enum : u8
	{
		CTRL_STOP = 0,
		CTRL_RUN,
		CTRL_SINGLE_STEP,
		CTRL_SLOW,
		CTRL_EXAMINE,
		CTRL_EXAMINE_NEXT,
		CTRL_DEPOSIT,
		CTRL_DEPOSIT_NEXT,
		CTRL_RESET,
		CTRL_CLR,
		CTRL_PROTECT,
		CTRL_UNPROTECT,
		CTRL_AUX1_UP,
		CTRL_AUX1_DOWN,
		CTRL_AUX2_UP,
		CTRL_AUX2_DOWN,

		// on the 8800b the two AUX switches are its accumulator switches
		CTRL_ACC_DISPLAY = CTRL_AUX1_UP,
		CTRL_ACC_LOAD = CTRL_AUX1_DOWN,
		CTRL_ACC_INPUT = CTRL_AUX2_UP,
		CTRL_ACC_OUTPUT = CTRL_AUX2_DOWN
	};

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	// the 8080 status words the panel cares about
	static constexpr u8 STATUS_FETCH = i8080_cpu_device::STATUS_MEMR | i8080_cpu_device::STATUS_M1 | i8080_cpu_device::STATUS_WO;
	static constexpr u8 STATUS_HALT = i8080_cpu_device::STATUS_MEMR | i8080_cpu_device::STATUS_HLTA | i8080_cpu_device::STATUS_WO;

	// rows of LEDs, which are also the rows of the "row.bit" layout outputs
	enum : u8
	{
		ROW_ADDRESS = 0,
		ROW_DATA,
		ROW_STATUS,     // the 8080 status latch, which lines up bit for bit with MEMR...INT
		ROW_OTHER
	};

	// bits of ROW_OTHER
	enum : u8
	{
		LED_INTE = 0,
		LED_PROT,
		LED_WAIT,
		LED_HLDA
	};

	void common(machine_config &config) ATTR_COLD;
	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 mem_r(offs_t offset);
	void mem_w(offs_t offset, u8 data);
	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);

	void status_w(u8 data);
	void inte_w(int state);

	bool stopped() const { return !m_run && !m_stepping; }
	void stop_now();
	void single_step();
	TIMER_CALLBACK_MEMBER(show_stopped_cb) { show_stopped(); }
	TIMER_CALLBACK_MEMBER(slow_cb) { single_step(); }

	void show_bus(offs_t address, u8 data) { m_rows[ROW_ADDRESS] = address; m_rows[ROW_DATA] = data; }
	void show_lever(u8 bit, bool pressed) { m_levers[bit / 2] = pressed ? (BIT(bit, 0) ? 2 : 1) : 0; }
	void show_prot(bool state);
	void show_other();
	void show_stopped();
	void count_cycle();
	TIMER_CALLBACK_MEMBER(update_leds);

	DECLARE_QUICKLOAD_LOAD_MEMBER(quickload_cb);

	required_device<i8080_cpu_device> m_maincpu;
	required_device<s100_bus_device> m_bus;
	required_ioport m_switches;
	optional_ioport m_jumpers;      // the 8800b's SLOW speed jumper
	output_finder<4, 16> m_leds;    // brightness level 0-4
	output_finder<8> m_levers;      // control switch positions for the layout: 0 centre, 1 up, 2 down

	emu_timer *m_show_stopped_timer = nullptr;
	emu_timer *m_led_timer = nullptr;
	emu_timer *m_slow_timer = nullptr;

	u8 m_status = 0;            // copy of the 8080's status latch
	bool m_inte = false;
	bool m_prot = false;        // PS* from the memory board being read
	bool m_halted = false;      // executed HLT, so there is no SYNC for STOP to catch
	bool m_run = false;         // the panel's RUN/STOP flip-flop
	bool m_stepping = false;    // the panel's SINGLE STEP flip-flop
	bool m_stop_held = false;
	u16 m_rows[4] = { };        // what each row of LEDs is being driven with now

	// for LED brightness, not saved
	u32 m_cycles = 0;
	u32 m_lit[4][16] = { };
	double m_brightness[4][16] = { };
};


void al8800b_state::mem_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(al8800b_state::mem_r), FUNC(al8800b_state::mem_w));
}

void al8800b_state::io_map(address_map &map)
{
	map(0x00, 0xff).rw(FUNC(al8800b_state::io_r), FUNC(al8800b_state::io_w));
}


u8 al8800b_state::mem_r(offs_t offset)
{
	u8 const data = m_bus->smemr_r(offset);

	if (!machine().side_effects_disabled())
	{
		show_bus(offset, data);
		show_prot(!m_bus->ps_r(offset));
		count_cycle();

		// An opcode fetch is where STOP and SINGLE STEP take hold. A halted
		// CPU produces no SYNC, so neither can stop it.
		if ((m_status == STATUS_FETCH) && !m_halted && ((m_run && m_stop_held) || m_stepping))
			stop_now();
	}

	return data;
}

void al8800b_state::mem_w(offs_t offset, u8 data)
{
	m_bus->mwrt_w(offset, data);

	// nothing drives the Data In bus during a write, so its pull-ups read 377
	if (!machine().side_effects_disabled())
	{
		show_bus(offset, 0xff);
		show_prot(false);
		count_cycle();
	}
}

u8 al8800b_state::io_r(offs_t offset)
{
	// the 8080 puts the port number on both halves of the address bus
	offs_t const address = offset << 8 | offset;

	// The panel puts the sense switches straight onto the CPU's own data bus,
	// not the Data In bus, so the data LEDs do not see them.
	u8 const data = (offset == 0xff) ? u8(m_switches->read() >> 8) : m_bus->sinp_r(offset);

	if (!machine().side_effects_disabled())
	{
		show_bus(address, (offset == 0xff) ? 0xff : data);
		show_prot(false);
		count_cycle();
	}

	return data;
}

void al8800b_state::io_w(offs_t offset, u8 data)
{
	m_bus->sout_w(offset, data);

	if (!machine().side_effects_disabled())
	{
		show_bus(offset << 8 | offset, 0xff);
		show_prot(false);
		count_cycle();
	}
}


void al8800b_state::status_w(u8 data)
{
	// While halted the 8080 core keeps refetching the HLT opcode, which the
	// real CPU does not. Leave the LEDs on halt acknowledge rather than let
	// them flicker between that and a fetch.
	if (data == STATUS_HALT)
		m_halted = true;
	else if (data != STATUS_FETCH)
		m_halted = false;

	m_status = data;
	if (!m_halted || (data == STATUS_HALT))
		m_rows[ROW_STATUS] = data;
}

void al8800b_state::inte_w(int state)
{
	m_inte = bool(state);
	show_other();
}


// The 8080 core runs a whole instruction at a time, so the CPU cannot be held
// in the middle of a fetch the way the panel holds a real 8080. Suspending it
// here, while it reads an opcode, lets that instruction finish and stops the
// CPU before the next one. For SINGLE STEP that is one instruction, as
// intended; for STOP it lands one instruction later than the real panel,
// which nobody could time a switch closely enough to notice.
void al8800b_state::stop_now()
{
	m_run = false;
	m_stepping = false;
	m_maincpu->suspend(SUSPEND_REASON_HALT, true);

	// refresh the LEDs once the instruction has finished
	m_show_stopped_timer->adjust(attotime::zero);
}

void al8800b_state::single_step()
{
	if (stopped())
	{
		m_stepping = true;
		show_other();
		m_maincpu->resume(SUSPEND_REASON_HALT);
	}
}


void al8800b_state::show_prot(bool state)
{
	if (m_prot != state)
	{
		m_prot = state;
		show_other();
	}
}

void al8800b_state::show_other()
{
	u16 data = 0;
	if (m_inte)
		data |= 1U << LED_INTE;
	if (m_prot)
		data |= 1U << LED_PROT;
	if (stopped() && !m_halted)
		data |= 1U << LED_WAIT;
	m_rows[ROW_OTHER] = data;
}

void al8800b_state::show_stopped()
{
	// waiting in an opcode fetch: its address on the bus, memory's reply on Data In
	offs_t const pc = m_maincpu->pc();
	show_bus(pc, m_bus->smemr_r(pc));
	m_rows[ROW_STATUS] = m_halted ? STATUS_HALT : STATUS_FETCH;
	m_prot = !m_bus->ps_r(pc);
	show_other();
}


// LED BRIGHTNESS - the share of machine cycles each LED was lit for
// ------------------------------------------------------------------
// While a program runs, the LEDs follow bus signals that change every machine
// cycle, and what the eye sees is how long each one was on. Kill the Bit
// shows its moving light that way, through the address of a memory read
// repeated in a delay loop.
//
// This counts machine cycles rather than time. The 8080 core takes an
// instruction's whole time from the CPU's clock when it fetches the opcode,
// so timing the changes gives all of it to the fetch and none to the reads
// and writes that follow - which is exactly where Kill the Bit's light is.
// Every cycle counts the same, although a fetch is really four or five
// clocks long and a read three.
//
// When no cycles ran in a frame the machine is stopped, and the LEDs show
// what is on the bus.
void al8800b_state::count_cycle()
{
	m_cycles++;
	for (unsigned row = 0; row < 4; row++)
	{
		u16 const bits = m_rows[row];
		for (unsigned bit = 0; bit < 16; bit++)
			m_lit[row][bit] += BIT(bits, bit);
	}
}

TIMER_CALLBACK_MEMBER(al8800b_state::update_leds)
{
	// brightness thresholds for levels 1 to 4
	static constexpr double LEVELS[] = { 0.005, 0.05, 0.2, 0.5 };

	for (unsigned row = 0; row < 4; row++)
	{
		for (unsigned bit = 0; bit < 16; bit++)
		{
			double const on = m_cycles ? (double(m_lit[row][bit]) / m_cycles) : double(BIT(m_rows[row], bit));

			// average with the last frame, so a changing pattern does not flicker
			double &brightness = m_brightness[row][bit];
			brightness = (brightness + on) / 2.0;

			u8 level = 0;
			while ((level < std::size(LEVELS)) && (brightness > LEVELS[level]))
				level++;
			m_leds[row][bit] = level;

			m_lit[row][bit] = 0;
		}
	}
	m_cycles = 0;
}


INPUT_CHANGED_MEMBER(al8800b_state::control_changed)
{
	show_lever(param, newval);

	switch (param)
	{
	case CTRL_STOP:
		m_stop_held = bool(newval);
		break;

	case CTRL_RUN:
		if (newval && stopped())
		{
			m_run = true;
			show_other();
			m_maincpu->resume(SUSPEND_REASON_HALT);
		}
		break;

	case CTRL_SINGLE_STEP:
		if (newval)
			single_step();
		break;

	// Only the 8800b has SLOW: while it is held, the panel single steps on
	// each rising edge of one output of a 24-bit counter clocked by phi2.
	// The SLOW speed jumper picks stage 18, 20 or 22 (JA, JB or JC to JD).
	// The counter is never reset, so the steps fall on a fixed grid of
	// machine time rather than a fixed delay after the switch goes down.
	case CTRL_SLOW:
		if (newval)
		{
			unsigned const stage = 18 + 2 * m_jumpers.read_safe(0);
			u32 const clock = m_maincpu->clock();
			u64 const next = ((machine().time().as_ticks(clock) >> stage) + 1) << stage;
			m_slow_timer->adjust(attotime::from_ticks(next, clock) - machine().time(), 0, attotime::from_ticks(u64(1) << stage, clock));
		}
		else
			m_slow_timer->adjust(attotime::never);
		break;

	case CTRL_EXAMINE:
		if (newval && stopped())
		{
			m_halted = false;
			m_maincpu->set_pc(m_switches->read());
			show_stopped();
		}
		break;

	case CTRL_EXAMINE_NEXT:
		if (newval && stopped())
		{
			m_halted = false;
			m_maincpu->set_pc((m_maincpu->pc() + 1) & 0xffff);
			show_stopped();
		}
		break;

	case CTRL_DEPOSIT:
		if (newval && stopped())
		{
			m_bus->mwrt_w(m_maincpu->pc(), m_switches->read() & 0xff);
			show_stopped();
		}
		break;

	case CTRL_DEPOSIT_NEXT:
		if (newval && stopped())
		{
			m_maincpu->set_pc((m_maincpu->pc() + 1) & 0xffff);
			m_bus->mwrt_w(m_maincpu->pc(), m_switches->read() & 0xff);
			show_stopped();
		}
		break;

	// RESET holds the CPU reset directly rather than through INPUT_LINE_RESET.
	// Releasing that input line resets the device, which also lifts the
	// suspend that keeps a stopped CPU from running.
	case CTRL_RESET:
		if (newval)
		{
			m_halted = false;
			m_maincpu->reset();
			m_maincpu->suspend(SUSPEND_REASON_RESET, true);
		}
		else
		{
			// The CPU restarts at 000000 and runs unless the panel holds it:
			// stopped already, or STOP held so the first fetch sets the flip-flop.
			if (m_stop_held || m_stepping)
			{
				m_run = false;
				m_stepping = false;
			}
			if (!m_run)
			{
				m_maincpu->suspend(SUSPEND_REASON_HALT, true);
				show_stopped();
			}
			m_maincpu->resume(SUSPEND_REASON_RESET);
		}
		break;

	case CTRL_CLR:
		m_bus->slave_clr_w(newval ? ASSERT_LINE : CLEAR_LINE);
		break;

	// PROT and UNPROT go to the memory board at the address being examined
	case CTRL_PROTECT:
		if (newval && stopped())
		{
			m_bus->prot_w(m_maincpu->pc());
			show_stopped();
		}
		break;

	case CTRL_UNPROTECT:
		if (newval && stopped())
		{
			m_bus->unprot_w(m_maincpu->pc());
			show_stopped();
		}
		break;
	}
}

// The 8800b's accumulator switches, which only work while it is stopped. For
// each one the panel PROM feeds the CPU an IN or an OUT and then a jump back
// to where it stopped, so the bus sees a real I/O cycle and the program
// counter ends up where it was. INPUT and OUTPUT take their I/O channel from
// switches A15-A8.
INPUT_CHANGED_MEMBER(al8800b_state::accumulator_changed)
{
	show_lever(param, newval);
	if (!newval || !stopped())
		return;

	u8 const a = m_maincpu->state_int(i8080_cpu_device::I8085_A);
	u8 const channel = m_switches->read() >> 8;
	m_halted = false;

	switch (param)
	{
	// OUT 377, which the 8800b's interface card latches onto the data LEDs
	case CTRL_ACC_DISPLAY:
		m_bus->sout_w(0xff, a);
		show_stopped();
		m_rows[ROW_DATA] = a;
		break;

	// IN 376, which the panel answers with switches A7-A0 in place of the bus
	case CTRL_ACC_LOAD:
		m_bus->sinp_r(0xfe);
		m_maincpu->set_state_int(i8080_cpu_device::I8085_A, m_switches->read() & 0xff);
		show_stopped();
		break;

	// channel 377 reads the sense switches, as it does for a program
	case CTRL_ACC_INPUT:
		m_maincpu->set_state_int(i8080_cpu_device::I8085_A, (channel == 0xff) ? u8(m_switches->read() >> 8) : m_bus->sinp_r(channel));
		show_stopped();
		break;

	case CTRL_ACC_OUTPUT:
		m_bus->sout_w(channel, a);
		show_stopped();
		if (channel == 0xff)
			m_rows[ROW_DATA] = a;
		break;
	}
}


QUICKLOAD_LOAD_MEMBER(al8800b_state::quickload_cb)
{
	u64 const length = image.length();
	if (length == 0 || length > 0x10000)
		return std::make_pair(image_error::INVALIDLENGTH, std::string());

	std::vector<u8> data(length);
	if (image.fread(&data[0], length) != length)
		return std::make_pair(image_error::UNSPECIFIED, std::string());

	// write through the bus, so the program lands in whatever memory cards are fitted
	for (offs_t i = 0; i < length; i++)
		m_bus->mwrt_w(i, data[i]);

	if (stopped())
	{
		m_halted = false;
		m_maincpu->set_pc(0);
		show_stopped();
	}

	return std::make_pair(std::error_condition(), std::string());
}


#define CONTROL_SWITCH(bit, name) \
	PORT_BIT(1U << al8800b_state::bit, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME(name) \
	PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(al8800b_state::control_changed), al8800b_state::bit)

static INPUT_PORTS_START( al8800 )
	PORT_START("SWITCHES")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A0 / D0")
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A1 / D1")
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A2 / D2")
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A3 / D3")
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A4 / D4")
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A5 / D5")
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A6 / D6")
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A7 / D7")
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A8 / Sense 0")
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A9 / Sense 1")
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A10 / Sense 2")
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A11 / Sense 3")
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A12 / Sense 4")
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A13 / Sense 5")
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A14 / Sense 6")
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_OTHER) PORT_TOGGLE PORT_NAME("A15 / Sense 7")

	PORT_START("CONTROLS")
	CONTROL_SWITCH(CTRL_STOP,         "STOP")
	CONTROL_SWITCH(CTRL_RUN,          "RUN")
	CONTROL_SWITCH(CTRL_SINGLE_STEP,  "SINGLE STEP")
	CONTROL_SWITCH(CTRL_EXAMINE,      "EXAMINE")
	CONTROL_SWITCH(CTRL_EXAMINE_NEXT, "EXAMINE NEXT")
	CONTROL_SWITCH(CTRL_DEPOSIT,      "DEPOSIT")
	CONTROL_SWITCH(CTRL_DEPOSIT_NEXT, "DEPOSIT NEXT")
	CONTROL_SWITCH(CTRL_RESET,        "RESET")
	CONTROL_SWITCH(CTRL_CLR,          "CLR")
	CONTROL_SWITCH(CTRL_PROTECT,      "PROTECT")
	CONTROL_SWITCH(CTRL_UNPROTECT,    "UNPROTECT")
	CONTROL_SWITCH(CTRL_AUX1_UP,      "AUX 1 up")
	CONTROL_SWITCH(CTRL_AUX1_DOWN,    "AUX 1 down")
	CONTROL_SWITCH(CTRL_AUX2_UP,      "AUX 2 up")
	CONTROL_SWITCH(CTRL_AUX2_DOWN,    "AUX 2 down")
INPUT_PORTS_END

#define ACCUMULATOR_SWITCH(bit, name) \
	PORT_BIT(1U << al8800b_state::bit, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME(name) \
	PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(al8800b_state::accumulator_changed), al8800b_state::bit)

static INPUT_PORTS_START( al8800b )
	PORT_INCLUDE( al8800 )

	PORT_MODIFY("CONTROLS")
	CONTROL_SWITCH(CTRL_SLOW,         "SLOW")
	CONTROL_SWITCH(CTRL_CLR,          "EXT CLR")
	ACCUMULATOR_SWITCH(CTRL_ACC_DISPLAY, "ACCUMULATOR DISPLAY")
	ACCUMULATOR_SWITCH(CTRL_ACC_LOAD,    "ACCUMULATOR LOAD")
	ACCUMULATOR_SWITCH(CTRL_ACC_INPUT,   "ACCUMULATOR INPUT")
	ACCUMULATOR_SWITCH(CTRL_ACC_OUTPUT,  "ACCUMULATOR OUTPUT")

	PORT_START("JUMPERS")
	PORT_CONFNAME(0x03, 0x00, "SLOW speed")
	PORT_CONFSETTING(0x00, "JA-JD (standard)")
	PORT_CONFSETTING(0x01, "JB-JD (slower)")
	PORT_CONFSETTING(0x02, "JC-JD (slowest)")
INPUT_PORTS_END


void al8800b_state::machine_start()
{
	m_show_stopped_timer = timer_alloc(FUNC(al8800b_state::show_stopped_cb), this);
	m_led_timer = timer_alloc(FUNC(al8800b_state::update_leds), this);
	m_slow_timer = timer_alloc(FUNC(al8800b_state::slow_cb), this);
	m_led_timer->adjust(attotime::from_hz(60), 0, attotime::from_hz(60));

	save_item(NAME(m_status));
	save_item(NAME(m_inte));
	save_item(NAME(m_prot));
	save_item(NAME(m_halted));
	save_item(NAME(m_run));
	save_item(NAME(m_stepping));
	save_item(NAME(m_stop_held));
	save_item(NAME(m_rows));
}

void al8800b_state::machine_reset()
{
	m_halted = false;
	m_run = false;
	m_stepping = false;
	m_maincpu->suspend(SUSPEND_REASON_HALT, true);
	m_show_stopped_timer->adjust(attotime::zero);
}


static void al8800_s100_cards(device_slot_interface &device)
{
	device.option_add("1mcs", S100_MITS_1MCS);
	device.option_add("4mcs", S100_MITS_4MCS);
	device.option_add("s4k", S100_MITS_S4K);
	device.option_add("16mcs", S100_MITS_16MCS);
	device.option_add("16mcd", S100_MITS_16MCD);
	device.option_add("pmc", S100_MITS_PMC);
	device.option_add("sio", S100_MITS_SIO);
	device.option_add("acr", S100_MITS_ACR);
	device.option_add("2sio", S100_MITS_2SIO);
	device.option_add("4pio", S100_MITS_4PIO);
	device.option_add("hdsk", S100_MITS_HDSK);
	device.option_add("pio", S100_MITS_PIO);
	device.option_add("dcdd", S100_MITS_DCDD);
	device.option_add("vi", S100_MITS_VI);
	device.option_add("lpc", S100_MITS_LPC);
	device.option_add("c700", S100_MITS_C700);
}

void al8800b_state::al8800(machine_config &config)
{
	// The CPU board clock is a 2.000 MHz crystal oscillator with one-shots
	// shaping the two phases; there is no 8224.
	I8080(config, m_maincpu, 2_MHz_XTAL);
	common(config);
}

void al8800b_state::al8800b(machine_config &config)
{
	// the 8800b CPU board has an 8224 clock generator
	I8080A(config, m_maincpu, 18_MHz_XTAL / 9);
	common(config);

	config.set_default_layout(layout_al8800b);
}

void al8800b_state::common(machine_config &config)
{
	m_maincpu->set_addrmap(AS_PROGRAM, &al8800b_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &al8800b_state::io_map);
	m_maincpu->out_status_func().set(FUNC(al8800b_state::status_w));
	m_maincpu->out_inte_func().set(FUNC(al8800b_state::inte_w));

	// an interrupt acknowledge reads the Data In bus, where the pull-ups give
	// RST 7 unless a card such as the 88-VI answers
	S100_BUS(config, m_bus, 2_MHz_XTAL);
	m_bus->irq().set_inputline(m_maincpu, I8085_INTR_LINE);
	m_maincpu->in_inta_func().set(m_bus, FUNC(s100_bus_device::sinta_r));
	S100_SLOT(config, "s100:1", al8800_s100_cards, "2sio");
	S100_SLOT(config, "s100:2", al8800_s100_cards, "16mcs");
	for (unsigned i = 3; i <= 16; i++)
		S100_SLOT(config, util::string_format("s100:%u", i).c_str(), al8800_s100_cards, nullptr);

	config.set_default_layout(layout_al8800);

	QUICKLOAD(config, "quickload", "bin").set_load_callback(FUNC(al8800b_state::quickload_cb));
}


ROM_START( al8800 )
ROM_END

ROM_START( al8800b )
	ROM_REGION( 0x100, "panel", 0 )
	// This 1702A EPROM is not mapped into the 8080 memory space. It contains custom microcode implementing front panel functions.
	ROM_LOAD( "8800b front panel.bin", 0x000, 0x100, CRC(8b462c1b) SHA1(3c13b1cc941225b16580655c15a61b8e8418b052) )
ROM_END

} // anonymous namespace


//    YEAR  NAME     PARENT   COMPAT  MACHINE  INPUT    CLASS          INIT        COMPANY  FULLNAME        FLAGS
COMP( 1976, al8800b, 0,       0,      al8800b, al8800b, al8800b_state, empty_init, "MITS",  "Altair 8800b", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW | MACHINE_SUPPORTS_SAVE )
COMP( 1975, al8800,  al8800b, 0,      al8800,  al8800,  al8800b_state, empty_init, "MITS",  "Altair 8800",  MACHINE_NO_SOUND_HW | MACHINE_SUPPORTS_SAVE )
