// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

  Heathkit H-8-5 Serial I/O and Cassette Interface

****************************************************************************/

#include "emu.h"

#include "h_8_5.h"

#include "imagedev/cassette.h"
#include "machine/clock.h"
#include "machine/i8251.h"
#include "machine/timer.h"
#include "bus/rs232/rs232.h"
#include "formats/h8_cas.h"

#include "speaker.h"

#define LOG_LINES (1U << 1)
#define LOG_CASS  (1U << 2)
#define LOG_FUNC  (1U << 3)
#define VERBOSE   (0)

#include "logmacro.h"

#define LOGLINES(...)      LOGMASKED(LOG_LINES, __VA_ARGS__)
#define LOGCASS(...)       LOGMASKED(LOG_CASS, __VA_ARGS__)
#define LOGFUNC(...)       LOGMASKED(LOG_FUNC, __VA_ARGS__)

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

namespace {

class h_8_5_device : public device_t
				   , public device_h8bus_card_interface
{
public:

	h_8_5_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void map_io(address_space_installer & space) override ATTR_COLD;

	void uart_rts(u8 data);
	void uart_tx_empty(u8 data);
	void console_rxrdy_w(int state);
	void console_dtr_w(int state);
	void update_console_int();

	TIMER_DEVICE_CALLBACK_MEMBER(kansas_r);
	TIMER_DEVICE_CALLBACK_MEMBER(kansas_w);

	required_device<i8251_device>          m_uart;
	required_device<i8251_device>          m_console;
	required_device<clock_device>          m_console_clock;
	required_device<rs232_port_device>     m_rs232;
	required_device<cassette_image_device> m_cass_player;
	required_device<cassette_image_device> m_cass_recorder;
	required_ioport                        m_jumpers;

	u8   m_cass_data[4];
	bool m_cassbit;
	bool m_cassold;
	u8   m_console_intr_level;
	bool m_console_rxrdy;
	bool m_console_dtr_n;
};

h_8_5_device::h_8_5_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, H8BUS_H_8_5, tag, owner, 0)
	, device_h8bus_card_interface(mconfig, *this)
	, m_uart(*this, "uart")
	, m_console(*this, "console")
	, m_console_clock(*this, "console_clock")
	, m_rs232(*this, "rs232")
	, m_cass_player(*this, "cassette_player")
	, m_cass_recorder(*this, "cassette_recorder")
	, m_jumpers(*this, "JUMPERS")
{
}

TIMER_DEVICE_CALLBACK_MEMBER(h_8_5_device::kansas_w)
{
	m_cass_data[3]++;

	if (m_cassbit != m_cassold)
	{
		LOGCASS("%s: m_cassbit changed : %d\n", FUNCNAME, m_cassbit);
		m_cass_data[3] = 0;
		m_cassold = m_cassbit;
	}

	LOGCASS("%s: m_cassbit: %d\n", FUNCNAME, m_cassbit);
	// 2400Hz -> 0, 1200Hz -> 1
	const int bit_pos = m_cassbit ? 0 : 1;

	m_cass_recorder->output(BIT(m_cass_data[3], bit_pos) ? -1.0 : +1.0);
}

TIMER_DEVICE_CALLBACK_MEMBER(h_8_5_device::kansas_r)
{
	// cassette - turn 1200/2400Hz to a bit
	m_cass_data[1]++;
	u8 cass_ws = (m_cass_player->input() > +0.03) ? 1 : 0;

	LOGCASS("%s: cass_ws: %d\n", FUNCNAME, cass_ws);

	if (cass_ws != m_cass_data[0])
	{
		LOGCASS("%s: cass_ws has changed value\n", FUNCNAME);
		m_cass_data[0] = cass_ws;
		m_uart->write_rxd((m_cass_data[1] < 12) ? 1 : 0);
		m_cass_data[1] = 0;
	}
}

void h_8_5_device::uart_rts(u8 data)
{
	LOGLINES("%s: data: %d\n", FUNCNAME, data);

	m_cass_player->change_state(bool(data) ? CASSETTE_STOPPED : CASSETTE_PLAY, CASSETTE_MASK_UISTATE);
}

void h_8_5_device::uart_tx_empty(u8 data)
{
	LOGLINES("%s: data: %d\n", FUNCNAME, data);

	m_cass_recorder->change_state(bool(data) ? CASSETTE_STOPPED : CASSETTE_RECORD, CASSETTE_MASK_UISTATE);
}

void h_8_5_device::console_rxrdy_w(int state)
{
	LOGFUNC("%s: state: %d\n", FUNCNAME, state);

	m_console_rxrdy = bool(state);

	update_console_int();
}

void h_8_5_device::console_dtr_w(int state)
{
	LOGLINES("%s: state: %d\n", FUNCNAME, state);

	m_console_dtr_n = bool(state);

	m_rs232->write_dtr(state);

	update_console_int();
}

void h_8_5_device::update_console_int()
{
	// The USART's RxRDY is not what reaches the bus on its own - the gate it
	// passes through is held open by /DTR, so the console interrupt is armed
	// and disarmed by bit 1 of the USART's command register.  HDOS names that
	// bit for what it does here rather than what the datasheet calls it:
	//
	//     UCI.IE  EQU  00000010B  ENABLE INTERRUPTS FLAG
	//
	// and it runs the console both ways.  The boot loader writes
	// UCI.ER+UCI.RE+UCI.TE (025Q) and polls, so a character sitting in the
	// USART raises no interrupt - which is what keeps the boot ROM alive,
	// since until an OS is up the level-3 vector is one of the EI/RET stubs
	// the ROM fills the table with, and a live level there would re-fire out
	// of it forever.  HDOS then writes UCI.RE+UCI.TE+UCI.ER+UCI.IE (027Q)
	// when its interrupt-driven console driver takes over.
	//
	// /DTR is active low, so the gate is open when the handler state is 0.
	int const state = (m_console_rxrdy && !m_console_dtr_n) ? 1 : 0;

	switch (m_console_intr_level)
	{
		case 3: set_slot_int3(state); break;
		case 4: set_slot_int4(state); break;
		case 5: set_slot_int5(state); break;
		case 6: set_slot_int6(state); break;
		case 7: set_slot_int7(state); break;

		// no wire fitted to the interrupt level holes
		default: break;
	}
}

void h_8_5_device::device_start()
{
	save_item(NAME(m_cass_data));
	save_item(NAME(m_cassbit));
	save_item(NAME(m_cassold));
	save_item(NAME(m_console_intr_level));
	save_item(NAME(m_console_rxrdy));
	save_item(NAME(m_console_dtr_n));
}

void h_8_5_device::device_reset()
{
	LOGFUNC("%s\n", FUNCNAME);

	// cassette
	m_cassbit      = 1;
	m_cassold      = 0;
	m_cass_data[0] = 0;
	m_cass_data[1] = 0;
	m_cass_data[2] = 0;
	m_cass_data[3] = 0;

	m_uart->write_cts(0);
	m_uart->write_dsr(0);
	m_uart->write_rxd(0);

	// console interrupt gate closed until the USART is programmed
	m_console_rxrdy  = false;
	m_console_dtr_n  = true;

	// The serial I/O speed is wired, not programmed: a 4 MHz crystal is
	// divided down (IC113 by 13, IC114 by 16, IC115 by 11, IC116 by 2 and 8)
	// and a jumper carries one of the taps to the console USART, which runs at
	// 16X the baud rate. The board brings all eight out to lettered holes, and
	// has separate ones for "SERIAL I/O Rx SPEED" and "SERIAL I/O Tx SPEED";
	// only the pair set the same is useful, so one selection drives both here.
	static constexpr u32 BAUD_RATES[] = { 110, 150, 300, 600, 1200, 2400, 4800, 9600 };

	ioport_value const jumpers(m_jumpers->read());

	m_console_clock->set_unscaled_clock(BAUD_RATES[jumpers & 0x07] * 16);

	// Which condition raises an interrupt, and on which bus level, are wires
	// too: an INT ON/INT OFF pair gates IC128 and IC129, whose four inputs
	// apiece are their USART's TxE, SYN, TxRDY and RxRDY, and the outputs go
	// through the source pads to lettered holes for INT3 through INT7. Only
	// RxRDY is modelled as a source here, gated as update_console_int
	// describes.
	m_console_intr_level = (jumpers >> 3) & 0x07;
}

static INPUT_PORTS_START( h_8_5_jumpers )

	PORT_START("JUMPERS")
	// 9600 to match what a terminal on this port comes up at - the H-19's own
	// SW401, and the generic rs232 options alike. The 600 baud that used to be
	// fixed here looks to have suited the generic "terminal" this port once
	// defaulted to, which does take RS232_ input defaults; the H-19 option
	// keeps its rate on SW401 instead, which those defaults do not reach.
	// Which tap Heath had you wire is not recorded here; the assembly manual
	// would settle it.
	PORT_CONFNAME(0x07, 0x07, "Serial I/O speed")
	PORT_CONFSETTING(   0x00, "110")
	PORT_CONFSETTING(   0x01, "150")
	PORT_CONFSETTING(   0x02, "300")
	PORT_CONFSETTING(   0x03, "600")
	PORT_CONFSETTING(   0x04, "1200")
	PORT_CONFSETTING(   0x05, "2400")
	PORT_CONFSETTING(   0x06, "4800")
	PORT_CONFSETTING(   0x07, "9600")

	// Level 3, which is how the board was built rather than a preference: the
	// assembly manual's "Interrupt Select" step has you fit three jumpers -
	// the indicated RXR holes, hole S to hole /I3, and the INT ON holes - and
	// notes that "the remaining holes are not used with the Heath H8
	// Computer" (595-2032-03).  HDOS agrees: its console input is interrupt
	// driven, the resident ISR sitting behind the level-3 RAM vector at 2025
	// and reading the console USART's data port, so without this wire HDOS
	// stops accepting keys the moment its driver takes over from the boot
	// ROM's polling.  Leaving it wired costs the polled stages nothing,
	// because the gate is closed until the software opens it; see
	// update_console_int.  The other settings are here for experiment only.
	PORT_CONFNAME(0x38, 0x18, "Console interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x18, "Level 3")
	PORT_CONFSETTING(   0x20, "Level 4")
	PORT_CONFSETTING(   0x28, "Level 5")
	PORT_CONFSETTING(   0x30, "Level 6")
	PORT_CONFSETTING(   0x38, "Level 7")

INPUT_PORTS_END

ioport_constructor h_8_5_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(h_8_5_jumpers);
}

void h_8_5_device::map_io(address_space_installer & space)
{
	space.install_readwrite_handler(0xf8, 0xf9,
		read8sm_delegate(m_uart, FUNC(i8251_device::read)),
		write8sm_delegate(m_uart, FUNC(i8251_device::write))
	);

	space.install_readwrite_handler(0xfa, 0xfb,
		read8sm_delegate(m_console, FUNC(i8251_device::read)),
		write8sm_delegate(m_console, FUNC(i8251_device::write))
	);
}


void h_8_5_device::device_add_mconfig(machine_config &config)
{
	I8251(config, m_uart);
	m_uart->txd_handler().set([this] (bool state) { m_cassbit = state; });
	m_uart->rts_handler().set(FUNC(h_8_5_device::uart_rts));
	m_uart->txempty_handler().set(FUNC(h_8_5_device::uart_tx_empty));

	clock_device &cassette_clock(CLOCK(config, "cassette_clock", 4800));
	cassette_clock.signal_handler().set(m_uart, FUNC(i8251_device::write_txc));
	cassette_clock.signal_handler().append(m_uart, FUNC(i8251_device::write_rxc));

	I8251(config, m_console);

	m_console->txd_handler().set("rs232", FUNC(rs232_port_device::write_txd));
	m_console->rts_handler().set("rs232", FUNC(rs232_port_device::write_rts));
	// /DTR both leaves the board and holds the interrupt gate open, so it has
	// to be taken here rather than wired straight to the port.
	m_console->dtr_handler().set(FUNC(h_8_5_device::console_dtr_w));
	// RxRdy reaches the bus only through that gate, and only if a level is
	// jumpered; see update_console_int and device_reset.
	m_console->rxrdy_handler().set(FUNC(h_8_5_device::console_rxrdy_w));

	// Rate comes from the JUMPERS port in device_reset; see the note there.
	CLOCK(config, m_console_clock, 0);
	m_console_clock->signal_handler().set(m_console, FUNC(i8251_device::write_txc));
	m_console_clock->signal_handler().append(m_console, FUNC(i8251_device::write_rxc));

	RS232_PORT(config, m_rs232, default_rs232_devices, "h19");
	m_rs232->rxd_handler().set(m_console, FUNC(i8251_device::write_rxd));
	m_rs232->cts_handler().set(m_console, FUNC(i8251_device::write_cts));
	m_rs232->dsr_handler().set(m_console, FUNC(i8251_device::write_dsr));

	SPEAKER(config, "mono").front_center();

	CASSETTE(config, m_cass_player);
	m_cass_player->set_formats(h8_cassette_formats);
	m_cass_player->set_default_state(CASSETTE_STOPPED | CASSETTE_MOTOR_ENABLED | CASSETTE_SPEAKER_ENABLED);
	m_cass_player->add_route(ALL_OUTPUTS, "mono", 0.15);
	m_cass_player->set_interface("h8_cass_player");

	CASSETTE(config, m_cass_recorder);
	m_cass_recorder->set_formats(h8_cassette_formats);
	m_cass_recorder->set_default_state(CASSETTE_STOPPED | CASSETTE_MOTOR_ENABLED | CASSETTE_SPEAKER_ENABLED);
	m_cass_recorder->add_route(ALL_OUTPUTS, "mono", 0.15);
	m_cass_recorder->set_interface("h8_cass_recorder");

	TIMER(config, "kansas_w").configure_periodic(FUNC(h_8_5_device::kansas_w), attotime::from_hz(4800));
	TIMER(config, "kansas_r").configure_periodic(FUNC(h_8_5_device::kansas_r), attotime::from_hz(40000));
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H8BUS_H_8_5, device_h8bus_card_interface, h_8_5_device, "h8_h_8_5", "Heath H-8-5 Serial/Casette Card");
