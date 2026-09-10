// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heathkit H-8-4 Four Port Serial I/O

    Four INS8250 ACEs on one card, each with its own RS-232 connector, its
    own address selection jumpers and its own interrupt select jumpers. All
    four channels are the same circuit - what a port is "for" is only a
    matter of where its jumpers put it and what the software expects to find
    there.

    The four addresses offered here are the ones Heath software expects, and
    they are the same four an H-89 decodes on its own CPU board (the decoder
    tables in bus/heathzenith/h89/h89bus.cpp list them), so an H8 with this
    card answers where a program written for an H-89 looks:

        320-327 octal   D0-D7   alternate terminal, HDOS AT:
        330-337 octal   D8-DF   third serial port
        340-347 octal   E0-E7   line printer, HDOS LP:
        350-357 octal   E8-EF   console terminal, HDOS TT:

    HDOS from issue 50.03.00 on ships a driver built for this card,
    ATH84.DVD, and its source listing (ATDVD in HOS-1-SL, 595-2466) has

        IF H84IO=0 THEN PORT = 374-5 ELSE PORT = 320-7

    - 374-5 octal being where the same driver's H-8-5 build looks, 320 the
    first port here. So an H8 with this card fitted and left at the defaults
    is what that driver is looking for.

    Port 4 is the exception: it ships switched off, because HDOS moves its
    console onto a card it finds at 350 octal. The note above the jumper
    settings at the bottom of this file has the detail.

    Speed is programmed rather than jumpered: one 1.8432 MHz oscillator
    clocks all four ACEs and the divisor latches do the rest.

    The card takes a 16550 in place of an 8250 on the reproduction boards,
    and the original had a 20 mA current loop option for a Teletype; neither
    is modelled.

    Card description and address list from the SEBHC reproduction's
    documentation [https://sebhc.github.io/sebhc/pcbs/H8-4_doc.pdf] and its
    8250 programming notes
    [https://sebhc.github.io/sebhc/project8080/pgs/8250.html].

****************************************************************************/

#include "emu.h"

#include "h_8_4.h"

#include "bus/rs232/rs232.h"
#include "machine/ins8250.h"

namespace {

class h_8_4_device : public device_t
				   , public device_h8bus_card_interface
{
public:

	h_8_4_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void map_io(address_space_installer &space) override ATTR_COLD;

	void read_jumpers();
	void uart_int_w(int index, int state);
	void update_int(u8 level);

	required_device_array<ins8250_device, 4> m_uart;
	required_ioport_array<4>                 m_jumpers;

	// base address each port answers at, 0 when its address jumpers are out
	u8  m_base[4];

	// bus interrupt level each port is wired to, 0 when no wire is fitted
	u8  m_level[4];

	int m_intr[4];
};

h_8_4_device::h_8_4_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, H8BUS_H_8_4, tag, owner, 0)
	, device_h8bus_card_interface(mconfig, *this)
	, m_uart(*this, "uart%u", 1U)
	, m_jumpers(*this, "JUMPERS%u", 1U)
{
}

void h_8_4_device::read_jumpers()
{
	// Address selection, in the order the settings are listed in
	// h_8_4_jumpers. Index 0 is a port with its address jumpers out, and the
	// three spare slots keep a hand-edited cfg file from running off the end.
	static constexpr u8 BASE[8] = { 0x00, 0xd0, 0xd8, 0xe0, 0xe8, 0x00, 0x00, 0x00 };

	for (int i = 0; i < 4; i++)
	{
		ioport_value const jumpers(m_jumpers[i]->read());

		m_base[i]  = BASE[jumpers & 0x07];
		m_level[i] = (jumpers >> 3) & 0x07;
	}
}

void h_8_4_device::uart_int_w(int index, int state)
{
	m_intr[index] = state;

	update_int(m_level[index]);
}

void h_8_4_device::update_int(u8 level)
{
	// no wire fitted to this port's interrupt holes
	if (level == 0)
	{
		return;
	}

	// Ports jumpered to the same level share one bus line, so all of them
	// have to be looked at to decide whether it is still being held.
	int state = 0;

	for (int i = 0; i < 4; i++)
	{
		if (m_base[i] && (m_level[i] == level))
		{
			state |= m_intr[i];
		}
	}

	switch (level)
	{
		case 3: set_slot_int3(state); break;
		case 4: set_slot_int4(state); break;
		case 5: set_slot_int5(state); break;
		case 6: set_slot_int6(state); break;
		case 7: set_slot_int7(state); break;
	}
}

void h_8_4_device::device_start()
{
	for (int i = 0; i < 4; i++)
	{
		m_base[i]  = 0;
		m_level[i] = 0;
		m_intr[i]  = 0;
	}

	save_item(NAME(m_base));
	save_item(NAME(m_level));
	save_item(NAME(m_intr));
}

void h_8_4_device::device_reset()
{
	// The jumpers are read in map_io, which the CPU card runs from its own
	// reset - not here, because nothing orders the two resets and reading them
	// twice would leave map_io unable to see that an address had moved.
	for (int i = 0; i < 4; i++)
	{
		m_intr[i] = 0;
	}

	for (u8 level = 3; level <= 7; level++)
	{
		update_int(level);
	}
}

void h_8_4_device::map_io(address_space_installer &space)
{
	// The bus maps its cards from the CPU card's reset, so this runs again on
	// every reset. Re-reading the jumpers here is what lets an address moved
	// in the machine configuration menu take effect, and a port that has
	// moved gives up the window it had before - only ever a window this card
	// installed itself.
	u8 const previous[4] = { m_base[0], m_base[1], m_base[2], m_base[3] };

	read_jumpers();

	// Every window that is going comes out before any new one goes in, so that
	// two ports swapping addresses cannot have the second one's unmap take the
	// first one's freshly installed window with it.
	for (int i = 0; i < 4; i++)
	{
		if (previous[i] && (previous[i] != m_base[i]))
		{
			space.unmap_readwrite(previous[i], previous[i] + 7);
		}
	}

	for (int i = 0; i < 4; i++)
	{
		if (m_base[i])
		{
			space.install_readwrite_handler(m_base[i], m_base[i] + 7,
				read8sm_delegate(m_uart[i], FUNC(ins8250_device::ins8250_r)),
				write8sm_delegate(m_uart[i], FUNC(ins8250_device::ins8250_w))
			);
		}
	}
}

void h_8_4_device::device_add_mconfig(machine_config &config)
{
	static char const *const RS232_TAG[4] = { "rs232_1", "rs232_2", "rs232_3", "rs232_4" };

	for (int i = 0; i < 4; i++)
	{
		INS8250(config, m_uart[i], XTAL(1'843'200));

		m_uart[i]->out_tx_callback().set(RS232_TAG[i], FUNC(rs232_port_device::write_txd));
		m_uart[i]->out_dtr_callback().set(RS232_TAG[i], FUNC(rs232_port_device::write_dtr));
		m_uart[i]->out_rts_callback().set(RS232_TAG[i], FUNC(rs232_port_device::write_rts));

		// Whether this reaches the bus, and on which level, is a wire; see
		// update_int and the JUMPERS ports.
		m_uart[i]->out_int_callback().set([this, i] (int state) { uart_int_w(i, state); });

		rs232_port_device &port(RS232_PORT(config, RS232_TAG[i], default_rs232_devices, nullptr));

		port.rxd_handler().set(m_uart[i], FUNC(ins8250_uart_device::rx_w));
		port.dcd_handler().set(m_uart[i], FUNC(ins8250_uart_device::dcd_w));
		port.dsr_handler().set(m_uart[i], FUNC(ins8250_uart_device::dsr_w));
		port.cts_handler().set(m_uart[i], FUNC(ins8250_uart_device::cts_w));
		port.ri_handler().set(m_uart[i], FUNC(ins8250_uart_device::ri_w));
	}
}

// PORT 4 STARTS OUT SWITCHED OFF - fitting this card must not cost the H8 its
// console.  HDOS from 50.03.00 on looks for an 8250 at 350 octal and, if it
// finds one, runs the console there instead of on the H-8-5's USART.  It does
// find this card: with port 4 on, HDOS probes 350 (IER 0, LCR 8N1, MCR loop),
// sets the divisor latch to 48 for 2400 baud, and from that moment says
// nothing on the H-8-5 at all - measured on h8_h17 with HDOS 1.6, which wrote
// 18 characters to the H-8-5's data port with the port off and none with it
// on.  With nothing attached to port 4 the machine simply looks dead.
// Leaving port 4 out keeps the console where an H8's console is.
//
// Attaching a terminal is not quite enough either, and the trap is the one
// SW401 sets on the H-8-5: HDOS programs 350 for 2400 baud while the H-19 and
// the generic terminal both come up at 9600, so the two talk past each other.
// Set the terminal to 2400 and the console does appear - though not until it
// receives a character, so the first thing to send is a SPACE, after which
// ACTION? <BOOT> and the rest of the boot arrive on port 4.
//
// Port 4's interrupt is jumpered to level 3 so that turning the address on and
// attaching a terminal is all it takes.  HDOS's console input is interrupt
// driven and this wire is what carries it: on the same h8_h17 boot, a command
// typed at the date prompt is echoed and answered with the wire fitted, and
// with it out not one character is taken - the console prints and never
// accepts a key.  That is the same wire, and the same failure, as the H-8-5's
// "Console interrupt" jumper.
//
// A period source agrees that this is where an H-8-4 console lives and that it
// runs on interrupts.  REMark issue 22 (1981) prints an INKEY routine "designed
// for the H8-4 Multi-Port Serial Card with the console set to 350 octal" which
// brackets a polled read with OUT 233,0 and OUT 233,1 - 233 decimal is 0351
// octal, the 8250's interrupt enable register - to stop the console interrupt
// firing while it reads the line status at 237 (0355, LSR) and takes the byte
// from 232 (0350, RBR).
//
// The other three interrupt holes start out empty, which is how the AT: and
// LP: ports are normally left - HDOS drives both polled, and its AT: driver
// is documented as not to be used with receiver interrupts.  A modem is the
// case that wants one, and REMark issue 17 (1981), walking a reader through
// connecting to CompuServe, gives both settings for it: "the interrupt on
// each machine must be jumpered to level 5.  The port assignment to the
// channel used on the H8-4 serial I/O board must also be jumpered to 330Q."
// An earlier note here said level 6, from
// [https://sebhc.github.io/sebhc/project8080/pgs/feat.html]; the contemporary
// walkthrough for the software people actually ran is the better source.
// Which lettered holes the board brings out is still a question for the
// assembly manual, which is not to hand; levels 3 to 7 are what the H8 bus
// carries and what the H-8-5 offers.
static INPUT_PORTS_START( h_8_4_jumpers )

	PORT_START("JUMPERS1")
	PORT_CONFNAME(0x07, 0x01, "Port 1 address")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "320-327 octal")
	PORT_CONFSETTING(   0x02, "330-337 octal")
	PORT_CONFSETTING(   0x03, "340-347 octal")
	PORT_CONFSETTING(   0x04, "350-357 octal")
	PORT_CONFNAME(0x38, 0x00, "Port 1 interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x18, "Level 3")
	PORT_CONFSETTING(   0x20, "Level 4")
	PORT_CONFSETTING(   0x28, "Level 5")
	PORT_CONFSETTING(   0x30, "Level 6")
	PORT_CONFSETTING(   0x38, "Level 7")

	PORT_START("JUMPERS2")
	PORT_CONFNAME(0x07, 0x02, "Port 2 address")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "320-327 octal")
	PORT_CONFSETTING(   0x02, "330-337 octal")
	PORT_CONFSETTING(   0x03, "340-347 octal")
	PORT_CONFSETTING(   0x04, "350-357 octal")
	PORT_CONFNAME(0x38, 0x00, "Port 2 interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x18, "Level 3")
	PORT_CONFSETTING(   0x20, "Level 4")
	PORT_CONFSETTING(   0x28, "Level 5")
	PORT_CONFSETTING(   0x30, "Level 6")
	PORT_CONFSETTING(   0x38, "Level 7")

	PORT_START("JUMPERS3")
	PORT_CONFNAME(0x07, 0x03, "Port 3 address")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "320-327 octal")
	PORT_CONFSETTING(   0x02, "330-337 octal")
	PORT_CONFSETTING(   0x03, "340-347 octal")
	PORT_CONFSETTING(   0x04, "350-357 octal")
	PORT_CONFNAME(0x38, 0x00, "Port 3 interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x18, "Level 3")
	PORT_CONFSETTING(   0x20, "Level 4")
	PORT_CONFSETTING(   0x28, "Level 5")
	PORT_CONFSETTING(   0x30, "Level 6")
	PORT_CONFSETTING(   0x38, "Level 7")

	PORT_START("JUMPERS4")
	PORT_CONFNAME(0x07, 0x00, "Port 4 address")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "320-327 octal")
	PORT_CONFSETTING(   0x02, "330-337 octal")
	PORT_CONFSETTING(   0x03, "340-347 octal")
	PORT_CONFSETTING(   0x04, "350-357 octal")
	PORT_CONFNAME(0x38, 0x18, "Port 4 interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x18, "Level 3")
	PORT_CONFSETTING(   0x20, "Level 4")
	PORT_CONFSETTING(   0x28, "Level 5")
	PORT_CONFSETTING(   0x30, "Level 6")
	PORT_CONFSETTING(   0x38, "Level 7")

INPUT_PORTS_END

ioport_constructor h_8_4_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(h_8_4_jumpers);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H8BUS_H_8_4, device_h8bus_card_interface, h_8_4_device, "h8_h_8_4", "Heath H-8-4 Four Port Serial I/O Card");
