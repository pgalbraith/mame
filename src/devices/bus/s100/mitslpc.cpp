// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-LPC line printer controller, with its 88-LP printer

    The controller for MITS's 88-LP, an Okidata line printer with an 80
    character buffer that prints 6-bit ASCII. It takes two ports, control
    at an even address and data at the next one up, set by jumpers on
    A7-A1; MITS software expects 002 octal.

    OUT to the control port, each bit active high: D0 prints the buffer,
    D1 feeds a line, D2 clears the buffer, and D3 enables the interrupt,
    which a 0 there disables. IN from the control port has D0 high while
    the buffer holds from one to 79 characters, D1 high when the print head
    is not moving, D2 high while paper feeds normally, and D3 high when a
    line feed will be accepted. OUT to the data port puts the low six bits
    of a character in the buffer, and the 80th starts a print by itself.

    With its interrupt enabled the controller interrupts once a line has
    printed, until the next print command. A jumper from pad E8 takes the
    interrupt to PINT or to one of the 88-VI's lines.

    The printout goes to a file in ASCII, a line of text for each print and
    an empty line for each line feed. The manual never says that the printer
    moves the paper on after printing a line, but MITS software expects it:
    the manual's test program prints its 64 lines without a single line feed
    command, and Disk BASIC 4.1 ends each line with a print command alone.

    Not emulated
    - the printer's speed, which the manual does not give; a line takes half
      a second to print, and a line feed 100 ms, the rate the FEED switch
      repeats at
    - the printer's LINE and FEED switches; it is always on line
    - paper jams
    - status bits D4-D7, which read high

    References
    - MITS 88-LPC Line Printer Controller documentation, 1975
      [https://deramp.com/downloads/altair/hardware/88-LPC%20Printer%20Interface.pdf]

**********************************************************************/

#include "emu.h"
#include "mitslpc.h"

#include "imagedev/printer.h"


namespace {

class s100_mits_lpc_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_lpc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_sinp_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;

private:
	static constexpr unsigned COLUMNS = 80;

	static attotime print_time() { return attotime::from_msec(500); }
	static attotime feed_time() { return attotime::from_msec(100); }

	// A7-A1 select the board, A0 the control or data port
	bool selected(offs_t offset) { return (offset & 0xfe) == m_address->read(); }

	void control_w(u8 data);
	void data_w(u8 data);
	void start_print();
	void set_irq(bool state);

	TIMER_CALLBACK_MEMBER(print_done);
	TIMER_CALLBACK_MEMBER(feed_done);

	required_device<printer_image_device> m_printer;
	required_ioport m_address;
	required_ioport m_irq_jumper;

	emu_timer *m_print_timer;
	emu_timer *m_feed_timer;

	u8 m_buffer[COLUMNS];
	u8 m_count;             // characters in the buffer
	bool m_printing;        // print head in motion
	bool m_feeding;         // line feed taking place
	bool m_int_enable;
	bool m_irq_state;
	u8 m_irq_route;         // 0 not connected, 1 PINT, 2-9 VI0-VI7
};


s100_mits_lpc_device::s100_mits_lpc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_LPC, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_printer(*this, "printer")
	, m_address(*this, "ADDRESS")
	, m_irq_jumper(*this, "IRQ")
	, m_print_timer(nullptr)
	, m_feed_timer(nullptr)
	, m_buffer{ }
	, m_count(0)
	, m_printing(false)
	, m_feeding(false)
	, m_int_enable(false)
	, m_irq_state(false)
	, m_irq_route(0)
{
}


void s100_mits_lpc_device::device_start()
{
	m_print_timer = timer_alloc(FUNC(s100_mits_lpc_device::print_done), this);
	m_feed_timer = timer_alloc(FUNC(s100_mits_lpc_device::feed_done), this);

	save_item(NAME(m_buffer));
	save_item(NAME(m_count));
	save_item(NAME(m_printing));
	save_item(NAME(m_feeding));
	save_item(NAME(m_int_enable));
	save_item(NAME(m_irq_state));
	save_item(NAME(m_irq_route));
}

void s100_mits_lpc_device::device_reset()
{
	// drop the interrupt line of the old jumper before taking the new one
	if (m_irq_state)
		set_irq(false);
	m_irq_route = m_irq_jumper->read();

	m_count = 0;
	m_printing = false;
	m_feeding = false;
	m_int_enable = false;
	m_print_timer->adjust(attotime::never);
	m_feed_timer->adjust(attotime::never);
}


u8 s100_mits_lpc_device::s100_sinp_r(offs_t offset)
{
	// the data port is output only
	if (!selected(offset) || BIT(offset, 0))
		return 0xff;

	u8 data = 0xf4;     // paper feeding normally; D4-D7 not driven
	if (m_count && (m_count < COLUMNS))
		data |= 0x01;
	if (!m_printing)
		data |= 0x02;
	if (!m_feeding)
		data |= 0x08;
	return data;
}

void s100_mits_lpc_device::s100_sout_w(offs_t offset, u8 data)
{
	if (selected(offset))
	{
		if (BIT(offset, 0))
			data_w(data);
		else
			control_w(data);
	}
}


void s100_mits_lpc_device::control_w(u8 data)
{
	// a print command, or disabling the interrupt, takes back one already raised
	m_int_enable = BIT(data, 3);
	if (m_irq_state && (!m_int_enable || BIT(data, 0)))
		set_irq(false);

	if (BIT(data, 2))
		m_count = 0;

	if (BIT(data, 1) && !m_printing && !m_feeding)
	{
		m_feeding = true;
		m_feed_timer->adjust(feed_time());
	}

	if (BIT(data, 0))
		start_print();
}

void s100_mits_lpc_device::data_w(u8 data)
{
	// the buffer takes nothing while its line is printing
	if (m_printing || (m_count >= COLUMNS))
		return;

	m_buffer[m_count++] = data & 0x3f;
	if (m_count == COLUMNS)
		start_print();
}

void s100_mits_lpc_device::start_print()
{
	if (!m_printing)
	{
		m_printing = true;
		m_print_timer->adjust(print_time());
	}
}

void s100_mits_lpc_device::set_irq(bool state)
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


TIMER_CALLBACK_MEMBER(s100_mits_lpc_device::print_done)
{
	// the 6-bit codes are the low six bits of ASCII 040-137, and the paper
	// moves on a line once the line has printed
	unsigned length = m_count;
	while (length && (m_buffer[length - 1] == 0x20))
		length--;

	for (unsigned i = 0; i < length; i++)
		m_printer->output((m_buffer[i] < 0x20) ? (m_buffer[i] + 0x40) : m_buffer[i]);
	m_printer->output('\n');

	m_count = 0;
	m_printing = false;

	if (m_int_enable)
		set_irq(true);
}

TIMER_CALLBACK_MEMBER(s100_mits_lpc_device::feed_done)
{
	m_printer->output('\n');
	m_feeding = false;
}


void s100_mits_lpc_device::device_add_mconfig(machine_config &config)
{
	PRINTER(config, m_printer);
}


#define ADDRESS_JUMPER(bit, dflt) \
	PORT_CONFNAME(1 << bit, (dflt) << bit, "Address jumper A" #bit) \
	PORT_CONFSETTING(0, "0") \
	PORT_CONFSETTING(1 << bit, "1")

static INPUT_PORTS_START( mits_lpc )
	// A7-A1 are compared with the jumpers; the default is 002 octal
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(7, 0)
	ADDRESS_JUMPER(6, 0)
	ADDRESS_JUMPER(5, 0)
	ADDRESS_JUMPER(4, 0)
	ADDRESS_JUMPER(3, 0)
	ADDRESS_JUMPER(2, 0)
	ADDRESS_JUMPER(1, 1)

	PORT_START("IRQ")
	PORT_CONFNAME(0x0f, 0x00, "Interrupt (pad E8)")
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

ioport_constructor s100_mits_lpc_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_lpc);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_LPC, device_s100_card_interface, s100_mits_lpc_device, "s100_mits_lpc", "MITS 88-LPC Line Printer Controller")
