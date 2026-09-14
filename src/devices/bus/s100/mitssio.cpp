// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-SIO serial interface, revision 1

    The Altair's first serial board: one COM2502 UART (the AY-5-1013
    family), with word length, parity and stop bits set by jumpers rather
    than software. It takes two ports, control/status at an even address
    and data at the next one up. 000 and 001 octal is the usual place.

    Status (IN from the control port) is active low for the two ready bits:
    bit 0 is low when a character has arrived, bit 7 is low when the
    transmitter can take one. Bits 2, 3 and 4 are high for parity error,
    framing error and overrun. An OUT to the control port enables the
    input interrupt with bit 0 and the output interrupt with bit 1.

    Revision 0 boards have a different status layout, taking the ready bits
    from device ready pulses on the connector, and are not emulated. An
    errata modification from MITS converts them to the revision 1 layout.

    Not emulated
    - revision 0 status
    - separate jumpers for the input and output interrupts; both share one
    - the RS-232 and 20 mA current loop options (88-SIOA and 88-SIOC); the
      port here is RS-232 at logic levels
    - the baud rate counter, which is preset by 12 jumpers and clocked from
      the bus; the rates here are exact

    References
    - 88-SIO Rev 0 and 1 documentation [https://deramp.com/downloads/altair/hardware/sio_serial_interface/88-SIO%20Rev%200%20&%201.pdf]
    - 88-SIO Rev 0 documentation with a Rev 1 to 0 schematic
      [https://deramp.com/downloads/altair/hardware/sio_serial_interface/88-SIO%20Rev%200%20Doc,%20Rev%201%20to%200%20Schematic.pdf]

**********************************************************************/

#include "emu.h"
#include "mitssio.h"

#include "bus/rs232/rs232.h"
#include "machine/ay31015.h"
#include "machine/clock.h"


namespace {

class s100_mits_sio_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_sio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

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
	bool selected(offs_t offset) { return (offset & 0xfe) == (m_address->read() & 0xfe); }

	void update_irq();
	void drive_irq(unsigned route, int state);

	required_device<ay51013_device> m_uart;
	required_device<clock_device> m_clock;
	required_device<rs232_port_device> m_rs232;
	required_ioport m_address;
	required_ioport m_format;
	required_ioport m_baud;
	required_ioport m_irq_jumper;

	bool m_input_int_enable;
	bool m_output_int_enable;
	u8 m_irq_route;         // 0 not connected, 1 PINT, 2-9 VI0-VI7
	bool m_irq_state;
};


s100_mits_sio_device::s100_mits_sio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_SIO, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_uart(*this, "uart")
	, m_clock(*this, "clock")
	, m_rs232(*this, "rs232")
	, m_address(*this, "ADDRESS")
	, m_format(*this, "FORMAT")
	, m_baud(*this, "BAUD")
	, m_irq_jumper(*this, "IRQ")
	, m_input_int_enable(false)
	, m_output_int_enable(false)
	, m_irq_route(0)
	, m_irq_state(false)
{
}


void s100_mits_sio_device::device_start()
{
	save_item(NAME(m_input_int_enable));
	save_item(NAME(m_output_int_enable));
	save_item(NAME(m_irq_route));
	save_item(NAME(m_irq_state));
}

void s100_mits_sio_device::device_reset()
{
	static constexpr u32 RATES[] = { 110, 150, 300, 600, 1200, 2400, 4800, 9600, 19200 };

	// the UART counts 16 clocks per bit
	m_clock->set_unscaled_clock(RATES[std::min<unsigned>(m_baud->read(), std::size(RATES) - 1)] * 16);

	// word format jumpers, tied high or low on the UART's control pins
	ioport_value const format = m_format->read();
	m_uart->write_cs(1);
	m_uart->write_nb1(BIT(format, 0));
	m_uart->write_nb2(BIT(format, 1));
	m_uart->write_np(BIT(format, 2));
	m_uart->write_eps(BIT(format, 3));
	m_uart->write_tsb(BIT(format, 4));
	m_uart->write_swe(0);
	m_uart->write_xr(1);
	m_uart->write_xr(0);

	if (m_irq_state)
		drive_irq(m_irq_route, 0);
	m_irq_route = m_irq_jumper->read();
	m_irq_state = false;
	update_irq();
}


u8 s100_mits_sio_device::s100_sinp_r(offs_t offset)
{
	if (!selected(offset))
		return 0xff;

	if (BIT(offset, 0))
	{
		// reading data also clears data available, which may drop the interrupt
		u8 const data = m_uart->receive();
		update_irq();
		return data;
	}

	u8 status = 0;
	if (!m_uart->dav_r())
		status |= 0x01;
	if (m_uart->pe_r())
		status |= 0x04;
	if (m_uart->fe_r())
		status |= 0x08;
	if (m_uart->or_r())
		status |= 0x10;
	if (!m_uart->tbmt_r())
		status |= 0x80;
	return status;
}

void s100_mits_sio_device::s100_sout_w(offs_t offset, u8 data)
{
	if (!selected(offset))
		return;

	if (BIT(offset, 0))
	{
		m_uart->transmit(data);
	}
	else
	{
		m_input_int_enable = BIT(data, 0);
		m_output_int_enable = BIT(data, 1);
	}
	update_irq();
}


void s100_mits_sio_device::update_irq()
{
	bool const state = (m_input_int_enable && m_uart->dav_r()) || (m_output_int_enable && m_uart->tbmt_r());
	if (state != m_irq_state)
	{
		m_irq_state = state;
		drive_irq(m_irq_route, state);
	}
}

void s100_mits_sio_device::drive_irq(unsigned route, int state)
{
	switch (route)
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


static DEVICE_INPUT_DEFAULTS_START( terminal )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

void s100_mits_sio_device::device_add_mconfig(machine_config &config)
{
	AY51013(config, m_uart);
	m_uart->set_auto_rdav(true);
	m_uart->write_so_callback().set(m_rs232, FUNC(rs232_port_device::write_txd));
	m_uart->write_dav_callback().set([this] (int state) { update_irq(); });
	m_uart->write_tbmt_callback().set([this] (int state) { update_irq(); });

	CLOCK(config, m_clock, 0);
	m_clock->signal_handler().set(m_uart, FUNC(ay51013_device::write_tcp));
	m_clock->signal_handler().append(m_uart, FUNC(ay51013_device::write_rcp));

	RS232_PORT(config, m_rs232, default_rs232_devices, "terminal");
	m_rs232->rxd_handler().set(m_uart, FUNC(ay51013_device::write_si));
	m_rs232->set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(terminal));
}


#define ADDRESS_JUMPER(bit) \
	PORT_CONFNAME(1 << bit, 0, "Address jumper A" #bit) \
	PORT_CONFSETTING(0, "0") \
	PORT_CONFSETTING(1 << bit, "1")

static INPUT_PORTS_START( mits_sio )
	// A1-A7 are compared with the jumpers; A0 picks control or data
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(7)
	ADDRESS_JUMPER(6)
	ADDRESS_JUMPER(5)
	ADDRESS_JUMPER(4)
	ADDRESS_JUMPER(3)
	ADDRESS_JUMPER(2)
	ADDRESS_JUMPER(1)

	// bits are the UART pins: NB1, NB2, NP, EPS, TSB
	PORT_START("FORMAT")
	PORT_CONFNAME(0x03, 0x03, "Data bits")
	PORT_CONFSETTING(0x00, "5")
	PORT_CONFSETTING(0x01, "6")
	PORT_CONFSETTING(0x02, "7")
	PORT_CONFSETTING(0x03, "8")
	PORT_CONFNAME(0x0c, 0x04, "Parity")
	PORT_CONFSETTING(0x04, "None")
	PORT_CONFSETTING(0x00, "Odd")
	PORT_CONFSETTING(0x08, "Even")
	PORT_CONFNAME(0x10, 0x00, "Stop bits")
	PORT_CONFSETTING(0x00, "1")
	PORT_CONFSETTING(0x10, "2")

	PORT_START("BAUD")
	PORT_CONFNAME(0x0f, 0x07, "Baud rate")
	PORT_CONFSETTING(0x00, "110")
	PORT_CONFSETTING(0x01, "150")
	PORT_CONFSETTING(0x02, "300")
	PORT_CONFSETTING(0x03, "600")
	PORT_CONFSETTING(0x04, "1200")
	PORT_CONFSETTING(0x05, "2400")
	PORT_CONFSETTING(0x06, "4800")
	PORT_CONFSETTING(0x07, "9600")
	PORT_CONFSETTING(0x08, "19200")

	PORT_START("IRQ")
	PORT_CONFNAME(0x0f, 0x00, "Interrupt")
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

ioport_constructor s100_mits_sio_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_sio);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_SIO, device_s100_card_interface, s100_mits_sio_device, "s100_mits_sio", "MITS 88-SIO Serial Interface")
