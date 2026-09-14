// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-2SIO two port serial interface

    Two MC6850 ACIAs. Word length, parity and stop bits are all set by
    software through the 6850 control register; only the board's I/O
    address and each port's baud rate are jumpers.

    The board takes four ports: base+0 is port 0 control/status, base+1
    port 0 data, and base+2 and base+3 the same for port 1. MITS software
    expects the board at 020 octal.

    MITS says that if CTS and DCD are not wired to anything they must be
    jumpered to ground, which is the default here. With them left to the
    serial port instead, nothing connected means the 6850 never reports
    its transmitter empty, and software waiting to send hangs.

    Not emulated
    - the wait state the board inserts on every IN, about 500 ns
    - the TTL and 20 mA current loop interface options; both ports are RS-232

    References
    - Altair 88-2SIO documentation, March 1977
      [http://www.bitsavers.org/pdf/mits/8800/Altair_88-2-SIO_Documentation_197703.pdf]
    - schematic 8800-140 [https://deramp.com/downloads/altair/hardware/2sio_serial_interface/Altair%202SIO%20Schematic.pdf]

**********************************************************************/

#include "emu.h"
#include "mits2sio.h"

#include "bus/rs232/rs232.h"
#include "machine/6850acia.h"
#include "machine/clock.h"


namespace {

class s100_mits_2sio_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_2sio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

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
	bool selected(offs_t offset) { return (offset & 0xfc) == (m_address->read() & 0xfc); }

	void drive_irq(unsigned route, int state);
	void irq_w(unsigned port, int state);
	void cts_w(unsigned port, int state);
	void dcd_w(unsigned port, int state);

	required_device_array<acia6850_device, 2> m_acia;
	required_device_array<clock_device, 2> m_clock;
	required_device_array<rs232_port_device, 2> m_rs232;
	required_ioport m_address;
	required_ioport_array<2> m_baud;
	required_ioport_array<2> m_irq_jumper;
	required_ioport_array<2> m_handshake;

	u8 m_irq_route[2];      // 0 not connected, 1 PINT, 2-9 VI0-VI7
	bool m_irq_state[2];
	bool m_grounded[2];     // CTS and DCD jumpered to ground
	bool m_cts[2];
	bool m_dcd[2];
};


s100_mits_2sio_device::s100_mits_2sio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_2SIO, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_acia(*this, "acia%u", 0U)
	, m_clock(*this, "clock%u", 0U)
	, m_rs232(*this, "rs232%u", 0U)
	, m_address(*this, "ADDRESS")
	, m_baud(*this, "BAUD%u", 0U)
	, m_irq_jumper(*this, "IRQ%u", 0U)
	, m_handshake(*this, "HANDSHAKE%u", 0U)
	, m_irq_route{ 0, 0 }
	, m_irq_state{ false, false }
	, m_grounded{ true, true }
	, m_cts{ true, true }
	, m_dcd{ true, true }
{
}


void s100_mits_2sio_device::device_start()
{
	save_item(NAME(m_irq_route));
	save_item(NAME(m_irq_state));
	save_item(NAME(m_grounded));
	save_item(NAME(m_cts));
	save_item(NAME(m_dcd));
}

void s100_mits_2sio_device::device_reset()
{
	// the baud rate pads, in pad order
	static constexpr u32 RATES[] = { 110, 150, 300, 1200, 1800, 2400, 4800, 9600 };

	for (unsigned port = 0; port < 2; port++)
	{
		// the 6850 counts 16 or 64 clocks per bit, so the pad rate assumes divide by 16
		m_clock[port]->set_unscaled_clock(RATES[m_baud[port]->read() & 7] * 16);

		// drop the interrupt line of the old jumper before taking the new one
		if (m_irq_state[port])
			drive_irq(m_irq_route[port], 0);
		m_irq_route[port] = m_irq_jumper[port]->read();
		if (m_irq_state[port])
			drive_irq(m_irq_route[port], 1);

		m_grounded[port] = !m_handshake[port]->read();
		m_acia[port]->write_cts(m_grounded[port] ? 0 : m_cts[port]);
		m_acia[port]->write_dcd(m_grounded[port] ? 0 : m_dcd[port]);
	}
}


u8 s100_mits_2sio_device::s100_sinp_r(offs_t offset)
{
	if (!selected(offset))
		return 0xff;

	return m_acia[BIT(offset, 1)]->read(BIT(offset, 0));
}

void s100_mits_2sio_device::s100_sout_w(offs_t offset, u8 data)
{
	if (selected(offset))
		m_acia[BIT(offset, 1)]->write(BIT(offset, 0), data);
}


void s100_mits_2sio_device::drive_irq(unsigned route, int state)
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

void s100_mits_2sio_device::irq_w(unsigned port, int state)
{
	m_irq_state[port] = bool(state);
	drive_irq(m_irq_route[port], state);
}

void s100_mits_2sio_device::cts_w(unsigned port, int state)
{
	m_cts[port] = bool(state);
	if (!m_grounded[port])
		m_acia[port]->write_cts(state);
}

void s100_mits_2sio_device::dcd_w(unsigned port, int state)
{
	m_dcd[port] = bool(state);
	if (!m_grounded[port])
		m_acia[port]->write_dcd(state);
}


static DEVICE_INPUT_DEFAULTS_START( terminal )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

void s100_mits_2sio_device::device_add_mconfig(machine_config &config)
{
	for (unsigned port = 0; port < 2; port++)
	{
		ACIA6850(config, m_acia[port]);
		m_acia[port]->txd_handler().set(m_rs232[port], FUNC(rs232_port_device::write_txd));
		m_acia[port]->rts_handler().set(m_rs232[port], FUNC(rs232_port_device::write_rts));
		m_acia[port]->irq_handler().set([this, port] (int state) { irq_w(port, state); });

		CLOCK(config, m_clock[port], 0);
		m_clock[port]->signal_handler().set(m_acia[port], FUNC(acia6850_device::write_txc));
		m_clock[port]->signal_handler().append(m_acia[port], FUNC(acia6850_device::write_rxc));

		RS232_PORT(config, m_rs232[port], default_rs232_devices, port ? nullptr : "terminal");
		m_rs232[port]->rxd_handler().set(m_acia[port], FUNC(acia6850_device::write_rxd));
		m_rs232[port]->cts_handler().set([this, port] (int state) { cts_w(port, state); });
		m_rs232[port]->dcd_handler().set([this, port] (int state) { dcd_w(port, state); });
		m_rs232[port]->set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(terminal));
	}
}


#define ADDRESS_JUMPER(bit, dflt) \
	PORT_CONFNAME(1 << bit, (dflt) << bit, "Address jumper A" #bit) \
	PORT_CONFSETTING(0, "0") \
	PORT_CONFSETTING(1 << bit, "1")

#define PORT_JUMPERS(port) \
	PORT_START("BAUD" #port) \
	PORT_CONFNAME(0x07, 0x07, "Port " #port " baud rate") \
	PORT_CONFSETTING(0x00, "110") \
	PORT_CONFSETTING(0x01, "150") \
	PORT_CONFSETTING(0x02, "300") \
	PORT_CONFSETTING(0x03, "1200") \
	PORT_CONFSETTING(0x04, "1800") \
	PORT_CONFSETTING(0x05, "2400") \
	PORT_CONFSETTING(0x06, "4800") \
	PORT_CONFSETTING(0x07, "9600") \
	PORT_START("IRQ" #port) \
	PORT_CONFNAME(0x0f, 0x00, "Port " #port " interrupt") \
	PORT_CONFSETTING(0x00, "Not connected") \
	PORT_CONFSETTING(0x01, "PINT") \
	PORT_CONFSETTING(0x02, "VI0") \
	PORT_CONFSETTING(0x03, "VI1") \
	PORT_CONFSETTING(0x04, "VI2") \
	PORT_CONFSETTING(0x05, "VI3") \
	PORT_CONFSETTING(0x06, "VI4") \
	PORT_CONFSETTING(0x07, "VI5") \
	PORT_CONFSETTING(0x08, "VI6") \
	PORT_CONFSETTING(0x09, "VI7") \
	PORT_START("HANDSHAKE" #port) \
	PORT_CONFNAME(0x01, 0x00, "Port " #port " CTS and DCD") \
	PORT_CONFSETTING(0x00, "Jumpered to ground") \
	PORT_CONFSETTING(0x01, "From serial port")

static INPUT_PORTS_START( mits_2sio )
	// A2-A7 are compared with the jumpers; the default is 020 octal
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(7, 0)
	ADDRESS_JUMPER(6, 0)
	ADDRESS_JUMPER(5, 0)
	ADDRESS_JUMPER(4, 1)
	ADDRESS_JUMPER(3, 0)
	ADDRESS_JUMPER(2, 0)

	PORT_JUMPERS(0)
	PORT_JUMPERS(1)
INPUT_PORTS_END

ioport_constructor s100_mits_2sio_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_2sio);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_2SIO, device_s100_card_interface, s100_mits_2sio_device, "s100_mits_2sio", "MITS 88-2SIO Serial Interface")
