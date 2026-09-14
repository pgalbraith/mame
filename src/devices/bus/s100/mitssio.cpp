// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-SIO serial interface, revision 1, and 88-ACR audio cassette
    interface

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

    88-ACR
    An 88-SIO B, the TTL level board, with a modem board mated to it and
    wired as MITS software expects: 006 and 007 octal, 300 baud, eight data
    bits, one stop bit and no parity. The modulator's presettable counter
    divides the 2 MHz bus clock by 104 while the UART sends a 1 and by 135
    while it sends a 0, and a divide by 8 after it makes a square wave of
    2404 Hz or 1852 Hz with no break in phase, so an idle transmitter
    records a steady 2400 Hz tone. The demodulator filters the tape signal
    and feeds a phase locked loop, set halfway between the two tones and
    enabled by a carrier detector, into the UART.

    MITS suggested a relay on the input interrupt flip-flop, which MITS
    software leaves unused, so that control bit D0 starts and stops a
    recorder's motor: OUT 6,1 and OUT 6,0 from BASIC. It is an option here,
    not fitted by default.

    Not emulated on the 88-ACR
    - the filter and phase locked loop: each cycle of the tape signal is
      timed instead and averaged with the one before, and shorter than a
      cycle of 2125 Hz gives a 1, with 20 us of hysteresis
    - the carrier detector; with no signal for 2 ms the UART receives 1s

    References
    - 88-SIO Rev 0 and 1 documentation [https://deramp.com/downloads/altair/hardware/sio_serial_interface/88-SIO%20Rev%200%20&%201.pdf]
    - 88-SIO Rev 0 documentation with a Rev 1 to 0 schematic
      [https://deramp.com/downloads/altair/hardware/sio_serial_interface/88-SIO%20Rev%200%20Doc,%20Rev%201%20to%200%20Schematic.pdf]
    - 88-ACR documentation, second printing, February 1977
      [https://deramp.com/downloads/altair/hardware/cassette_interface/Altair%2088-ACR%20Cassette%20Interface.pdf]

**********************************************************************/

#include "emu.h"
#include "mitssio.h"

#include "bus/rs232/rs232.h"
#include "imagedev/cassette.h"
#include "machine/ay31015.h"
#include "machine/clock.h"

#include "formats/mits_cas.h"

#include "softlist_dev.h"
#include "speaker.h"


namespace {

//**************************************************************************
//  88-SIO
//**************************************************************************

class s100_mits_sio_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_sio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	s100_mits_sio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_sinp_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;

	required_device<ay51013_device> m_uart;
	required_device<clock_device> m_clock;
	optional_device<rs232_port_device> m_rs232;
	required_ioport m_address;
	required_ioport m_format;
	required_ioport m_baud;
	required_ioport m_irq_jumper;

	bool m_input_int_enable;
	bool m_output_int_enable;
	u8 m_irq_route;         // 0 not connected, 1 PINT, 2-9 VI0-VI7
	bool m_irq_state;

private:
	bool selected(offs_t offset) { return (offset & 0xfe) == (m_address->read() & 0xfe); }

	void update_irq();
	void drive_irq(unsigned route, int state);
};


s100_mits_sio_device::s100_mits_sio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
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

s100_mits_sio_device::s100_mits_sio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: s100_mits_sio_device(mconfig, S100_MITS_SIO, tag, owner, clock)
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


#define ADDRESS_JUMPER(bit, dflt) \
	PORT_CONFNAME(1 << bit, (dflt) << bit, "Address jumper A" #bit) \
	PORT_CONFSETTING(0, "0") \
	PORT_CONFSETTING(1 << bit, "1")

#define BAUD_RATE_SETTINGS \
	PORT_CONFSETTING(0x00, "110") \
	PORT_CONFSETTING(0x01, "150") \
	PORT_CONFSETTING(0x02, "300") \
	PORT_CONFSETTING(0x03, "600") \
	PORT_CONFSETTING(0x04, "1200") \
	PORT_CONFSETTING(0x05, "2400") \
	PORT_CONFSETTING(0x06, "4800") \
	PORT_CONFSETTING(0x07, "9600") \
	PORT_CONFSETTING(0x08, "19200")

static INPUT_PORTS_START( mits_sio )
	// A1-A7 are compared with the jumpers; A0 picks control or data
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(7, 0)
	ADDRESS_JUMPER(6, 0)
	ADDRESS_JUMPER(5, 0)
	ADDRESS_JUMPER(4, 0)
	ADDRESS_JUMPER(3, 0)
	ADDRESS_JUMPER(2, 0)
	ADDRESS_JUMPER(1, 0)

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
	BAUD_RATE_SETTINGS

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


//**************************************************************************
//  88-ACR
//**************************************************************************

class s100_mits_acr_device : public s100_mits_sio_device
{
public:
	s100_mits_acr_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual void s100_sout_w(offs_t offset, u8 data) override;

private:
	static constexpr u32 BUS_CLOCK = 2'000'000;

	TIMER_CALLBACK_MEMBER(modulator_count);
	TIMER_CALLBACK_MEMBER(demodulator_sample);
	void receive(int state);
	void update_motor();

	required_device<cassette_image_device> m_cassette;
	required_ioport m_motor_relay;

	emu_timer *m_modulator_timer;
	emu_timer *m_demodulator_timer;

	u8 m_transmit;          // serial data from the UART
	u8 m_divider;           // the divide by 8 after the presettable counter
	u8 m_level;             // the tape signal, 1 above zero
	u8 m_receive;           // demodulated data to the UART
	attotime m_fall;        // when the tape signal last crossed zero each way
	attotime m_rise;
	double m_fall_cycle;    // the last whole cycle timed each way, -1 if none
	double m_rise_cycle;
};


s100_mits_acr_device::s100_mits_acr_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: s100_mits_sio_device(mconfig, S100_MITS_ACR, tag, owner, clock)
	, m_cassette(*this, "cassette")
	, m_motor_relay(*this, "MOTOR")
	, m_modulator_timer(nullptr)
	, m_demodulator_timer(nullptr)
	, m_transmit(1)
	, m_divider(0)
	, m_level(0)
	, m_receive(1)
	, m_fall(attotime::zero)
	, m_rise(attotime::zero)
	, m_fall_cycle(-1.0)
	, m_rise_cycle(-1.0)
{
}


void s100_mits_acr_device::device_start()
{
	s100_mits_sio_device::device_start();

	m_modulator_timer = timer_alloc(FUNC(s100_mits_acr_device::modulator_count), this);
	m_modulator_timer->adjust(attotime::from_ticks(104, BUS_CLOCK));
	m_demodulator_timer = timer_alloc(FUNC(s100_mits_acr_device::demodulator_sample), this);
	m_demodulator_timer->adjust(attotime::from_hz(40'000), 0, attotime::from_hz(40'000));

	save_item(NAME(m_transmit));
	save_item(NAME(m_divider));
	save_item(NAME(m_level));
	save_item(NAME(m_receive));
	save_item(NAME(m_fall));
	save_item(NAME(m_rise));
	save_item(NAME(m_fall_cycle));
	save_item(NAME(m_rise_cycle));
}

void s100_mits_acr_device::device_reset()
{
	s100_mits_sio_device::device_reset();

	m_uart->write_si(m_receive);
	update_motor();
}


void s100_mits_acr_device::s100_sout_w(offs_t offset, u8 data)
{
	s100_mits_sio_device::s100_sout_w(offset, data);
	update_motor();
}


TIMER_CALLBACK_MEMBER(s100_mits_acr_device::modulator_count)
{
	// the tone changes level every fourth time the counter runs out, and the
	// counter takes its next count from the UART's output as it reloads
	m_divider = (m_divider + 1) & 7;
	if (!(m_divider & 3))
		m_cassette->output(BIT(m_divider, 2) ? 1.0 : -1.0);
	m_modulator_timer->adjust(attotime::from_ticks(m_transmit ? 104 : 135, BUS_CLOCK));
}

TIMER_CALLBACK_MEMBER(s100_mits_acr_device::demodulator_sample)
{
	// MITS has the recorder on Play In or Record Out, never both, so nothing
	// comes back while recording
	attotime const now = machine().time();
	double const signal = m_cassette->is_playing() ? m_cassette->input() : 0.0;

	// a zero crossing, with a little hysteresis against noise
	u8 const level = (signal > 0.05) ? 1 : (signal < -0.05) ? 0 : m_level;
	if (level != m_level)
	{
		// time a whole cycle from the last crossing the same way
		attotime &last = level ? m_rise : m_fall;
		double &cycle = level ? m_rise_cycle : m_fall_cycle;
		double const period = (now - last).as_double();
		cycle = (period < 0.002) ? period : -1.0;
		last = now;
		m_level = level;

		// average it with the cycle timed the other way, standing in for the
		// smoothing in the phase locked loop, and divide the tones at 2125 Hz
		// with a little hysteresis
		if ((m_rise_cycle >= 0.0) && (m_fall_cycle >= 0.0))
		{
			double const average = (m_rise_cycle + m_fall_cycle) / 2.0;
			if (m_receive && (average > (1.0 / 2125.0) + 20e-6))
				receive(0);
			else if (!m_receive && (average < (1.0 / 2125.0) - 20e-6))
				receive(1);
		}
	}
	else if (((now - m_rise) > attotime::from_msec(2)) && ((now - m_fall) > attotime::from_msec(2)))
	{
		receive(1);
	}
}

void s100_mits_acr_device::receive(int state)
{
	if (state != m_receive)
	{
		m_receive = state;
		m_uart->write_si(state);
	}
}

void s100_mits_acr_device::update_motor()
{
	// the relay hangs on the input interrupt flip-flop
	m_cassette->set_motor(!BIT(m_motor_relay->read(), 0) || m_input_int_enable);
}


void s100_mits_acr_device::device_add_mconfig(machine_config &config)
{
	s100_mits_sio_device::device_add_mconfig(config);

	// the modem takes the place of a serial connector
	config.device_remove("rs232");
	m_uart->write_so_callback().set([this] (int state) { m_transmit = state; });

	SPEAKER(config, "mono").front_center();

	CASSETTE(config, m_cassette);
	m_cassette->set_formats(mits_cassette_formats);
	m_cassette->set_default_state(CASSETTE_STOPPED | CASSETTE_MOTOR_ENABLED | CASSETTE_SPEAKER_ENABLED);
	m_cassette->add_route(ALL_OUTPUTS, "mono", 0.05);
	m_cassette->set_interface("88acr_cass");

	SOFTWARE_LIST(config, "cass_list").set_original("88acr_cass");
}


static INPUT_PORTS_START( mits_acr )
	PORT_INCLUDE(mits_sio)

	// wired for 006 octal and 300 baud, as MITS software expects
	PORT_MODIFY("ADDRESS")
	ADDRESS_JUMPER(2, 1)
	ADDRESS_JUMPER(1, 1)

	PORT_MODIFY("BAUD")
	PORT_CONFNAME(0x0f, 0x02, "Baud rate")
	BAUD_RATE_SETTINGS

	PORT_START("MOTOR")
	PORT_CONFNAME(0x01, 0x00, "Recorder motor relay on D0")
	PORT_CONFSETTING(0x00, "Not fitted")
	PORT_CONFSETTING(0x01, "Fitted")
INPUT_PORTS_END

ioport_constructor s100_mits_acr_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_acr);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_SIO, device_s100_card_interface, s100_mits_sio_device, "s100_mits_sio", "MITS 88-SIO Serial Interface")
DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_ACR, device_s100_card_interface, s100_mits_acr_device, "s100_mits_acr", "MITS 88-ACR Audio Cassette Interface")
