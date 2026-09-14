// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-C700 Centronics printer interface

    The interface for the Altair C700, a bidirectional dot matrix printer
    with a Centronics connection. It takes two ports, status in and control
    out at an even address and data out at the next one up, set by switches
    SW3-1 to SW3-4 on A1-A4 and SW2-1 to SW2-3 on A5-A7. MITS software
    expects 002 octal.

    OUT to the control port: D0 low pulses PRIME, which clears the printer's
    buffer and sends its head home, and D1 enables the interrupt when high
    and disables it when low. IN from the control port: D0 high once the
    printer has acknowledged the last character, D1 BUSY, D2 PAPER EMPTY,
    D3 low while the printer is selected, D4 FAULT, D6 interrupt enabled
    and D7 interrupt requested. The manual does not describe D5, which
    reads 0 here.

    OUT to the data port latches all eight bits for the printer, clears the
    acknowledge latch and any interrupt request, and strobes the byte into
    the printer with a 1.5 us pulse starting 1.5 us later. The acknowledge
    latch is clear at power on, so software has to send something before it
    can wait for the latch: Disk BASIC 4.1 sends DC1, which selects the
    printer, when it starts.

    With the interrupt enabled, SW2-4 picks what requests one: on, the end
    of each acknowledge; off, the end of BUSY, which the C700 raises only
    for a print, carriage return or line feed. A data OUT takes the request
    back. SW1 connects it to PINT or to the 88-VI's lines.

    Any Centronics device can be connected, and an empty connector reads
    as the board's pull-ups. MAME's printer saves the bytes as they are
    sent: Disk BASIC ends each line with a carriage return alone, since the
    C700 prints and feeds a line on it. That printer is busy for every
    character, so it requests an interrupt after each one either way.

    References
    - MITS C700 printer interface documentation, March 1977, with the
      schematic

**********************************************************************/

#include "emu.h"
#include "mitsc700.h"

#include "bus/centronics/ctronics.h"


namespace {

class s100_mits_c700_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_c700_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

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
	// A7-A1 select the board, A0 the control or data port
	bool selected(offs_t offset) { return (offset & 0xfe) == m_address->read(); }

	// the signal SW2-4 picks to clock the interrupt request flip-flop
	bool interrupt_clock() const { return m_ack_clocks ? m_ack_latch : !m_busy; }
	void clock_interrupt(bool before);
	void set_irq(bool state);

	void ack_w(int state);
	void busy_w(int state);
	void perror_w(int state) { m_perror = state; }
	void select_w(int state) { m_select = state; }
	void fault_w(int state) { m_fault = state; }

	TIMER_CALLBACK_MEMBER(strobe_tick);

	required_device<centronics_device> m_centronics;
	required_device<output_latch_device> m_data_latch;
	required_ioport m_address;
	required_ioport m_options;
	required_ioport m_irq_switch;

	emu_timer *m_strobe_timer;

	// printer lines at the connector, which pull up when nothing drives them
	u8 m_ack;
	u8 m_busy;
	u8 m_perror;
	u8 m_select;
	u8 m_fault;

	bool m_ack_latch;       // IC D
	bool m_int_enable;      // IC C, first half
	bool m_int_request;     // IC C, second half
	bool m_ack_clocks;      // SW2-4 on
	bool m_irq_state;
	u8 m_irq_route;         // 0 not connected, 1 PINT, 2-9 VI0-VI7
};


s100_mits_c700_device::s100_mits_c700_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_C700, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_centronics(*this, "centronics")
	, m_data_latch(*this, "data_latch")
	, m_address(*this, "ADDRESS")
	, m_options(*this, "OPTIONS")
	, m_irq_switch(*this, "IRQ")
	, m_strobe_timer(nullptr)
	, m_ack(1)
	, m_busy(1)
	, m_perror(1)
	, m_select(1)
	, m_fault(1)
	, m_ack_latch(false)
	, m_int_enable(false)
	, m_int_request(false)
	, m_ack_clocks(true)
	, m_irq_state(false)
	, m_irq_route(0)
{
}


void s100_mits_c700_device::device_start()
{
	m_strobe_timer = timer_alloc(FUNC(s100_mits_c700_device::strobe_tick), this);

	save_item(NAME(m_ack));
	save_item(NAME(m_busy));
	save_item(NAME(m_perror));
	save_item(NAME(m_select));
	save_item(NAME(m_fault));
	save_item(NAME(m_ack_latch));
	save_item(NAME(m_int_enable));
	save_item(NAME(m_int_request));
	save_item(NAME(m_ack_clocks));
	save_item(NAME(m_irq_state));
	save_item(NAME(m_irq_route));
}

void s100_mits_c700_device::device_reset()
{
	// drop the interrupt line of the old switch setting before taking the new one
	if (m_irq_state)
		set_irq(false);
	m_irq_route = m_irq_switch->read();
	m_ack_clocks = BIT(m_options->read(), 0);

	// power-on clear
	m_ack_latch = false;
	m_int_enable = false;
	m_int_request = false;

	// STB rests high, so the first byte sent has a falling edge to strobe it
	m_strobe_timer->adjust(attotime::never);
	m_centronics->write_strobe(1);
}


u8 s100_mits_c700_device::s100_sinp_r(offs_t offset)
{
	// the data port is output only
	if (!selected(offset) || BIT(offset, 0))
		return 0xff;

	u8 data = 0x00;
	if (m_ack_latch)
		data |= 0x01;
	if (m_busy)
		data |= 0x02;
	if (m_perror)
		data |= 0x04;
	if (!m_select)
		data |= 0x08;
	if (!m_fault)
		data |= 0x10;
	if (m_int_enable)
		data |= 0x40;
	if (m_int_request)
		data |= 0x80;
	return data;
}

void s100_mits_c700_device::s100_sout_w(offs_t offset, u8 data)
{
	if (!selected(offset))
		return;

	if (BIT(offset, 0))
	{
		m_data_latch->write(data);
		m_ack_latch = false;
		if (m_int_request)
		{
			m_int_request = false;
			set_irq(false);
		}
		m_strobe_timer->adjust(attotime::from_nsec(1500), 0);
	}
	else
	{
		m_int_enable = BIT(data, 1);
		if (!BIT(data, 0))
		{
			m_centronics->write_init(0);
			m_centronics->write_init(1);
		}
	}
}


void s100_mits_c700_device::clock_interrupt(bool before)
{
	// the request flip-flop takes the interrupt enable on a rising edge
	if (!before && interrupt_clock() && (m_int_request != m_int_enable))
	{
		m_int_request = m_int_enable;
		set_irq(m_int_request);
	}
}

void s100_mits_c700_device::set_irq(bool state)
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


void s100_mits_c700_device::ack_w(int state)
{
	// the latch sets as /ACK goes high again at the end of the pulse
	if (state && !m_ack)
	{
		bool const before = interrupt_clock();
		m_ack_latch = true;
		clock_interrupt(before);
	}
	m_ack = state;
}

void s100_mits_c700_device::busy_w(int state)
{
	bool const before = interrupt_clock();
	m_busy = state;
	clock_interrupt(before);
}


TIMER_CALLBACK_MEMBER(s100_mits_c700_device::strobe_tick)
{
	m_centronics->write_strobe(param);
	if (!param)
		m_strobe_timer->adjust(attotime::from_nsec(1500), 1);
}


void s100_mits_c700_device::device_add_mconfig(machine_config &config)
{
	CENTRONICS(config, m_centronics, centronics_devices, "printer");
	m_centronics->ack_handler().set(FUNC(s100_mits_c700_device::ack_w));
	m_centronics->busy_handler().set(FUNC(s100_mits_c700_device::busy_w));
	m_centronics->perror_handler().set(FUNC(s100_mits_c700_device::perror_w));
	m_centronics->select_handler().set(FUNC(s100_mits_c700_device::select_w));
	m_centronics->fault_handler().set(FUNC(s100_mits_c700_device::fault_w));

	OUTPUT_LATCH(config, m_data_latch);
	m_centronics->set_output_latch(*m_data_latch);
}


#define ADDRESS_SWITCH(bit, name, dflt) \
	PORT_CONFNAME(1 << bit, (dflt) << bit, name) \
	PORT_CONFSETTING(0, "Off") \
	PORT_CONFSETTING(1 << bit, "On")

static INPUT_PORTS_START( mits_c700 )
	// A7-A1 are compared with the switches; the default is 002 octal
	PORT_START("ADDRESS")
	ADDRESS_SWITCH(7, "SW2-3 (A7)", 0)
	ADDRESS_SWITCH(6, "SW2-2 (A6)", 0)
	ADDRESS_SWITCH(5, "SW2-1 (A5)", 0)
	ADDRESS_SWITCH(4, "SW3-4 (A4)", 0)
	ADDRESS_SWITCH(3, "SW3-3 (A3)", 0)
	ADDRESS_SWITCH(2, "SW3-2 (A2)", 0)
	ADDRESS_SWITCH(1, "SW3-1 (A1)", 1)

	PORT_START("OPTIONS")
	PORT_CONFNAME(0x01, 0x01, "Interrupt after (SW2-4)")
	PORT_CONFSETTING(0x00, "Each carriage return or line feed (off)")
	PORT_CONFSETTING(0x01, "Each character (on)")

	PORT_START("IRQ")
	PORT_CONFNAME(0x0f, 0x00, "Interrupt (SW1)")
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

ioport_constructor s100_mits_c700_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_c700);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_C700, device_s100_card_interface, s100_mits_c700_device, "s100_mits_c700", "MITS 88-C700 Centronics Printer Interface")
