// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-VI vectored interrupt card and 88-RTC real time clock

    The 88-VI takes the eight vectored interrupt lines, VI0 the highest
    priority and VI7 the lowest, through an 8214 priority interrupt unit.
    When the highest line asking is above the level the software last set,
    the card pulls PINT, and when the CPU acknowledges the interrupt the
    card answers with RST 0 to RST 7 for that line, pulling down DI3-DI5
    while the bus pull-ups give the other bits.

    An OUT to 376 octal sets it up:
    - D2-D0: the level now being served, written as 7 minus the level, as
      in the manual's table of service routines
    - D3: 1 lets only lines above that level interrupt; 0, as MITS's
      initialisation writes it, lets any line interrupt
    - D4: 1 clears the RTC's interrupt
    - D5: 1 clears the RTC's dividers
    - D6: enables the RTC's interrupt
    - D7: enables the vectored interrupt structure
    Power-on clear disables everything.

    The 88-RTC, sold with the 88-VI or fitted to it, divides the line
    frequency or 10 kHz from the 2 MHz bus clock (jumper S) by 1, 10, 100
    or 1000 (jumper IN), and sets its interrupt on every cycle while it is
    enabled. Jumper RI takes that interrupt to one of the VI lines, and the
    service routine clears it with D4.

    Not emulated
    - two cards on one VI line; the line follows whichever changed last
    - the 8214's clocked latching: lines are compared as they change, and
      an acknowledge is answered for the line asking at that moment
    - the line frequency is taken as 60 Hz

    References
    - MITS 88-VI and 88-RTC documentation, 1976
      [https://deramp.com/downloads/altair/hardware/Vectored%20Interrupt%20(88-VI-RTC).pdf]

**********************************************************************/

#include "emu.h"
#include "mitsvi.h"


namespace {

class s100_mits_vi_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_vi_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual void s100_vi0_w(int state) override { line_w(0, state); }
	virtual void s100_vi1_w(int state) override { line_w(1, state); }
	virtual void s100_vi2_w(int state) override { line_w(2, state); }
	virtual void s100_vi3_w(int state) override { line_w(3, state); }
	virtual void s100_vi4_w(int state) override { line_w(4, state); }
	virtual void s100_vi5_w(int state) override { line_w(5, state); }
	virtual void s100_vi6_w(int state) override { line_w(6, state); }
	virtual void s100_vi7_w(int state) override { line_w(7, state); }
	virtual u8 s100_sinta_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;

private:
	void line_w(unsigned line, int state);
	void update_pint();
	void restart_rtc();

	TIMER_CALLBACK_MEMBER(rtc_cycle);

	required_ioport m_rtc_jumpers;

	emu_timer *m_rtc_timer;

	u8 m_lines;             // VI lines asking, bit n for VIn
	u8 m_control;           // the last OUT to 376 octal
	bool m_rtc_int;         // IC Fb
	u8 m_rtc_line;          // jumper RI: 0 not connected, 1-8 VI0-VI7
	bool m_pint;
	u8 m_vector;            // IC C, the line last accepted
};


s100_mits_vi_device::s100_mits_vi_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_VI, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_rtc_jumpers(*this, "RTC")
	, m_rtc_timer(nullptr)
	, m_lines(0)
	, m_control(0)
	, m_rtc_int(false)
	, m_rtc_line(0)
	, m_pint(false)
	, m_vector(7)
{
}


void s100_mits_vi_device::device_start()
{
	m_rtc_timer = timer_alloc(FUNC(s100_mits_vi_device::rtc_cycle), this);

	save_item(NAME(m_lines));
	save_item(NAME(m_control));
	save_item(NAME(m_rtc_int));
	save_item(NAME(m_rtc_line));
	save_item(NAME(m_pint));
	save_item(NAME(m_vector));
}

void s100_mits_vi_device::device_reset()
{
	// power-on clear disables everything
	m_control = 0;
	m_rtc_int = false;
	m_rtc_line = m_rtc_jumpers->read() & 0x0f;
	restart_rtc();
	update_pint();
}


u8 s100_mits_vi_device::s100_sinta_r(offs_t offset)
{
	// DI3-DI5 carry the line's number into an RST; the pull-ups give the rest
	return BIT(m_control, 7) ? (0xc7 | (m_vector << 3)) : 0xff;
}

void s100_mits_vi_device::s100_sout_w(offs_t offset, u8 data)
{
	if ((offset & 0xff) != 0xfe)
		return;

	m_control = data;
	if (BIT(data, 4))
		m_rtc_int = false;
	if (BIT(data, 5))
		restart_rtc();
	update_pint();
}


void s100_mits_vi_device::line_w(unsigned line, int state)
{
	if (state)
		m_lines |= 1 << line;
	else
		m_lines &= ~(1 << line);
	update_pint();
}

void s100_mits_vi_device::update_pint()
{
	u8 asking = m_lines;
	if (m_rtc_int && m_rtc_line)
		asking |= 1 << (m_rtc_line - 1);

	// the highest line asking interrupts if it is above the current level,
	// where the 8214 counts VI0 as priority 7 and VI7 as priority 0
	bool pint = false;
	if (BIT(m_control, 7) && asking)
	{
		unsigned line = 0;
		while (!BIT(asking, line))
			line++;
		if (!BIT(m_control, 3) || ((7 - line) > (m_control & 7)))
		{
			pint = true;
			m_vector = line;
		}
	}

	if (pint != m_pint)
	{
		m_pint = pint;
		m_bus->irq_w(pint ? 1 : 0);
	}
}

void s100_mits_vi_device::restart_rtc()
{
	static constexpr double DIVIDERS[] = { 1.0, 10.0, 100.0, 1000.0 };

	ioport_value const jumpers = m_rtc_jumpers->read();
	double const source = BIT(jumpers, 4) ? 10'000.0 : 60.0;
	attotime const period = attotime::from_hz(source / DIVIDERS[(jumpers >> 5) & 3]);
	m_rtc_timer->adjust(period, 0, period);
}


TIMER_CALLBACK_MEMBER(s100_mits_vi_device::rtc_cycle)
{
	if (BIT(m_control, 6))
	{
		m_rtc_int = true;
		update_pint();
	}
}


static INPUT_PORTS_START( mits_vi )
	PORT_START("RTC")
	PORT_CONFNAME(0x0f, 0x00, "RTC interrupt (jumper RI)")
	PORT_CONFSETTING(0x00, "Not connected")
	PORT_CONFSETTING(0x01, "VI0")
	PORT_CONFSETTING(0x02, "VI1")
	PORT_CONFSETTING(0x03, "VI2")
	PORT_CONFSETTING(0x04, "VI3")
	PORT_CONFSETTING(0x05, "VI4")
	PORT_CONFSETTING(0x06, "VI5")
	PORT_CONFSETTING(0x07, "VI6")
	PORT_CONFSETTING(0x08, "VI7")
	PORT_CONFNAME(0x10, 0x00, "RTC source (jumper S)")
	PORT_CONFSETTING(0x00, "Line frequency, 60 Hz (LF)")
	PORT_CONFSETTING(0x10, "10 kHz from the bus clock (CF)")
	PORT_CONFNAME(0x60, 0x00, "RTC divider (jumper IN)")
	PORT_CONFSETTING(0x00, "1 (A)")
	PORT_CONFSETTING(0x20, "10 (B)")
	PORT_CONFSETTING(0x40, "100 (C)")
	PORT_CONFSETTING(0x60, "1000 (D)")
INPUT_PORTS_END

ioport_constructor s100_mits_vi_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_vi);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_VI, device_s100_card_interface, s100_mits_vi_device, "s100_mits_vi", "MITS 88-VI Vectored Interrupt and 88-RTC Real Time Clock")
