// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-PIO parallel interface

    The Altair's first parallel board, from 1975: an 8212 latch for eight
    bits in and another for eight bits out, each with a strobe line from its
    device. It takes two ports, control/status at an even address and data
    at the next one up, set by jumpers on A7-A1. DBL sends its error code to
    the data port at 005 octal, so the default here is 004.

    A status read (IN from the control port) drives only D0 and D1. D0 is
    high when the output device is ready for a byte and D1 when the input
    device has sent one; each comes from the service request flip-flop of
    that device's 8212, which the device's strobe sets. D2-D7 are not driven
    and read high here. An OUT to the control port enables the output
    interrupt with D0 and the input interrupt with D1, and the data port
    reads the input latch and writes the output latch.

    Not emulated
    - anything on the connectors. No device strobes either latch, so both
      ready bits stay low, the input latch reads FF, and nothing can
      interrupt; the jumpers that take the interrupts to the 88-VI or PINT
      are left out

    References
    - MITS Parallel I/O Board documentation, 1975
      [https://deramp.com/downloads/altair/hardware/MITS%2088-PIO.pdf]

**********************************************************************/

#include "emu.h"
#include "mitspio.h"


namespace {

class s100_mits_pio_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_pio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_sinp_r(offs_t offset) override;

private:
	// A7-A1 select the board, A0 the control or data port
	bool selected(offs_t offset) { return (offset & 0xfe) == m_address->read(); }

	required_ioport m_address;
};


s100_mits_pio_device::s100_mits_pio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_PIO, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_address(*this, "ADDRESS")
{
}


void s100_mits_pio_device::device_start()
{
}


u8 s100_mits_pio_device::s100_sinp_r(offs_t offset)
{
	if (!selected(offset))
		return 0xff;

	// With nothing connected, neither ready flip-flop is ever set and the
	// input latch's inputs float high.
	return BIT(offset, 0) ? 0xff : 0xfc;
}


#define ADDRESS_JUMPER(bit, dflt) \
	PORT_CONFNAME(1 << bit, (dflt) << bit, "Address jumper A" #bit) \
	PORT_CONFSETTING(0, "0") \
	PORT_CONFSETTING(1 << bit, "1")

static INPUT_PORTS_START( mits_pio )
	// A7-A1 are compared with the jumpers; the default is 004 octal
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(7, 0)
	ADDRESS_JUMPER(6, 0)
	ADDRESS_JUMPER(5, 0)
	ADDRESS_JUMPER(4, 0)
	ADDRESS_JUMPER(3, 0)
	ADDRESS_JUMPER(2, 1)
	ADDRESS_JUMPER(1, 0)
INPUT_PORTS_END

ioport_constructor s100_mits_pio_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_pio);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_PIO, device_s100_card_interface, s100_mits_pio_device, "s100_mits_pio", "MITS 88-PIO Parallel Interface")
