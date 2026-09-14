// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-4PIO four port parallel interface

    One to four 6820 PIAs, ICs J, K, L and M. Each is a port of two
    sections, A and B, with eight data lines and two control lines apiece,
    and everything about them but the board's address is set by software
    through the PIA registers.

    The board takes 16 I/O addresses. Jumpers compare A7-A4, A3-A2 pick the
    port, and A1-A0 the register: base+0 A control, base+1 A data or data
    direction, base+2 B control, base+3 B data or data direction. So each
    control register sits below its data register, the other way round from
    the 6820's own register select order. DBL expects the board at 040
    octal: it sends its error code to port 0's B data register at 043.

    The parts list is for a one port board, which is the default here; the
    PIAs for ports K, L and M, with their connector cables, were options. A
    port with no PIA reads as FF.

    Not emulated
    - anything on the ports' connectors. The inputs read high, as
      unconnected TTL inputs do, so no port can interrupt, and the jumpers
      that take each section's interrupt request to PINT are left out
    - the wait state the board inserts on every IN, about 500 ns

    References
    - MITS 88-4PIO documentation, third printing, March 1977
      [https://deramp.com/downloads/altair/hardware/MITS%2088-4PIO.pdf]

**********************************************************************/

#include "emu.h"
#include "mits4pio.h"

#include "machine/6821pia.h"


namespace {

class s100_mits_4pio_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_4pio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_sinp_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;

private:
	// A7-A4 select the board and A3-A2 a port, which answers only if its PIA is fitted
	bool selected(offs_t offset) { return ((offset & 0xf0) == m_address->read()) && (BIT(offset, 2, 2) < m_ports->read()); }

	// A1-A0 of 00 is the A control register, which the PIA has at register 01
	static offs_t pia_register(offs_t offset) { return (offset & 0x03) ^ 0x01; }

	required_device_array<pia6821_device, 4> m_pia;
	required_ioport m_address;
	required_ioport m_ports;
};


s100_mits_4pio_device::s100_mits_4pio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_4PIO, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_pia(*this, "pia%u", 0U)
	, m_address(*this, "ADDRESS")
	, m_ports(*this, "PORTS")
{
}


void s100_mits_4pio_device::device_start()
{
}


u8 s100_mits_4pio_device::s100_sinp_r(offs_t offset)
{
	if (!selected(offset))
		return 0xff;

	return m_pia[BIT(offset, 2, 2)]->read(pia_register(offset));
}

void s100_mits_4pio_device::s100_sout_w(offs_t offset, u8 data)
{
	if (selected(offset))
		m_pia[BIT(offset, 2, 2)]->write(pia_register(offset), data);
}


void s100_mits_4pio_device::device_add_mconfig(machine_config &config)
{
	for (unsigned port = 0; port < 4; port++)
	{
		PIA6821(config, m_pia[port]);

		// nothing is connected: the inputs float high and the outputs go nowhere
		m_pia[port]->readpa_handler().set_constant(0xff);
		m_pia[port]->readpb_handler().set_constant(0xff);
		m_pia[port]->readca1_handler().set_constant(1);
		m_pia[port]->readca2_handler().set_constant(1);
		m_pia[port]->readcb1_handler().set_constant(1);
		m_pia[port]->writepa_handler().set_nop();
		m_pia[port]->writepb_handler().set_nop();
		m_pia[port]->ca2_handler().set_nop();
		m_pia[port]->cb2_handler().set_nop();
	}
}


#define ADDRESS_JUMPER(bit, dflt) \
	PORT_CONFNAME(1 << bit, (dflt) << bit, "Address jumper A" #bit) \
	PORT_CONFSETTING(0, "0") \
	PORT_CONFSETTING(1 << bit, "1")

static INPUT_PORTS_START( mits_4pio )
	// A7-A4 are compared with the jumpers; the default is 040 octal
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(7, 0)
	ADDRESS_JUMPER(6, 0)
	ADDRESS_JUMPER(5, 1)
	ADDRESS_JUMPER(4, 0)

	PORT_START("PORTS")
	PORT_CONFNAME(0x07, 0x01, "Ports fitted")
	PORT_CONFSETTING(0x01, "1 (IC J)")
	PORT_CONFSETTING(0x02, "2 (ICs J and K)")
	PORT_CONFSETTING(0x03, "3 (ICs J, K and L)")
	PORT_CONFSETTING(0x04, "4 (ICs J, K, L and M)")
INPUT_PORTS_END

ioport_constructor s100_mits_4pio_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_4pio);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_4PIO, device_s100_card_interface, s100_mits_4pio_device, "s100_mits_4pio", "MITS 88-4PIO Parallel Interface")
