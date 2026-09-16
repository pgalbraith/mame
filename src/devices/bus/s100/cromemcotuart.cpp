// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    Cromemco TU-ART digital interface (1977)

    Twin Universal Asynchronous Receiver and Transmitter: two TMS5501s,
    which between them give two serial ports, two 8-bit parallel ports and
    ten interval timers. The card is how a Z-2 without a disk controller
    gets a console, and the Z-2 manual lists it as the I/O interface card
    of the minimum system.

    The two chips are called Device A and Device B. Device A is IC4 and
    drives serial connector J4 and parallel connector J2; Device B is IC5
    and drives J5 and J3.

    Ports
    Each device answers fourteen ports at a base address of its own, set by
    a DIP switch that compares A4 to A7. The low four bits of the I/O
    address then pick the register:

    0   IN  status                 OUT baud rate
    1   IN  receiver data          OUT transmitter data
    2                              OUT command
    3   IN  interrupt address      OUT interrupt mask
    4   IN  parallel port          OUT parallel port
    5-9                            OUT timers 1 to 5

    The status byte is not the TMS5501's own order. Software sees TBE, RDA,
    IPG, SBD, FBD, SRV, ORE and FME, which is the same order the 4FDC puts
    on the bus, so one console driver works against either card. That is
    what the Z-80 Monitor relies on: it reads status at the base address and
    data one above, and finds the baud rate by writing each entry of a table
    to the baud rate register until two carriage returns arrive.

    If both devices are switched to the same base address, Device A wins.

    Not emulated
    - The parallel ports on J2 and J3. Nothing on the MAME side would
      connect to them, so the input reads high and the output goes nowhere.
    - The Z-80 mode 2 vector. Device A's A5, A6 and A7 switches also drive
      D5, D6 and D7 of the byte the card returns during interrupt
      acknowledge, but s100_bus_device has no vector fetch and the Z-80
      Monitor polls.
    - The interrupt priority chain on J1, for the same reason the 4FDC does
      not have it.
    - Address reverse, the switch that lets an output to a parallel port
      swap the two base addresses so one driver can talk to either device.
      It needs the parallel ports.
    - The spare RS-232 drivers and receivers, and the originate mode
      modification for an acoustic coupler.

    References
    - TU-ART manual 023-0011, January 1980. Switches in section 1.2, the
      port table in figure 2, register descriptions in section 2, interrupt
      operation in section 3.
    - Z-2/Z-2D instruction manual, 1978, which lists the minimum system and
      says the Z-80 Monitor sets the baud rate of the TU-ART or the 4FDC to
      match the terminal.

**********************************************************************/

#include "emu.h"
#include "cromemcotuart.h"

#include "bus/rs232/rs232.h"
#include "machine/tms5501.h"


namespace {

class s100_cromemco_tuart_device : public device_t, public device_s100_card_interface
{
public:
	s100_cromemco_tuart_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_sinp_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;

private:
	// which device answers this port, or -1 for neither
	int device_for(offs_t offset) const;

	u8 status_r(int device) const;

	required_device_array<tms5501_device, 2> m_uart;
	required_ioport m_switches;
};

s100_cromemco_tuart_device::s100_cromemco_tuart_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_CROMEMCO_TUART, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_uart(*this, "uart%u", 0U)
	, m_switches(*this, "SWITCHES")
{
}


void s100_cromemco_tuart_device::device_start()
{
	// the two TMS5501s hold all of the card's state
}


//**************************************************************************
//  I/O
//**************************************************************************

int s100_cromemco_tuart_device::device_for(offs_t offset) const
{
	// the card decodes A0-A7 only, so ignore whatever the CPU left in the
	// high half of the I/O address
	u8 const base = offset & 0xf0;
	u8 const switches = m_switches->read();

	// Device A is checked first, so it overrides Device B if the two are
	// switched to the same base address
	if (base == ((switches & 0x0f) << 4))
		return 0;

	if (base == (switches & 0xf0))
		return 1;

	return -1;
}

u8 s100_cromemco_tuart_device::status_r(int device) const
{
	// The jumper header reorders the TMS5501's status bits on the way to the
	// bus: TBE, RDA, IPG, SBD, FBD, SRV, ORE, FME.
	return bitswap<8>(m_uart[device]->sta_r(), 4, 3, 5, 7, 6, 2, 1, 0);
}

u8 s100_cromemco_tuart_device::s100_sinp_r(offs_t offset)
{
	int const device = device_for(offset);

	if (device < 0)
		return 0xff;

	switch (offset & 0x0f)
	{
	case 0x00:
		return status_r(device);
	case 0x01:
		return m_uart[device]->rb_r();
	case 0x03:
		return m_uart[device]->rst_r();
	case 0x04:
		return m_uart[device]->xi_r();
	}

	// IN 2 and IN 5 to IN 15 are unassigned and free for system use
	return 0xff;
}

void s100_cromemco_tuart_device::s100_sout_w(offs_t offset, u8 data)
{
	int const device = device_for(offset);

	if (device < 0)
		return;

	switch (offset & 0x0f)
	{
	case 0x00:
		m_uart[device]->rr_w(data);
		break;
	case 0x01:
		m_uart[device]->tb_w(data);
		break;
	case 0x02:
		m_uart[device]->cmd_w(data);
		break;
	case 0x03:
		m_uart[device]->mr_w(data);
		break;
	case 0x04:
		m_uart[device]->xo_w(data);
		break;
	case 0x05: case 0x06: case 0x07: case 0x08: case 0x09:
		m_uart[device]->tmr_w((offset & 0x0f) - 0x05, data);
		break;
	}
}


//**************************************************************************
//  CONFIGURATION
//**************************************************************************

void s100_cromemco_tuart_device::device_add_mconfig(machine_config &config)
{
	// The card has its own crystal and talks to the bus asynchronously, so
	// the CPU clock does not matter. Both chips run from the same divider
	// the 4FDC uses.
	TMS5501(config, m_uart[0], 8_MHz_XTAL / 4);
	m_uart[0]->xmt_callback().set("rs232a", FUNC(rs232_port_device::write_txd));
	m_uart[0]->int_callback().set([this] (int state) { m_bus->irq_w(state); });

	TMS5501(config, m_uart[1], 8_MHz_XTAL / 4);
	m_uart[1]->xmt_callback().set("rs232b", FUNC(rs232_port_device::write_txd));
	m_uart[1]->int_callback().set([this] (int state) { m_bus->irq_w(state); });

	// J4, Device A's serial port, is where the console goes. The ADM-3A is
	// the default for the same reason it is on the 4FDC: the Z-80 Monitor
	// finds the rate from the carriage returns the operator presses, and
	// 9600 is the second entry in its table.
	rs232_port_device &rs232a(RS232_PORT(config, "rs232a", default_rs232_devices, "adm3a"));
	rs232a.rxd_handler().set(m_uart[0], FUNC(tms5501_device::rcv_w));

	// J5, Device B's serial port, is empty as shipped
	rs232_port_device &rs232b(RS232_PORT(config, "rs232b", default_rs232_devices, nullptr));
	rs232b.rxd_handler().set(m_uart[1], FUNC(tms5501_device::rcv_w));
}


static INPUT_PORTS_START( cromemco_tuart )

	// Switch positions 6 to 3 set Device A's base address and positions 10
	// to 7 set Device B's, each comparing one address line from A4 to A7.
	// The manual's own example is Device A at 00 and Device B at F0, which
	// is what the card comes up with here.
	//
	// Base addresses to keep clear: 00 and 30 are the 4FDC, 40 is the bank
	// select port every Cromemco memory board answers, and 50 is the PRI
	// printer interface.
	PORT_START("SWITCHES")
	PORT_DIPNAME(0x0f, 0x00, "Device A base address")
	PORT_DIPSETTING(0x00, "00")
	PORT_DIPSETTING(0x01, "10")
	PORT_DIPSETTING(0x02, "20")
	PORT_DIPSETTING(0x03, "30")
	PORT_DIPSETTING(0x04, "40")
	PORT_DIPSETTING(0x05, "50")
	PORT_DIPSETTING(0x06, "60")
	PORT_DIPSETTING(0x07, "70")
	PORT_DIPSETTING(0x08, "80")
	PORT_DIPSETTING(0x09, "90")
	PORT_DIPSETTING(0x0a, "A0")
	PORT_DIPSETTING(0x0b, "B0")
	PORT_DIPSETTING(0x0c, "C0")
	PORT_DIPSETTING(0x0d, "D0")
	PORT_DIPSETTING(0x0e, "E0")
	PORT_DIPSETTING(0x0f, "F0")
	PORT_DIPNAME(0xf0, 0xf0, "Device B base address")
	PORT_DIPSETTING(0x00, "00")
	PORT_DIPSETTING(0x10, "10")
	PORT_DIPSETTING(0x20, "20")
	PORT_DIPSETTING(0x30, "30")
	PORT_DIPSETTING(0x40, "40")
	PORT_DIPSETTING(0x50, "50")
	PORT_DIPSETTING(0x60, "60")
	PORT_DIPSETTING(0x70, "70")
	PORT_DIPSETTING(0x80, "80")
	PORT_DIPSETTING(0x90, "90")
	PORT_DIPSETTING(0xa0, "A0")
	PORT_DIPSETTING(0xb0, "B0")
	PORT_DIPSETTING(0xc0, "C0")
	PORT_DIPSETTING(0xd0, "D0")
	PORT_DIPSETTING(0xe0, "E0")
	PORT_DIPSETTING(0xf0, "F0")

INPUT_PORTS_END

ioport_constructor s100_cromemco_tuart_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cromemco_tuart);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_CROMEMCO_TUART, device_s100_card_interface, s100_cromemco_tuart_device, "cromemco_tuart", "Cromemco TU-ART Digital Interface")
