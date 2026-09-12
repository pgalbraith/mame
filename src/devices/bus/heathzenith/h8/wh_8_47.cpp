// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath H47 Interface and Serial I/O Card

    Model number WH8-47.  The H8 end of the H-47 8" floppy disk system, and
    two RS-232C channels that have nothing to do with it beyond sharing the
    board: "In addition to the disk interface circuitry, this Card contains
    two channels of RS232C asynchronous serial interface for interfacing to
    such devices as the Heath H19 Terminal, H14 Printer, or a modem."  The
    quotations here and the jumper names are from the card's Operation
    manual (595-2469, in
    [https://sebhc.github.io/sebhc/documentation/hardware/H8/H8-47_Op.zip]).

    The disk half is described in bus/heathzenith/h47/h47_intf.cpp, and the
    cabinet it talks to in bus/heathzenith/h47/h47.cpp.  What belongs to
    this card is where it answers and where its interrupts go.

    THE DISK ADDRESS
    ----------------
    "These jumpers select ports 170 through 173 for the H47 disk system",
    which is the setting Pictorial 3 shows and the one Heath's software
    expects.  The decoder takes the first two octal digits from jumpers and
    the third from a pair of ranges, XX0-XX3 or XX4-XX7, so the block is
    four ports wide and the two the card uses appear twice in it.  The
    monitors know the two blocks as Disk I/O #1 (170) and Disk I/O #2 (174),
    the same pair an H-89 decodes, and those are the two offered here; the
    H8's own configuration switch names them the "Port 170 device" and the
    "Port 174 device".

    A DISK ENABLE jumper turns the whole disk section off, which is how the
    card is fitted when it is wanted only for its two serial channels.

    THE SERIAL ADDRESSES
    --------------------
    "Any address may be assigned to any channel, but two channels must not be
    assigned the same address."  The six the manual's table of I/O address
    assignments lists are offered here:

        300-307 octal   alternate terminal 0
        310-317 octal   alternate terminal 1
        320-327 octal   alternate terminal 2, HDOS AT:
        330-337 octal   alternate terminal 3
        340-347 octal   line printer, HDOS LP:
        350-357 octal   console terminal, HDOS TT:

    Both channels ship switched off, for the reason the H-8-4 leaves its
    fourth port off: HDOS moves its console to a card it finds at 350 octal,
    and an H8 whose console has gone somewhere with nothing attached looks
    dead.  Turn a channel on and attach a terminal together.

    THE INTERRUPTS
    --------------
    Three sources - the disk and the two channels - reach a patch area where
    "the interrupt signal from the disk I/O or either serial I/O channel can
    be connected to any H8 bus interrupt line (INT3, 4, 5, 6, or 7) by jumper
    plugs".  The installation steps set the disk one: "Set DISK INTERRUPT
    jumper J104 to 5", which is the default here.  Nothing that drives this
    card is known to use it - HDOS's H47 driver and MTR-90 both poll the
    status port and leave the interrupt enable bit clear - so it costs
    nothing either way.

    THE MONITOR DISABLE
    -------------------
    The card can bank out the H8's monitor ROM: "This is done by setting D5
    to a logic 1 and outputting it to I/O port 362 ... which latches the D5
    bit into flip-flop U105B.  This latched D5 bit is inverted by an open
    collector gate (U106B) and applied to pin 46 of the H8 bus."  That is
    bit for bit what the HA-8-8's and the HA-8-6's port 362 write does, and
    pin 46 is wire-ORed across the slots, so on real hardware whichever card
    is asked does the same thing.

    It is modelled here as a write-only window, which is the part that
    matters: the read side of port 362 is SW1 on whichever configuration or
    CPU card owns it, and taking that away would leave XCON8 with every
    switch off and nothing to say.  A machine with two of these cards in it
    therefore has one card's latch driving the line and the other's sitting
    idle, where the real bus would let either do it - a difference no
    software can see, since both ends drive the same line to the same place.

    WHAT IS NOT MODELLED
    --------------------
    The 20 mA current loop the serial channels can be strapped for, and the
    DCE/DTE connector pairs - each channel here is one RS-232 port rather
    than the two sockets the card brings out for it.

****************************************************************************/

#include "emu.h"

#include "wh_8_47.h"

#include "bus/heathzenith/h47/h47_intf.h"
#include "bus/rs232/rs232.h"
#include "machine/ins8250.h"

namespace {

class wh_8_47_device : public heath_h47_intf_device
					 , public device_h8bus_card_interface
{
public:

	wh_8_47_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void map_io(address_space_installer &space) override ATTR_COLD;

	virtual void set_interrupt(int state) override;

	void read_jumpers();
	void uart_int_w(int index, int state);
	void update_int(u8 level);
	void portf2_w(u8 data);

	required_device_array<ins8250_device, 2> m_uart;
	required_ioport_array<2>                 m_serial_jumpers;
	required_ioport                          m_disk_jumpers;

	// base address of the four port disk block, 0 with DISK ENABLE out
	u8  m_disk_base;

	// base address each serial channel answers at, 0 with CHAN ENABLE off
	u8  m_serial_base[2];

	// bus interrupt level each source is wired to, 0 when no wire is fitted
	u8  m_disk_level;
	u8  m_serial_level[2];

	int m_disk_intr;
	int m_serial_intr[2];

	u8  m_gpp;
};


wh_8_47_device::wh_8_47_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_h47_intf_device(mconfig, H8BUS_WH_8_47, tag, owner, 0)
	, device_h8bus_card_interface(mconfig, *this)
	, m_uart(*this, "uart%u", 0U)
	, m_serial_jumpers(*this, "JUMPERS%u", 0U)
	, m_disk_jumpers(*this, "DISK")
{
}

void wh_8_47_device::read_jumpers()
{
	// in the order the settings are listed below; the spare slots keep a
	// hand-edited cfg file from running off the end
	static constexpr u8 DISK_BASE[4]   = { 0x00, 0x78, 0x7c, 0x00 };
	static constexpr u8 SERIAL_BASE[8] = { 0x00, 0xc0, 0xc8, 0xd0, 0xd8, 0xe0, 0xe8, 0x00 };

	ioport_value const disk(m_disk_jumpers->read());

	m_disk_base  = DISK_BASE[disk & 0x03];
	m_disk_level = (disk >> 2) & 0x07;

	for (int i = 0; i < 2; i++)
	{
		ioport_value const jumpers(m_serial_jumpers[i]->read());

		m_serial_base[i]  = SERIAL_BASE[jumpers & 0x07];
		m_serial_level[i] = (jumpers >> 3) & 0x07;
	}
}

void wh_8_47_device::set_interrupt(int state)
{
	m_disk_intr = state;

	update_int(m_disk_level);
}

void wh_8_47_device::uart_int_w(int index, int state)
{
	m_serial_intr[index] = state;

	update_int(m_serial_level[index]);
}

void wh_8_47_device::update_int(u8 level)
{
	// no wire fitted to this source's interrupt holes
	if (level == 0)
	{
		return;
	}

	// Sources jumpered to the same level share one bus line, so all three
	// have to be looked at to decide whether it is still being held.
	int state = 0;

	if (m_disk_base && (m_disk_level == level))
	{
		state |= m_disk_intr;
	}

	for (int i = 0; i < 2; i++)
	{
		if (m_serial_base[i] && (m_serial_level[i] == level))
		{
			state |= m_serial_intr[i];
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

// "The Monitor ROM is reenabled by either outputting a D5 logic 0 to I/O port
// 362 or by a system RESET."
void wh_8_47_device::portf2_w(u8 data)
{
	if (BIT(data ^ m_gpp, 5))
	{
		set_slot_rom_disable(BIT(data, 5));
	}

	m_gpp = data;
}

void wh_8_47_device::device_start()
{
	heath_h47_intf_device::device_start();

	m_disk_base  = 0;
	m_disk_level = 0;
	m_disk_intr  = 0;
	m_gpp        = 0;

	for (int i = 0; i < 2; i++)
	{
		m_serial_base[i]  = 0;
		m_serial_level[i] = 0;
		m_serial_intr[i]  = 0;
	}

	save_item(NAME(m_disk_base));
	save_item(NAME(m_serial_base));
	save_item(NAME(m_disk_level));
	save_item(NAME(m_serial_level));
	save_item(NAME(m_disk_intr));
	save_item(NAME(m_serial_intr));
	save_item(NAME(m_gpp));
}

void wh_8_47_device::device_reset()
{
	// The jumpers are read in map_io, which the CPU card runs from its own
	// reset - not here, because nothing orders the two resets and reading them
	// twice would leave map_io unable to see that an address had moved.
	m_disk_intr = 0;
	m_gpp       = 0;

	for (int i = 0; i < 2; i++)
	{
		m_serial_intr[i] = 0;
	}

	for (u8 level = 3; level <= 7; level++)
	{
		update_int(level);
	}

	heath_h47_intf_device::device_reset();
}

void wh_8_47_device::map_io(address_space_installer &space)
{
	// The bus maps its cards from the CPU card's reset, so this runs again on
	// every reset. Re-reading the jumpers here is what lets an address moved
	// in the machine configuration menu take effect, and a window that has
	// moved is given up - only ever a window this card installed itself.
	u8 const previous_disk(m_disk_base);
	u8 const previous_serial[2] = { m_serial_base[0], m_serial_base[1] };

	read_jumpers();

	// Everything that is going comes out before anything new goes in, so that
	// two channels swapping addresses cannot have the second one's unmap take
	// the first one's freshly installed window with it.
	if (previous_disk && (previous_disk != m_disk_base))
	{
		space.unmap_readwrite(previous_disk, previous_disk + 3);
	}

	for (int i = 0; i < 2; i++)
	{
		if (previous_serial[i] && (previous_serial[i] != m_serial_base[i]))
		{
			space.unmap_readwrite(previous_serial[i], previous_serial[i] + 7);
		}
	}

	if (m_disk_base)
	{
		space.install_readwrite_handler(m_disk_base, m_disk_base + 3,
			read8sm_delegate(*this, FUNC(wh_8_47_device::read)),
			write8sm_delegate(*this, FUNC(wh_8_47_device::write))
		);
	}

	for (int i = 0; i < 2; i++)
	{
		if (m_serial_base[i])
		{
			space.install_readwrite_handler(m_serial_base[i], m_serial_base[i] + 7,
				read8sm_delegate(m_uart[i], FUNC(ins8250_device::ins8250_r)),
				write8sm_delegate(m_uart[i], FUNC(ins8250_device::ins8250_w))
			);
		}
	}

	// write only, so that whichever card owns SW1 at this port keeps it
	space.install_write_handler(0xf2, 0xf2,
		write8smo_delegate(*this, FUNC(wh_8_47_device::portf2_w))
	);
}

void wh_8_47_device::device_add_mconfig(machine_config &config)
{
	heath_h47_intf_device::device_add_mconfig(config);

	static char const *const RS232_TAG[2] = { "rs232_0", "rs232_1" };

	for (int i = 0; i < 2; i++)
	{
		// "The 1.843 MHz crystal oscillator serves as the clock for the baud
		// rate generators of the ACE's."
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

static INPUT_PORTS_START( wh_8_47_jumpers )

	PORT_START("DISK")
	PORT_CONFNAME(0x03, 0x01, "Disk address")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "170-173 octal")
	PORT_CONFSETTING(   0x02, "174-177 octal")
	PORT_CONFNAME(0x1c, 0x14, "Disk interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x0c, "Level 3")
	PORT_CONFSETTING(   0x10, "Level 4")
	PORT_CONFSETTING(   0x14, "Level 5")
	PORT_CONFSETTING(   0x18, "Level 6")
	PORT_CONFSETTING(   0x1c, "Level 7")

	// "Disk I/O status, switch SW101 (not presently used)" - four sections
	// of a DIP that the disk system reads back in the status word and
	// nothing has ever been seen to look at.
	PORT_START("SW101")
	PORT_DIPNAME( 0x01, 0x00, "SW101-A" )   PORT_DIPLOCATION("SW101:1")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, "SW101-B" )   PORT_DIPLOCATION("SW101:2")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, "SW101-C" )   PORT_DIPLOCATION("SW101:3")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "SW101-D" )   PORT_DIPLOCATION("SW101:4")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )

	PORT_START("JUMPERS0")
	PORT_CONFNAME(0x07, 0x00, "Channel 0 address")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "300-307 octal")
	PORT_CONFSETTING(   0x02, "310-317 octal")
	PORT_CONFSETTING(   0x03, "320-327 octal")
	PORT_CONFSETTING(   0x04, "330-337 octal")
	PORT_CONFSETTING(   0x05, "340-347 octal")
	PORT_CONFSETTING(   0x06, "350-357 octal")
	PORT_CONFNAME(0x38, 0x00, "Channel 0 interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x18, "Level 3")
	PORT_CONFSETTING(   0x20, "Level 4")
	PORT_CONFSETTING(   0x28, "Level 5")
	PORT_CONFSETTING(   0x30, "Level 6")
	PORT_CONFSETTING(   0x38, "Level 7")

	PORT_START("JUMPERS1")
	PORT_CONFNAME(0x07, 0x00, "Channel 1 address")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "300-307 octal")
	PORT_CONFSETTING(   0x02, "310-317 octal")
	PORT_CONFSETTING(   0x03, "320-327 octal")
	PORT_CONFSETTING(   0x04, "330-337 octal")
	PORT_CONFSETTING(   0x05, "340-347 octal")
	PORT_CONFSETTING(   0x06, "350-357 octal")
	PORT_CONFNAME(0x38, 0x00, "Channel 1 interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x18, "Level 3")
	PORT_CONFSETTING(   0x20, "Level 4")
	PORT_CONFSETTING(   0x28, "Level 5")
	PORT_CONFSETTING(   0x30, "Level 6")
	PORT_CONFSETTING(   0x38, "Level 7")

INPUT_PORTS_END

ioport_constructor wh_8_47_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(wh_8_47_jumpers);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H8BUS_WH_8_47, device_h8bus_card_interface, wh_8_47_device, "h8_wh_8_47", "Heath H47 Interface and Serial I/O Card (WH8-47)");
