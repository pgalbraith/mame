// license:BSD-3-Clause
// copyright-holders:Mark Garlanger, Paul Galbraith
/***************************************************************************

  Heathkit H-17 Floppy Disk Controller

    The H8 version of the card, model number H-8-17. It is the same
    controller as the H89's H-88-1, so it decodes the same four ports
    (0174-0177 octal, also listed as the "Port 174 device" on the HA-8-8's
    SW1) and runs the same 444-19 ROM.

    Unlike the H89 - where the ROM sits in the U520 socket on the CPU board
    and the floppy RAM is part of the CPU board's lower 8k - both live on
    the card here. The H8 CPU board only decodes its own ROM at 0000-0FFF
    and the memory boards start at 8k, so nothing else in an H8 answers for
    the addresses the boot ROM runs from and uses:

        1400-17FF   1k RAM  - U16/U17, a pair of 2114s (443-764)
        1800-1FFF   2k ROM  - U14, a 2316 mask ROM (444-19)

    which is how the H-17 Operation manual (595-2160-03) puts it on page 21,
    in split octal - that manual and the schematic below are both in
    [https://sebhc.github.io/sebhc/documentation/hardware/H8/H8-17_Op_Sc.zip]: "The first 8K bytes ... are allocated to non-user
    functions ... The controller circuit board RAM (U16 and U17) is assigned
    the 5K slot (024 000 through 027 377) and the controller ROM (U14) is
    assigned the 6K and 7K slots (030 000 through 037 377)."

    RAM writes are gated by bit 7 of the control port.

    Per the schematic of the Model WH17 Floppy Disk (595-2195-01), the card
    takes only these H8 bus lines: D0-D7 (10-17), phi 2 (22), MWR (23), I/O WR
    (21), I/O RD (26), MRD (28), RESET (29) and A0-A15 (30-45). It drives no
    interrupt, and pin 46, /ROM DISABLE, goes nowhere - U21 (an LS42) decodes
    the address unconditionally, so the card always answers for 1400-1FFF.

    The XCON8 monitor (444-70) carries its own copy of this ROM image, so a
    machine fitted with it does not need U14 - but an unmodified card answers
    for those addresses either way.

    THE HA-8-8 REWIRE - after it, this board answers for nothing at all
    ---------------------------------------------------------------------
    Which is a problem for CP/M, and Heath's answer was to have the owner
    rewire this board.  The HA-8-8 Extended Configuration Option's Assembly
    and Operation manual (595-2509-1, in
    [https://sebhc.github.io/sebhc/documentation/hardware/H8/H8-8_As_Op_Sc.zip]) carries the steps under "ROM DISABLE
    (ORG 0)", pages 24 through 27, and they run across three boards.  This one
    comes first, under Pictorials 2-3 (page 24) and 2-4 (page 25), both
    captioned CONTROLLER BOARD 85-2204-1:

      unplug the controller board;
      on an older board, one with no IC at U28, remove U22 and bend pins 2 and
        13 up so they no longer reach the socket, then reinstall it - the card
        modelled here has U28, so that step does not apply;
      cut or unsolder C15 (47 pF mica) and R6 (68 ohm, blu-gry-blk), "This
        resistor will no longer be used";
      solder a 3/4" bare wire to the foils where C15 was.

    R6 is in series from MRD to the node that
    drives U20B pin 5 (/ROM EN) and U20D pin 12 (/RAM RD), and C15 hangs from
    that node to ground, so the two steps together isolate the node from MRD
    and tie it low: both gates hold a low input for good, and U14, U16 and U17
    stop driving the bus.  The manual puts the result plainly - with ORG-0 "the
    PAM-8 or H17 ROM/RAM areas ... now exist in read/write RAM, rather than ROM
    and write-protected RAM", which is what CP/M needs, since its system image
    loads from 0300 upward straight through 1400.

    WHY IT IS A SETTING, AND THE TWO THAT GO WITH IT
    -------------------------------------------------
    The board is the same part either way, so this is a setting here rather
    than a second card, and it is the owner's to make: page 22
    says the Extended Configuration Board has three functions and "Decide which
    functions you wish to use.  Then make the necessary modifications for those
    functions."  Everything above belongs to just one of the three, so an
    HA-8-8 fitted for the other two sits beside an untouched H-17.

    Two more settings elsewhere belong to the same function and have to agree
    with this one, or the machine boots nothing:

      X1-X2 closed on the CPU board (a CONFIG on cpu8080).  Page 26, under
        Pictorial 2-5, cuts the jumper wires at T1-T2, R1-R2, S2-S3, P2-P3 and
        Z2-Z3, then adds bare wires P1-P2, Z1-Z2 and "Solder a 3/4" bare wire
        from hole X1 to hole X2".  Page 27 finishes that board by fitting the
        XCON8 ROM (444-70) at IC204 and bending pin 2 of IC207 out.

      The memory boards moved down, so that RAM starts at 0 rather than 8k
        (the address-block switches on the memory card).  Page 27 closes the
        section with "NOTE: Make sure to reconfigure your memory circuit
        boards so the memory starts at 0 k instead of 8 k" - and with this
        modification made and nothing addressed at 1400, there is a hole where
        the boot ROM keeps its scratch.

    Page 24 is also where the bus side is spelled out: the option "pulls the
    ROM disable line on the bus (pin 46) low when a write is performed at I/O
    address 362Q (0F2H) and it sets data bit D5 to a logic 1", which is the
    GPP bit ha_8_8.cpp drives.

    h8.cpp sets all three together on the machine that has an HA-8-8.

    Note the modification is deliberately not gated off the /ROM DISABLE signal
    itself.  The leads are cut, so a modified board stays quiet whether or not
    ORG-0 is currently asserted.  Only the read paths go - U22C still makes
    /RAM WR - but with nothing able to read U16/U17 back, the writes cannot be
    observed, so the whole mapping simply goes away.

    An older board, one with no IC at U28, also needs U22 lifted at pins 2 and
    13.  The card modelled here has U28, so that step does not apply.

****************************************************************************/

#include "emu.h"

#include "h_8_17.h"

#include "bus/heathzenith/h17/h17_fdc_base.h"

#include "softlist_dev.h"

namespace {

class h_8_17_device : public heath_h17_fdc_base_device, public device_h8bus_card_interface
{
public:

	h_8_17_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:

	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void map_mem(address_space_installer & space) override ATTR_COLD;
	virtual void map_io(address_space_installer & space) override ATTR_COLD;

	virtual void set_ram_write_enable(int state) override;

	void ram_w(offs_t offset, u8 data);

	required_memory_region   m_rom;
	memory_share_creator<u8> m_ram;
	required_ioport          m_config;

	bool m_ram_write_enabled;
};

h_8_17_device::h_8_17_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_h17_fdc_base_device(mconfig, H8BUS_H_8_17, tag, owner, 0)
	, device_h8bus_card_interface(mconfig, *this)
	, m_rom(*this, "h17rom")
	, m_ram(*this, "ram", 0x400U, ENDIANNESS_LITTLE)
	, m_config(*this, "CONFIG")
{
}

void h_8_17_device::set_ram_write_enable(int state)
{
	m_ram_write_enabled = bool(state);
}

void h_8_17_device::ram_w(offs_t offset, u8 data)
{
	// the RAM is write protected unless the control port says otherwise
	if (m_ram_write_enabled)
	{
		m_ram[offset & 0x3ff] = data;
	}
}

void h_8_17_device::map_mem(address_space_installer & space)
{
	// A board rewired for the HA-8-8 answers for nothing here - see the notes
	// at the top of this file - leaving 1400-1FFF to whatever memory board is
	// addressed from zero.  Something needs to be, or the boot ROM has nowhere
	// to keep its scratch.
	if (BIT(m_config->read(), 0))
	{
		return;
	}

	// reads come straight from the RAM, writes have to go through the handler
	// so they can be dropped while it is protected
	space.install_rom(0x1400, 0x17ff, m_ram);
	space.install_write_handler(0x1400, 0x17ff,
		write8sm_delegate(*this, FUNC(h_8_17_device::ram_w))
	);

	space.install_rom(0x1800, 0x1fff, m_rom->base());
}

void h_8_17_device::map_io(address_space_installer & space)
{
	space.install_readwrite_handler(0x7c, 0x7f,
		read8sm_delegate(*this, FUNC(h_8_17_device::read)),
		write8sm_delegate(*this, FUNC(h_8_17_device::write))
	);
}

void h_8_17_device::device_start()
{
	heath_h17_fdc_base_device::device_start();

	save_item(NAME(m_ram_write_enabled));
}

void h_8_17_device::device_reset()
{
	m_ram_write_enabled = false;

	heath_h17_fdc_base_device::device_reset();
}

void h_8_17_device::device_add_mconfig(machine_config &config)
{
	heath_h17_fdc_base_device::device_add_mconfig(config);

	// the same hard-sectored disks the H-88-1 reads
	SOFTWARE_LIST(config, "flop_list").set_compatible("h17_flop");
}

static INPUT_PORTS_START( h_8_17 )

	PORT_START("CONFIG")
	PORT_CONFNAME(0x01, 0x00, "HA-8-8 ROM disable modification (R6 and C15 removed)")
	PORT_CONFSETTING(   0x00, DEF_STR( No ))
	PORT_CONFSETTING(   0x01, DEF_STR( Yes ))

INPUT_PORTS_END

ioport_constructor h_8_17_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(h_8_17);
}

ROM_START( h_8_17 )

	ROM_REGION( 0x800, "h17rom", 0 )
	ROM_LOAD( "2316_444-19_h17.u14", 0x0000, 0x0800, CRC(26e80ae3) SHA1(0c0ee95d7cb1a760f924769e10c0db1678f2435c))

ROM_END

const tiny_rom_entry *h_8_17_device::device_rom_region() const
{
	return ROM_NAME(h_8_17);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H8BUS_H_8_17, device_h8bus_card_interface, h_8_17_device, "h8_h_8_17", "Heath H-17 Hard-sectored Controller (H-8-17)");
