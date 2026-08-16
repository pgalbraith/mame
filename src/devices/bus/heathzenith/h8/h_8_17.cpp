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

    RAM writes are gated by bit 7 of the control port.

    Per the schematic of the Model WH17 Floppy Disk (595-2195-01), the card
    takes only these H8 bus lines: D0-D7 (10-17), phi 2 (22), MWR (23), I/O WR
    (21), I/O RD (26), MRD (28), RESET (29) and A0-A15 (30-45). It drives no
    interrupt, and pin 46, /ROM DISABLE, goes nowhere - U21 (an LS42) decodes
    the address unconditionally, so the card always answers for 1400-1FFF.

    The XCON8 monitor (444-70) carries its own copy of this ROM image, so a
    machine fitted with it does not need U14 - but the card answers for those
    addresses either way.

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

	virtual void map_mem(address_space_installer & space) override ATTR_COLD;
	virtual void map_io(address_space_installer & space) override ATTR_COLD;

	virtual void set_ram_write_enable(int state) override;

	void ram_w(offs_t offset, u8 data);

	required_memory_region   m_rom;
	memory_share_creator<u8> m_ram;

	bool m_ram_write_enabled;
};

h_8_17_device::h_8_17_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_h17_fdc_base_device(mconfig, H8BUS_H_8_17, tag, owner, 0)
	, device_h8bus_card_interface(mconfig, *this)
	, m_rom(*this, "h17rom")
	, m_ram(*this, "ram", 0x400U, ENDIANNESS_LITTLE)
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
	SOFTWARE_LIST(config, "flop_list").set_compatible("h89_flop");
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
