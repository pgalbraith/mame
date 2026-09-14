// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-PMC PROM Memory Card

    Up to eight 1702 or 1702A PROMs of 256 bytes each, 2K in all, in any
    2K block of memory. Jumpers from pads I1-I5 to A11-A15 or their
    inverses set the block; A10-A8 then pick the PROM, IC A for the first
    256 bytes through IC H for the last, and A7-A0 the byte. The card only
    drives the bus on a memory read of its own block. Empty sockets read
    as FF here.

    MITS supplied its Disk Boot Loader, DBL, on a PROM for this card, to
    answer at 177400 octal: IC H with the board at 174000, which is how the
    card comes by default. To boot a disk, set the address switches to
    177400, EXAMINE, then RUN. DBL copies itself into RAM at 026000 and runs
    there, because the PROM is too slow to run from.

    Not emulated
    - the 0 to 3 wait states, patched with J6 and J7, that slow PROMs need
    - the VGG switching that powers only the pair of PROMs being read

    References
    - MITS 88-PMC PROM Memory Card manual, 1976
      [https://deramp.com/downloads/altair/hardware/2K%20PROM%20Board%20(88-PMC).pdf]
    - DBL 4.1, disassembled by Martin Eberhard from an EPROM labelled
      'DBL 4.1' [https://deramp.com/downloads/altair/software/roms/orginal_roms/]

**********************************************************************/

#include "emu.h"
#include "mitspmc.h"


namespace {

class s100_mits_pmc_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_pmc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_smemr_r(offs_t offset) override;

private:
	// A15-A11 select the board, through a comparator with the jumpers
	bool board_selected(offs_t offset) { return (offset >> 11) == m_address->read(); }

	required_region_ptr<u8> m_proms;
	required_ioport m_address;
};

s100_mits_pmc_device::s100_mits_pmc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_PMC, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_proms(*this, "proms")
	, m_address(*this, "ADDRESS")
{
}

void s100_mits_pmc_device::device_start()
{
}

u8 s100_mits_pmc_device::s100_smemr_r(offs_t offset)
{
	return board_selected(offset) ? m_proms[offset & 0x7ff] : 0xff;
}

#define ADDRESS_JUMPER(pad, bit) \
	PORT_CONFNAME(1 << (bit - 11), 1 << (bit - 11), "Address jumper I" #pad) \
	PORT_CONFSETTING(0, "/A" #bit " (0)") \
	PORT_CONFSETTING(1 << (bit - 11), "A" #bit " (1)")

static INPUT_PORTS_START( mits_pmc )
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(5, 15)
	ADDRESS_JUMPER(4, 14)
	ADDRESS_JUMPER(3, 13)
	ADDRESS_JUMPER(2, 12)
	ADDRESS_JUMPER(1, 11)
INPUT_PORTS_END

ioport_constructor s100_mits_pmc_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_pmc);
}

ROM_START( mits_pmc )
	ROM_REGION( 0x800, "proms", ROMREGION_ERASEFF )
	// IC H, at 177400 with the board at 174000
	ROM_LOAD( "dbl 4.1.bin", 0x700, 0x100, CRC(8e658905) SHA1(def83ce7bbf16960a87228d6ce94b192a2012c97) )
ROM_END

const tiny_rom_entry *s100_mits_pmc_device::device_rom_region() const
{
	return ROM_NAME(mits_pmc);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_PMC, device_s100_card_interface, s100_mits_pmc_device, "s100_mits_pmc", "MITS 88-PMC 2K PROM Memory Card")
