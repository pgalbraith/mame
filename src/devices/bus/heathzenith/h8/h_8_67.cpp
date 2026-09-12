// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath H8 to Z-67 Winchester Interface

    The H8 end of the Z-67, the counterpart of the H-89's Z-89-67.  What it
    reaches, and every register it presents, is in
    bus/heathzenith/h67/h67.cpp - this file is where the card answers and
    where its interrupt goes, and nothing else.

    WHAT IT IS CALLED
    -----------------
    "H8-67", which is how Heath's own port allocation table for the H8 names
    it - HDOS 3.02's manual, Table 1-2, lists `8" Winchester HardDsk` against
    `H8-67` at `170-173Q or 174-177Q`, exactly as Table 1-1 lists `H89-67`
    for the card that is really the Z-89-67.  So the table is a shorthand
    rather than a part number, and no catalog entry, price or part number
    for an H8 Z-67 interface was found: the Z-67 was a Zenith Data Systems
    commercial product and never appeared on a Heathkit catalog hardware
    page at all.

    That it existed is not in doubt.  REMark issue 28 (1982), reporting from
    the West Coast Computer Faire, has Heath showing a controller board
    "that allows the H8, WITH Z80 Board installed, to operate with the H/Z-37
    Tandon drives and the Z-67 10 meg hard-disk", and the H8 monitor that
    board carries knows about it: PAM-37's switch offers "H67" as the device
    at either disk port, which is the setting this card is fitted to match.

    WHERE IT ANSWERS
    ----------------
    The same two four-port blocks every Heath disk controller uses, chosen by
    a jumper: 170-173 octal or 174-177.  Only the first two ports of the
    block are the card's; see h67.cpp for what they are.

    THE INTERRUPT
    -------------
    The card has an interrupt enable bit and the H8 bus carries INT3 to INT7,
    but nothing says which line the board was strapped to and nothing drives
    it - the CP/M BIOS runs the whole transaction with interrupts disabled,
    as does MTR-90 on the H-89.  So no wire is fitted by default.

****************************************************************************/

#include "emu.h"

#include "h_8_67.h"

#include "bus/heathzenith/h67/h67.h"

namespace {

class h_8_67_device : public heath_h67_intf_device
					, public device_h8bus_card_interface
{
public:

	h_8_67_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void map_io(address_space_installer &space) override ATTR_COLD;

	virtual void set_interrupt(int state) override;

	required_ioport m_jumpers;

	// base address of the four port block, 0 with the address jumper out
	u8  m_base;

	// bus interrupt level the card is wired to, 0 when no wire is fitted
	u8  m_level;

	int m_intr;
};


h_8_67_device::h_8_67_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_h67_intf_device(mconfig, H8BUS_H_8_67, tag, owner, 0)
	, device_h8bus_card_interface(mconfig, *this)
	, m_jumpers(*this, "JUMPERS")
{
}

void h_8_67_device::set_interrupt(int state)
{
	m_intr = state;

	switch (m_level)
	{
		case 3: set_slot_int3(state); break;
		case 4: set_slot_int4(state); break;
		case 5: set_slot_int5(state); break;
		case 6: set_slot_int6(state); break;
		case 7: set_slot_int7(state); break;
	}
}

void h_8_67_device::device_start()
{
	heath_h67_intf_device::device_start();

	m_base  = 0;
	m_level = 0;
	m_intr  = 0;

	save_item(NAME(m_base));
	save_item(NAME(m_level));
	save_item(NAME(m_intr));
}

void h_8_67_device::device_reset()
{
	// The jumpers are read in map_io, which the CPU card runs from its own
	// reset - not here, because nothing orders the two resets and reading them
	// twice would leave map_io unable to see that the address had moved.
	m_intr = 0;

	for (u8 level = 3; level <= 7; level++)
	{
		if (m_level == level)
		{
			set_interrupt(0);
		}
	}

	heath_h67_intf_device::device_reset();
}

void h_8_67_device::map_io(address_space_installer &space)
{
	// in the order the settings are listed below; the spare slot keeps a
	// hand-edited cfg file from running off the end
	static constexpr u8 BASE[4] = { 0x00, 0x78, 0x7c, 0x00 };

	u8 const previous(m_base);

	ioport_value const jumpers(m_jumpers->read());

	m_base  = BASE[jumpers & 0x03];
	m_level = (jumpers >> 2) & 0x07;

	if (previous && (previous != m_base))
	{
		space.unmap_readwrite(previous, previous + 3);
	}

	if (m_base)
	{
		space.install_readwrite_handler(m_base, m_base + 3,
			read8sm_delegate(*this, FUNC(h_8_67_device::read)),
			write8sm_delegate(*this, FUNC(h_8_67_device::write))
		);
	}
}

static INPUT_PORTS_START( h_8_67_jumpers )

	PORT_START("JUMPERS")
	PORT_CONFNAME(0x03, 0x01, "Disk address")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "170-173 octal")
	PORT_CONFSETTING(   0x02, "174-177 octal")
	PORT_CONFNAME(0x1c, 0x00, "Disk interrupt")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x0c, "Level 3")
	PORT_CONFSETTING(   0x10, "Level 4")
	PORT_CONFSETTING(   0x14, "Level 5")
	PORT_CONFSETTING(   0x18, "Level 6")
	PORT_CONFSETTING(   0x1c, "Level 7")

INPUT_PORTS_END

ioport_constructor h_8_67_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(h_8_67_jumpers);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H8BUS_H_8_67, device_h8bus_card_interface, h_8_67_device, "h8_h_8_67", "Heath H8 to Z-67 Winchester Interface");
