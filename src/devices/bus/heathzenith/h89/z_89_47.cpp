// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath/Zenith Z-89-47 8-inch Floppy Disk Interface

    The H-89 end of the H-47 8" floppy disk system, the counterpart of the
    H8's WH8-47.  Heath sold the cabinet as the H-47 and Zenith as the Z-47,
    and the card that reaches it as the WH-88-47 - WH-89-47 on some lists -
    or the Z-89-47.  It was $195 assembled in the Christmas 1980 catalog,
    alongside the H-88-7 replacement ROM set the machine needs to boot from
    it.

    The disk half is described in bus/heathzenith/h47/h47_intf.cpp, and the
    cabinet it talks to in bus/heathzenith/h47/h47.cpp.  Everything the host
    can see is the same as on the H8 - status port, data port, the same
    status word and the same command set - which is what makes one model do
    both machines, and is how MTR-90 drives either with one piece of code.

    WHERE IT ANSWERS
    ----------------
    The monitors know two disk blocks, and SW501 says what is in each.  The
    Configuration Guide for the H-88, H-89, Z-89 and Z-90 (597-2571-02,
    page 6) gives both settings in the same words: "01 Port 174(7CH)/177Q
    (7FH) is H/Z-47 disk" and "01 Port 170(78H)/173Q(7BH) is H/Z-47 disk".

        170-173 octal   78-7B   Disk I/O #1, the decoder's /CASS line,
                                which reaches the P504 and P505 slots
        174-177 octal   7C-7F   Disk I/O #2, the decoder's /FLPY line,
                                which reaches P506 alone

    So a card in P506 answers at 174 and one in P504 or P505 at 170.  That
    is decided by the slot rather than by a jumper here, which is what the
    decoder PROM does on the real machine - it drives a different select
    line to each slot.  The guide says the same thing from the other side,
    on page 7: the board "may be installed in either the right or left
    positions in the right expansion area (P506/P512 or P504/P510).
    However, they must be jumpered differently, depending on which of these
    positions are actually used."

    THE PULL-UP AT P506
    -------------------
    The same page has the owner fit a pull-up resistor "between pins 1 and
    12 of P512" when the board goes in that slot.  P506 is where the H-17
    normally sits, and the H-17 is what write-enables the 1k of floppy RAM
    on the CPU board; a card that takes its place has to hold that line up
    on its own or the RAM stays write protected and HDOS will not boot.
    Heath sold exactly such a resistor with the Z-89-37 for the same reason,
    and MAME already models it as the we_pullup device.  So a card in P506
    asserts FMWE here.

    That the guide's resistor is that line is settled by what it says next,
    about the other board of the pair: the pull-up "is also included on
    Z-89-67, with a jumper connector to enable or disable it, depending on
    whether the Z-89-67 is installed at P506/P512 or P504/P510 (the pullup
    must be disabled if P504/P510 is used)".  A line that has to be held up
    in one slot and left alone in the other is the P506 signal, and FMWE is
    the P506 signal a disk card has any business driving.

    An H-89 fitted with this card at 174 has given up its H-17, since the
    hard-sectored controller is the other thing P506 is for.

    None of that reaches 170 or 174 on a machine still carrying its original
    I/O decode PROM.  The 444-43 part an H-88 or H-89 shipped with has
    nothing at all decoded below 200 octal and puts /CASS on the cassette
    block at 370-371 instead, so a card in P504 answers there and no software
    ever looks for it.  That is what the H-88-7 replacement ROM set is for:
    it brings the 444-61 decoder, whose map opens Disk I/O #1 and #2, and
    without it the catalog's requirement to fit that set alongside this card
    is not a formality.  Pick the decoder in the machine configuration menu
    or with -h89bus:io_decoder 444_61.

    WHAT IS INFERRED
    ----------------
    No manual or schematic for this card was to hand.  What is modelled is
    what the software requires and what the H8 card's manual documents about
    the shared half, so the ports, the status word, the master reset bit and
    the interrupt enable bit are on firm ground.  Two things are not:

      the interrupt level.  The H8 card patches its disk interrupt to any
      of INT3 to INT7 and its installation steps put it on 5; an H-89's
      right-hand slots carry INT3, INT4 and INT5, and the HDOS System
      Programmer's Guide (595-2553-01, page 8) lists level "5 - Reserved
      for H47 (if implemented)", so 5 is the default here and the other two
      are offered.  Nothing exercises it - HDOS's H47 driver, the CP/M BIOS
      and MTR-90 all poll the status port and leave the interrupt enable
      bit clear - so the choice costs nothing either way.

      SW101.  The H8 card reads four sections of a DIP back in the middle of
      the status word and the manual says they are "not presently used".
      They are offered here on the assumption that a card presenting the
      same status word presents the same bits.

****************************************************************************/

#include "emu.h"

#include "z_89_47.h"

#include "bus/heathzenith/h47/h47_intf.h"

#define LOG_SETUP (1U << 1)
#define LOG_ERR   (1U << 2)

#define VERBOSE (0)

#include "logmacro.h"

#define LOGSETUP(...)  LOGMASKED(LOG_SETUP, __VA_ARGS__)
#define LOGERR(...)    LOGMASKED(LOG_ERR, __VA_ARGS__)

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif

namespace {

class z_89_47_device : public heath_h47_intf_device
					 , public device_h89bus_right_card_interface
{
public:

	z_89_47_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void set_interrupt(int state) override;

	required_ioport m_config;

	bool m_installed;
	u8   m_level;
	int  m_intr;
};


z_89_47_device::z_89_47_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_h47_intf_device(mconfig, H89BUS_Z_89_47, tag, owner, 0)
	, device_h89bus_right_card_interface(mconfig, *this)
	, m_config(*this, "CONFIG")
{
}

void z_89_47_device::set_interrupt(int state)
{
	m_intr = state;

	switch (m_level)
	{
		case 3: set_slot_int3(state); break;
		case 4: set_slot_int4(state); break;
		case 5: set_slot_int5(state); break;
	}
}

void z_89_47_device::device_start()
{
	heath_h47_intf_device::device_start();

	m_installed = false;
	m_level     = 0;
	m_intr      = 0;

	save_item(NAME(m_installed));
	save_item(NAME(m_level));
	save_item(NAME(m_intr));
}

void z_89_47_device::device_reset()
{
	if (!m_installed)
	{
		// /FLPY only reaches P506, so a card in that slot has to ask for it
		// with its own P506 signalling; anywhere else the disk block is
		// /CASS.  Asking for the wrong one matches nothing, or - worse, for
		// a request with no select lines at all - every address in the PROM.
		u8 const select(m_p506_signals ? h89bus::IO_FLPY : h89bus::IO_CASS);

		h89bus::addr_ranges addr_ranges = h89bus().get_address_ranges(select, m_p506_signals);

		if (addr_ranges.size() == 1)
		{
			h89bus::addr_range range = addr_ranges.front();

			LOGSETUP("%s: addr: 0x%02x-0x%02x\n", FUNCNAME, range.first, range.second);

			h89bus().install_io_device(range.first, range.second,
				read8sm_delegate(*this, FUNC(z_89_47_device::read)),
				write8sm_delegate(*this, FUNC(z_89_47_device::write)));
		}
		else
		{
			LOGERR("%s: no address provided for device\n", FUNCNAME);
		}

		m_installed = true;
	}

	m_level = m_config->read() & 0x07;
	m_intr  = 0;

	if (m_p506_signals)
	{
		set_slot_fmwe(ASSERT_LINE);
	}

	set_slot_int3(0);
	set_slot_int4(0);
	set_slot_int5(0);

	heath_h47_intf_device::device_reset();
}

static INPUT_PORTS_START( z_89_47_device )

	PORT_START("CONFIG")
	PORT_CONFNAME(0x07, 0x05, "Disk interrupt level")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x03, "Level 3")
	PORT_CONFSETTING(   0x04, "Level 4")
	PORT_CONFSETTING(   0x05, "Level 5")

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

INPUT_PORTS_END

ioport_constructor z_89_47_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(z_89_47_device);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H89BUS_Z_89_47, device_h89bus_right_card_interface, z_89_47_device, "h89_z_89_47", "Heath/Zenith Z-89-47 8-inch Floppy Disk Interface");
