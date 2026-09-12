// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath/Zenith Z-89-67 Winchester Disk Interface

    The H-89 end of the Z-67, which is a Winchester and an 8" floppy in one
    cabinet behind a SASI controller.  The card itself is a host adapter and
    nothing else; what it reaches, and how, is in
    bus/heathzenith/h67/h67.cpp.

    WHERE IT ANSWERS
    ----------------
    The same two disk blocks as the Z-89-47, and by the same rule - the
    decoder PROM drives a different select line to each slot, so a card in
    P506 answers at 174 octal and one in P504 or P505 at 170:

        170-173 octal   78-7B   Disk I/O #1, the decoder's /CASS line
        174-177 octal   7C-7F   Disk I/O #2, the decoder's /FLPY line

    The Configuration Guide for the H-88, H-89, Z-89 and Z-90 (597-2571-02,
    page 7) names this board alongside the Z-89-47 for both: "Z-89-47 and
    Z-89-67 interface boards may be installed in either the right or left
    positions in the right expansion area (P506/P512 or P504/P510).
    However, they must be jumpered differently, depending on which of these
    positions are actually used."  It settles the order when both are
    fitted, too: "If both a Z-89-47 and a Z-89-67 board are used (together),
    the Z-89-67 should be installed at P506/P512."

    MTR-90 knows the card as the "Z-67" setting of either Disk I/O switch,
    and none of that reaches 170 or 174 without the 444-61 I/O decode PROM -
    the stock 444-43 decodes nothing below 200 octal.  That comes with the
    card rather than being something to go and find: the guide says MTR-90
    "is supplied with Z-89-37 and Z-89-67", and being a 4k part where
    MTR-88 and MTR-89 are 2k parts it brings the 444-83 secondary address
    decoder with it.  Pick the decoder in the machine configuration menu or
    with -h89bus:io_decoder 444_61.

    THE PULL-UP AT P506
    -------------------
    A card in P506 holds the floppy RAM write enable up, and here the guide
    is explicit rather than leaving it to be worked out: the pull-up "is
    also included on Z-89-67, with a jumper connector to enable or disable
    it, depending on whether the Z-89-67 is installed at P506/P512 or
    P504/P510 (the pullup must be disabled if P504/P510 is used)".  So the
    jumper follows the slot, and so does this.

    WHAT IS INFERRED
    ----------------
    The interrupt level.  The card has an interrupt enable bit and an H-89's
    right-hand slots carry INT3, INT4 and INT5, but nothing to hand says
    which one the board was strapped to, and nothing exercises it - MTR-90
    boots the Winchester with interrupts disabled throughout.  So the card
    starts with no wire fitted and the three levels are offered.

****************************************************************************/

#include "emu.h"

#include "z_89_67.h"

#include "bus/heathzenith/h67/h67.h"

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

class z_89_67_device : public heath_h67_intf_device
					 , public device_h89bus_right_card_interface
{
public:

	z_89_67_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

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


z_89_67_device::z_89_67_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_h67_intf_device(mconfig, H89BUS_Z_89_67, tag, owner, 0)
	, device_h89bus_right_card_interface(mconfig, *this)
	, m_config(*this, "CONFIG")
{
}

void z_89_67_device::set_interrupt(int state)
{
	m_intr = state;

	switch (m_level)
	{
		case 3: set_slot_int3(state); break;
		case 4: set_slot_int4(state); break;
		case 5: set_slot_int5(state); break;
	}
}

void z_89_67_device::device_start()
{
	heath_h67_intf_device::device_start();

	m_installed = false;
	m_level     = 0;
	m_intr      = 0;

	save_item(NAME(m_installed));
	save_item(NAME(m_level));
	save_item(NAME(m_intr));
}

void z_89_67_device::device_reset()
{
	if (!m_installed)
	{
		// /FLPY only reaches P506, so a card in that slot has to ask for it
		// with its own P506 signalling; anywhere else the disk block is
		// /CASS.
		u8 const select(m_p506_signals ? h89bus::IO_FLPY : h89bus::IO_CASS);

		h89bus::addr_ranges addr_ranges = h89bus().get_address_ranges(select, m_p506_signals);

		if (addr_ranges.size() == 1)
		{
			h89bus::addr_range range = addr_ranges.front();

			LOGSETUP("%s: addr: 0x%02x-0x%02x\n", FUNCNAME, range.first, range.second);

			h89bus().install_io_device(range.first, range.second,
				read8sm_delegate(*this, FUNC(z_89_67_device::read)),
				write8sm_delegate(*this, FUNC(z_89_67_device::write)));
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

	heath_h67_intf_device::device_reset();
}

static INPUT_PORTS_START( z_89_67_device )

	PORT_START("CONFIG")
	PORT_CONFNAME(0x07, 0x00, "Disk interrupt level")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x03, "Level 3")
	PORT_CONFSETTING(   0x04, "Level 4")
	PORT_CONFSETTING(   0x05, "Level 5")

INPUT_PORTS_END

ioport_constructor z_89_67_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(z_89_67_device);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H89BUS_Z_89_67, device_h89bus_right_card_interface, z_89_67_device, "h89_z_89_67", "Heath/Zenith Z-89-67 Winchester Disk Interface");
