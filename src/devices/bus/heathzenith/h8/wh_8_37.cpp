// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath WH-8-37 Double-Density Disk Controller and Z-67 Interface

    Two controllers on one H8 card: the Z-37 soft-sectored floppy controller,
    for up to four H-17-1 or H-17-4 drives, and the host end of the Z-67
    Winchester.  Each half has a jumper to turn it off.  The card needs the
    HA-8-6 Z80 CPU board, and Heath's catalog #860 (1983) has it new at $395
    assembled.  The quotations, jumper names and part numbers here are from
    its manual, 595-2859, and the schematic that comes with it
    [https://sebhc.github.io/sebhc/documentation/hardware/H8/H8-37_Op_Sc.zip].

    Software: HDOS 2.0 with the HOS-5-UP update drives the floppy half only;
    CP/M 2.2.03 and later drive both.  PAM-37, the HA-8-6 monitor that comes
    with the card, boots from either.

    Heath's own port allocation table for the H8 calls the Z-67 half "H8-67"
    - HDOS 3.02's manual, Table 1-2 - and REMark issue 28 (1982) reports Heath
    showing a board "that allows the H8, WITH Z80 Board installed, to operate
    with the H/Z-37 Tandon drives and the Z-67 10 meg hard-disk".  Both are
    this card.

    THE HA-8-6 HAS TO BE CHANGED - set its intr_socket to h37
    ----------------------------------------------------------
    The floppy half runs its transfers through the CPU board's interrupt
    logic, the same way the Z-89-37 does on the H89.  The manual's
    installation steps take the 74LS148 priority encoder out of U38 on the
    HA-8-6 and plug a 16-conductor cable from this card into the empty socket;
    the encoder's job moves onto this card, into U37, U43 and the 444-82 PAL
    U42.  With the controller's DRQ enabled, that logic "blocks all
    interrupts to the processor (except its own)", answers a DRQ with an EI
    so that the monitor's HALT loop reads the next byte, and raises level 4
    when a command ends - "INT 4 is used by the Z-37 when running".  PAM-37's
    own read loop shows the HALT trick: it copies HALT, IN A,(173Q), LD
    (DE),A, INC DE, JP (HL) into RAM and runs it there.

    The HA-8-6's interrupt socket models U38, and its "h37" option is that
    logic.  Without it the card still answers, but the first floppy transfer
    hangs at the HALT.  The 444-82 has not been dumped; the H89's Z-37
    interrupt controller stands in for it.

    The same steps move the HA-8-6's 444-70 ROM to U19 and put PAM-37
    (444-140) in U13; ha_8_6.cpp has the detail.

    WHERE IT ANSWERS
    ----------------
    U14 decodes one block of eight ports, and two jumpers pick its base:
    "port base address 170 octal (normal)" or 270.  The 444-117 PAL U26
    splits the block between the two halves using A2 and the two enable
    jumpers.  The manual gives the result rather than the equations: with
    both halves in use "the Z-37 device must be assigned to port 170 and the
    Z-67 device assigned to port 174", and with only one, "either may be
    assigned to port 170".  So here the floppy half takes the first four
    ports of the block and the Z-67 the second four, and a half on its own
    takes the first four.  The 444-117 has not been dumped either, so
    whether a lone half also answers in the second four is not known.

    Each half only decodes A0 and A1 within its four ports.  The Z-67 half
    uses two of them; h67.cpp has the registers.

    The Z-67 half ships switched off here: its four ports would land on 174,
    where an H8 usually has its H-17.  Turn the Z-67 jumper on, and take the
    H-17 out, together.

    THE Z-67 INTERRUPT
    ------------------
    "Position one jumper in the vertical position ... This is the normal
    OFF (no interrupts) position", or in one of three positions for INT 3, 4
    or 5.  The manual itself warns that 3 is the console and 4 the floppy
    half, and CP/M runs the Z-67 with interrupts off.

    WHAT IS NOT MODELLED
    --------------------
    The PARITY and PARITY ERROR jumpers - the model has no parity line to
    check - and the Z-67 RESET pulse width.  PRE COMP is a write
    precompensation jumper, and MAME's floppy layer has nothing for it to
    change.  The 8-section DIP switch DS1 is read through the 444-117's RD
    SWITCH strobe, at a port only the undumped PAL knows, and "they are not
    currently used by Heath or Zenith Data Systems software".

****************************************************************************/

#include "emu.h"

#include "wh_8_37.h"

#include "bus/heathzenith/h8x_common/h67.h"
#include "bus/heathzenith/h8x_common/z37_fdc_base.h"
#include "bus/heathzenith/intr_cntrl/intr_cntrl.h"

#include "softlist_dev.h"

#define LOG_SETUP (1U << 1)

#define VERBOSE (0)

#include "logmacro.h"

#define LOGSETUP(...)  LOGMASKED(LOG_SETUP, __VA_ARGS__)


// ======================> wh_8_37_z67_device
//
//  The Z-67 half of the card.  The card decides where its ports go and where
//  its interrupt goes, so this only passes the interrupt up.

class wh_8_37_z67_device : public heath_h67_intf_device
{
public:

	wh_8_37_z67_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto int_cb() { return m_int_cb.bind(); }

protected:

	virtual void set_interrupt(int state) override { m_int_cb(state); }

private:

	devcb_write_line m_int_cb;
};

DECLARE_DEVICE_TYPE(H8BUS_WH_8_37_Z67, wh_8_37_z67_device)

wh_8_37_z67_device::wh_8_37_z67_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_h67_intf_device(mconfig, H8BUS_WH_8_37_Z67, tag, owner, clock)
	, m_int_cb(*this)
{
}

DEFINE_DEVICE_TYPE(H8BUS_WH_8_37_Z67, wh_8_37_z67_device, "h8_wh_8_37_z67", "Heath WH-8-37 Z-67 interface")


namespace {

class wh_8_37_device : public heath_z37_fdc_base_device
					 , public device_h8bus_card_interface
{
public:

	wh_8_37_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void map_io(address_space_installer &space) override ATTR_COLD;

	virtual void set_irq_out(int state) override;
	virtual void set_drq_out(int state) override;
	virtual void block_interrupts_out(int state) override;

	void z67_interrupt(int state);

	required_device<wh_8_37_z67_device> m_z67;
	required_ioport                     m_jumpers;

	// the CPU board's interrupt socket, U38 on the HA-8-6
	heath_intr_socket *m_intr_socket;

	// first port of each half's four, 0 while that half is not answering
	u8 m_z37_port;
	u8 m_z67_port;

	// bus interrupt level the Z-67 half is jumpered to, 0 for none
	u8 m_z67_level;
};


wh_8_37_device::wh_8_37_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_z37_fdc_base_device(mconfig, H8BUS_WH_8_37, tag, owner, 0)
	, device_h8bus_card_interface(mconfig, *this)
	, m_z67(*this, "z67")
	, m_jumpers(*this, "JUMPERS")
	, m_intr_socket(nullptr)
{
}

void wh_8_37_device::set_irq_out(int state)
{
	if (m_intr_socket)
	{
		m_intr_socket->set_irq(state);
	}
}

void wh_8_37_device::set_drq_out(int state)
{
	if (m_intr_socket)
	{
		m_intr_socket->set_drq(state);
	}
}

void wh_8_37_device::block_interrupts_out(int state)
{
	if (m_intr_socket)
	{
		m_intr_socket->block_interrupts(state);
	}
}

void wh_8_37_device::z67_interrupt(int state)
{
	switch (m_z67_level)
	{
		case 3: set_slot_int3(state); break;
		case 4: set_slot_int4(state); break;
		case 5: set_slot_int5(state); break;
	}
}

void wh_8_37_device::device_start()
{
	heath_z37_fdc_base_device::device_start();

	m_z37_port  = 0;
	m_z67_port  = 0;
	m_z67_level = 0;

	save_item(NAME(m_z37_port));
	save_item(NAME(m_z67_port));
	save_item(NAME(m_z67_level));
}

void wh_8_37_device::device_reset()
{
	// Every card has registered with the bus by now, so the CPU board can be
	// found.  An 8080 board has no socket to find, and the floppy half's
	// interrupts then go nowhere - the real card needs the HA-8-6 too.
	m_intr_socket = h8bus().cpu_intr_socket();

	if (!m_intr_socket)
	{
		logerror("no CPU board interrupt socket found; the WH-8-37 needs the HA-8-6\n");
	}
	else if (!dynamic_cast<z37_intr_cntrl *>(m_intr_socket->get_card_device()))
	{
		logerror("%s is not set to h37; floppy transfers will hang\n", m_intr_socket->tag());
	}

	// The jumpers are read in map_io, which the CPU card runs from its own
	// reset - not here, because nothing orders the two resets.
	set_slot_int3(0);
	set_slot_int4(0);
	set_slot_int5(0);

	heath_z37_fdc_base_device::device_reset();
}

void wh_8_37_device::map_io(address_space_installer &space)
{
	ioport_value const jumpers(m_jumpers->read());

	u8 const base(BIT(jumpers, 0) ? 0xb8 : 0x78);
	bool const z37_on(BIT(jumpers, 1));
	bool const z67_on(BIT(jumpers, 2));

	u8 const z37_port(z37_on ? base : 0);
	u8 const z67_port(z67_on ? (z37_on ? base + 4 : base) : 0);

	m_z67_level = (jumpers >> 3) & 0x07;

	// Unmap only what moved: another card's map_io may already have put its
	// ports where this card used to be.
	if (m_z37_port && (m_z37_port != z37_port))
	{
		space.unmap_readwrite(m_z37_port, m_z37_port + 3);
	}
	if (m_z67_port && (m_z67_port != z67_port))
	{
		space.unmap_readwrite(m_z67_port, m_z67_port + 3);
	}

	m_z37_port = z37_port;
	m_z67_port = z67_port;

	if (m_z37_port)
	{
		LOGSETUP("Z-37 at %03o\n", m_z37_port);

		space.install_readwrite_handler(m_z37_port, m_z37_port + 3,
			read8sm_delegate(*this, FUNC(wh_8_37_device::read)),
			write8sm_delegate(*this, FUNC(wh_8_37_device::write)));
	}

	if (m_z67_port)
	{
		LOGSETUP("Z-67 at %03o\n", m_z67_port);

		space.install_readwrite_handler(m_z67_port, m_z67_port + 3,
			read8sm_delegate(*m_z67, FUNC(wh_8_37_z67_device::read)),
			write8sm_delegate(*m_z67, FUNC(wh_8_37_z67_device::write)));
	}
}

void wh_8_37_device::device_add_mconfig(machine_config &config)
{
	heath_z37_fdc_base_device::device_add_mconfig(config);

	H8BUS_WH_8_37_Z67(config, m_z67);
	m_z67->int_cb().set(FUNC(wh_8_37_device::z67_interrupt));

	// the same disks as the H89's Z-89-37
	SOFTWARE_LIST(config, "flop_list").set_original("h37_flop");
}

static INPUT_PORTS_START( wh_8_37_jumpers )

	PORT_START("JUMPERS")
	PORT_CONFNAME(0x01, 0x00, "Port base address")
	PORT_CONFSETTING(   0x00, "170 octal")
	PORT_CONFSETTING(   0x01, "270 octal")
	PORT_CONFNAME(0x02, 0x02, "Z-37 jumper")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x02, DEF_STR( On ))
	PORT_CONFNAME(0x04, 0x00, "Z-67 jumper")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x04, DEF_STR( On ))
	PORT_CONFNAME(0x38, 0x00, "Z-67 INT jumper")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x18, "INT 3")
	PORT_CONFSETTING(   0x20, "INT 4")
	PORT_CONFSETTING(   0x28, "INT 5")

INPUT_PORTS_END

ioport_constructor wh_8_37_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(wh_8_37_jumpers);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H8BUS_WH_8_37, device_h8bus_card_interface, wh_8_37_device, "h8_wh_8_37", "Heath WH-8-37 Double-Density Disk Controller and Z-67 Interface");
