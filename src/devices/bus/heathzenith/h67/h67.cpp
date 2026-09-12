// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath/Zenith Z-67 interface, the part that is the same on both buses

    The Z-67 is "an assembled external case with a 10 MB SASI Winchester
    drive and one 8" floppy disk drive", and the card that reaches it is the
    Z-89-67, "SASI Winchester disk controller".  The card is a plain host
    adapter - no processor, no buffer, no firmware - so unlike the H-47 there
    is nothing here to model at arm's length: the drives hang off a standard
    bus MAME already emulates, and this file is only the register window onto
    it.

    WHAT IS IN THE CABINET
    ----------------------
    A Shugart SA1400-family controller, on the evidence of the command set.
    MTR-90's equates name command 2 "Request Syndrome", which is the SA1400
    manual's name for it and is not a command a Xebec S1410 has at all - its
    list goes straight from Rezero to Request Sense.  The rest follows: that
    family's whole point was serving a Winchester and floppies off one board
    by logical unit number, which is exactly the pair a Z-67 holds, and the
    "10 MB" on the cabinet is the unformatted figure for an SA1004, whose
    256 cylinders and 4 heads come to rather less once the controller's fixed
    32 x 256 byte track shape is laid on it.

    That is inference from three fitting facts rather than a part list - no
    photograph or parts list of the inside of a Z-67 was to hand - so the
    other SASI controllers MAME models are offered alongside it.

    The drive was not a Shugart, though, and CP/M says so almost in as many
    words.  Its BIOS sizes the Winchester with a conditional-assembly flag
    called SHUGART: true gives 32768 sectors, which is the SA1000 family's
    256 cylinders by 4 heads by 32 sectors exactly, and false gives 39040,
    which is 305 by 4 by 32.  The shipped BIOS has it false, and one
    cylinder short of 306 by 4 is the
    ST-506 interface's 10 MB shape, the commonest rigid drive of 1981 and
    1982, a spare cylinder left over the top; "11 meg hard disk with 8"
    drive back-up" is how a 1982 dealer advertisement put it, counting
    unformatted.  Which maker's it was is not recorded anywhere to hand.

    So the drive type this uses is not one of the controller's own: it is
    the 306 by 4 by 32 one added to bus/nscsi/sa1403d.cpp for it, 10,027,008
    bytes of 256-byte sectors, which covers the 39040 CP/M asks for.  CP/M
    can still only put 8 MB in one partition - FORMAT says "PARTITION IS
    LARGER THAN CP/M MAXIMUM SIZE -- ONLY 8 MEG USEABLE" - which is why its
    BIOS offers two hard disk partitions and a floppy off the one cabinet.

    WHAT RAN ON IT
    --------------
    CP/M, and only CP/M.  The 2.2.03 BIOS carries an H67 driver, MOVCPM67
    and a backup utility to go with it, and gets three drives out of one
    cabinet - two Winchester partitions and the 8" floppy.  HDOS never did:
    the 1982 Christmas catalog's HDOS 2.0 entry says it "Supports all disk
    systems except Z-67", and HDOS 3.02's manual says plainly that "H67 was
    never supported by HDOS v.3.0, although a device driver was available
    from non-Heath vendors".  There is no H67 driver anywhere in the HDOS
    sources, which is why the only Heath code this was written against is
    CP/M's BIOS and MTR-90.

    Everything below comes from the "H67 Constants and Equates" block of
    MTR-90's source, which names every bit, and from the monitor's own boot
    path, which uses all of them.  The register layout:

        base+0  read and write   the SASI data bus
        base+1  write            control register
        base+1  read             bus status register

    CP/M's BIOS names a third, HD$SWI at base+2, "switch" - and then never
    reads it.  MTR-90 does not know about it at all.  Nothing says what it
    holds, so nothing is modelled for it and base+2 mirrors base+0, which is
    what decoding A0 alone gives.

    CONTROL REGISTER
        D6  BC_SEL   select, "and Data Bit 0"
        D5  BC_IE    interrupt enable
        D4  BC_RST   reset
        D1  BC_EDT   enable data

    BUS STATUS REGISTER
        D7  BS_REQ   data transfer request
        D6  BS_DTD   data transfer direction, set is "to Controller"
        D5  BS_LMB   last byte in command/data string
        D4  BS_MTY   message type, set is command rather than data
        D3  BS_BSY   busy

    The three low bits of the status register are where Heath's own two
    sources fall out with each other.  MTR-90 calls them interrupt pending,
    parity error and hardware identification; CP/M's BIOS calls the same
    three parity error, interrupt request and ACK - and they act on
    different ones, MTR-90 testing bit 1 for an "interface error" and the
    BIOS testing bit 2.  One of the two is mislabelled and no schematic was
    to hand to say which, so all three read back zero here, which is the
    no-error answer both drivers are looking for.

    Those five status bits are the five SASI lines a host adapter has to
    show: REQ, I/O, MSG, C/D and BSY.  BS_DTD is I/O the other way up -
    "to Controller" is the phase where I/O is not asserted - and BS_LMB is
    MSG, which on this bus only comes up for the single completion byte at
    the end of a command, hence the name.

    SELECTION IS TARGET 0, ALWAYS
    -----------------------------
    "Select and Data Bit 0" is one bit doing two jobs: asserting SEL also
    puts DB0 on the bus, which is how SASI names the controller it wants.
    So this card can only ever talk to controller 0, and everything else it
    reaches is addressed by the logical unit number inside the command.
    MTR-90 uses that: H67UnitSel shifts the boot unit into bits 6-5 of the
    command's second byte, and its comment says "if unit = 1, 8" floppy is
    selected", so the Winchester is LUN 0 and the cabinet's 8" floppy LUN 1.

    THE SEQUENCE THE MONITOR USES
    -----------------------------
    GetCon in MTR-90 is the whole of it, and is what this was tested
    against: wait for BSY to clear; write BC_SEL; wait for BSY to come up;
    write BC_EDT, which drops SEL again; then follow REQ, taking the phase
    from BS_MTY and BS_DTD, writing command bytes, reading data bytes, and
    finishing on the status byte and the completion message.

    WHAT IS INFERRED
    ----------------
    Two control bits have names and nothing else.  BC_EDT, "enable data",
    is taken to be the data buffers' enable; it is latched here and gates
    nothing, because a card that refused to hand over bytes until it was
    set would break any driver that does not bother, while one that always
    hands them over cannot break a driver that does.  ACK is instead
    pulsed only while BSY is asserted, which keeps the selection phase -
    where the card is driving DB0 itself - from acknowledging anything.

    BC_IE enables an interrupt whose source is not documented.  REQ is what
    it is taken to be here, that being the only thing a host adapter with
    no buffer has to interrupt about.  Nothing exercises it: MTR-90 runs
    the whole boot with interrupts disabled.

****************************************************************************/

#include "emu.h"

#include "h67.h"

#include "bus/nscsi/dtc510.h"
#include "bus/nscsi/hd.h"
#include "bus/nscsi/s1410.h"
#include "bus/nscsi/sa1403d.h"

#define LOG_REG  (1U << 1)   // port accesses
#define LOG_LINE (1U << 2)   // bus lines

#define VERBOSE (0)

#include "logmacro.h"

#define LOGREG(...)   LOGMASKED(LOG_REG, __VA_ARGS__)
#define LOGLINE(...)  LOGMASKED(LOG_LINE, __VA_ARGS__)


//**************************************************************************
//  heath_z67_controller_device
//**************************************************************************

// The drive-type switch on the controller is what declares a cabinet's
// drives, and the stock device is wired for the Xerox 820-II complement -
// floppies on LUN 0 to 2 - which is not a Z-67.  MTR-90 says what a Z-67
// holds instead: the Winchester answers as unit 0 and, where its boot code
// skips the head-exercising seek, "if unit = 1, 8" floppy is selected".  So
// LUN 0 is rigid and LUN 1 is an 8" floppy, and the two LUNs the cabinet does
// not use are left as empty floppy connectors rather than phantom disks.
//
// The Winchester is the 10 MB one CP/M's sizing implies rather than either of
// the two Shugart rigid types the controller's own switch offers, so it is
// set here rather than left to the switch.
heath_z67_controller_device::heath_z67_controller_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: nscsi_sa1403d_device(mconfig, HEATH_Z67_CONTROLLER, tag, owner, clock)
{
	set_drive_type(0, RIGID10);
	set_drive_type(1, SA850);
	set_drive_type(2, SA850);
	set_drive_type(3, SA850);
}


//**************************************************************************
//  heath_h67_host_device
//**************************************************************************

heath_h67_host_device::heath_h67_host_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, HEATH_H67_HOST, tag, owner, clock)
	, nscsi_device_interface(mconfig, *this)
	, m_req_cb(*this)
	, m_ack_timer(nullptr)
	, m_control(0)
	, m_req(false)
{
}

void heath_h67_host_device::device_start()
{
	m_ack_timer = timer_alloc(FUNC(heath_h67_host_device::ack_off), this);

	m_control = 0;
	m_req     = false;

	save_item(NAME(m_control));
	save_item(NAME(m_req));
}

void heath_h67_host_device::device_reset()
{
	m_scsi_bus->ctrl_w(m_scsi_refid, 0, nscsi_device_interface::S_ALL);
	m_scsi_bus->data_w(m_scsi_refid, 0);

	m_control = 0;
	m_req     = false;

	// ask the bus to call scsi_ctrl_changed for the lines the target drives
	constexpr u32 target_mask =
		nscsi_device_interface::S_BSY |
		nscsi_device_interface::S_REQ |
		nscsi_device_interface::S_MSG |
		nscsi_device_interface::S_CTL |
		nscsi_device_interface::S_INP;

	m_scsi_bus->ctrl_wait(m_scsi_refid, target_mask, target_mask);
}

void heath_h67_host_device::scsi_ctrl_changed()
{
	bool const req = bool(m_scsi_bus->ctrl_r() & S_REQ);

	if (req != m_req)
	{
		m_req = req;
		m_req_cb(req ? 1 : 0);
	}
}

TIMER_CALLBACK_MEMBER(heath_h67_host_device::ack_off)
{
	// Drop ACK and stop driving the data bus.  Letting go matters on the
	// phases the target drives: a byte this end left behind would be ORed
	// into whatever the controller puts up.
	m_scsi_bus->ctrl_w(m_scsi_refid, 0, S_ACK);
	m_scsi_bus->data_w(m_scsi_refid, 0);
}

u8 heath_h67_host_device::data_r()
{
	u8 const value = m_scsi_bus->data_r();

	// Every access to the data port carries its own ACK, and only while the
	// controller has the bus - during selection this end is driving DB0 and
	// there is nothing to acknowledge.
	if (!machine().side_effects_disabled() && (m_scsi_bus->ctrl_r() & S_BSY))
	{
		m_scsi_bus->ctrl_w(m_scsi_refid, S_ACK, S_ACK);
		m_ack_timer->adjust(ACK_PULSE);
	}

	return value;
}

void heath_h67_host_device::data_w(u8 data)
{
	m_scsi_bus->data_w(m_scsi_refid, data);

	if (m_scsi_bus->ctrl_r() & S_BSY)
	{
		m_scsi_bus->ctrl_w(m_scsi_refid, S_ACK, S_ACK);
		m_ack_timer->adjust(ACK_PULSE);
	}
}

u8 heath_h67_host_device::status_r()
{
	u32 const ctrl = m_scsi_bus->ctrl_r();

	u8 value = 0;

	if (ctrl & S_REQ)
	{
		value |= BS_REQ;
	}
	if (!(ctrl & S_INP))
	{
		// I/O is the line that says "to the initiator", so the card's
		// "to Controller" bit is its opposite
		value |= BS_DTD;
	}
	if (ctrl & S_MSG)
	{
		value |= BS_LMB;
	}
	if (ctrl & S_CTL)
	{
		value |= BS_MTY;
	}
	if (ctrl & S_BSY)
	{
		value |= BS_BSY;
	}

	return value;
}

void heath_h67_host_device::control_w(u8 data)
{
	LOGLINE("%s: control 0x%02x\n", machine().describe_context(), data);

	m_control = data;

	// "Select and Data Bit 0" - asserting SEL puts DB0 up as well, which is
	// SASI for "controller 0", and this card has no way to name another.
	m_scsi_bus->data_w(m_scsi_refid, (data & BC_SEL) ? 0x01 : 0x00);

	m_scsi_bus->ctrl_w(m_scsi_refid,
		((data & BC_SEL) ? S_SEL : 0) |
		((data & BC_RST) ? S_RST : 0),
		S_SEL | S_RST);
}


//**************************************************************************
//  heath_h67_intf_device
//**************************************************************************

heath_h67_intf_device::heath_h67_intf_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_sasi(*this, "sasi")
	, m_host(*this, "host")
	, m_int_enabled(false)
	, m_req(false)
{
}

// What can sit on the far end of the cable.  A SASI bus of this vintage
// carries a controller rather than a drive, and the list is kept to the ones
// a Winchester subsystem of 1981 would have been built around rather than
// MAME's whole SCSI catalogue - nothing with a CD-ROM in it was ever wired to
// an H-89.
static void z67_sasi_devices(device_slot_interface &device)
{
	device.option_add("z67",      HEATH_Z67_CONTROLLER);
	device.option_add("sa1403d",  NSCSI_SA1403D);
	device.option_add("s1410",    NSCSI_S1410);
	device.option_add("dtc510",   NSCSI_DTC510);
	device.option_add("harddisk", NSCSI_HARDDISK);
}

void heath_h67_intf_device::device_add_mconfig(machine_config &config)
{
	NSCSI_BUS(config, m_sasi);

	// The cabinet's controller is the only thing on the bus, and the card can
	// only select controller 0.
	NSCSI_CONNECTOR(config, "sasi:0", z67_sasi_devices, "z67");

	HEATH_H67_HOST(config, m_host);
	m_host->req_cb().set(FUNC(heath_h67_intf_device::req_w));

	// the host adapter takes the initiator's usual place at the top of the bus
	m_sasi->set_external_device(7, m_host);
}

void heath_h67_intf_device::device_start()
{
	m_int_enabled = false;
	m_req         = false;

	save_item(NAME(m_int_enabled));
	save_item(NAME(m_req));
}

void heath_h67_intf_device::device_reset()
{
	m_int_enabled = false;
	m_req         = false;

	set_interrupt(0);
}

void heath_h67_intf_device::req_w(int state)
{
	m_req = bool(state);

	update_interrupt();
}

void heath_h67_intf_device::update_interrupt()
{
	set_interrupt((m_int_enabled && m_req) ? 1 : 0);
}

u8 heath_h67_intf_device::read(offs_t offset)
{
	u8 const value = BIT(offset, 0) ? m_host->status_r() : m_host->data_r();

	LOGREG("%s: read %s -> 0x%02x\n", machine().describe_context(),
		BIT(offset, 0) ? "status" : "data", value);

	return value;
}

void heath_h67_intf_device::write(offs_t offset, u8 data)
{
	LOGREG("%s: write %s <- 0x%02x\n", machine().describe_context(),
		BIT(offset, 0) ? "control" : "data", data);

	if (BIT(offset, 0))
	{
		m_int_enabled = bool(data & heath_h67_host_device::BC_IE);

		m_host->control_w(data);

		update_interrupt();
	}
	else
	{
		m_host->data_w(data);
	}
}

DEFINE_DEVICE_TYPE(HEATH_H67_HOST, heath_h67_host_device, "heath_h67_host", "Heath/Zenith Z-67 SASI host adapter");
DEFINE_DEVICE_TYPE(HEATH_Z67_CONTROLLER, heath_z67_controller_device, "heath_z67_ctrl", "Heath/Zenith Z-67 disk controller");
