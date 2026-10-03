// license:BSD-3-Clause
// copyright-holders:Mark Garlanger, Paul Galbraith
/***************************************************************************

  Heathkit Z-37 Floppy controller

    The registers and drives of the Z-37 controller, shared by the H89's
    Z-89-37 and the H8's WH-8-37.  See z37_fdc_base.h.

****************************************************************************/

#include "emu.h"

#include "z37_fdc_base.h"

#define LOG_REG (1U << 1)    // Shows register setup
#define LOG_LINES (1U << 2)  // Show control lines
#define LOG_DRIVE (1U << 3)  // Show drive select
#define LOG_FUNC (1U << 4)   // Function calls
#define LOG_ERR (1U << 5)    // log errors
#define LOG_SETUP (1U << 6)  // setup

#define VERBOSE (0)

#include "logmacro.h"

#define LOGREG(...)        LOGMASKED(LOG_REG, __VA_ARGS__)
#define LOGLINES(...)      LOGMASKED(LOG_LINES, __VA_ARGS__)
#define LOGDRIVE(...)      LOGMASKED(LOG_DRIVE, __VA_ARGS__)
#define LOGFUNC(...)       LOGMASKED(LOG_FUNC, __VA_ARGS__)
#define LOGERR(...)        LOGMASKED(LOG_ERR, __VA_ARGS__)
#define LOGSETUP(...)      LOGMASKED(LOG_SETUP, __VA_ARGS__)

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif


heath_z37_fdc_base_device::heath_z37_fdc_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, 0)
	, m_fdc(*this, "z37_fdc")
	, m_floppies(*this, "z37_fdc:%u", 0U)
{
}

void heath_z37_fdc_base_device::ctrl_w(u8 val)
{
	bool motor_on = bool(BIT(val, ctrl_MotorsOn_c));

	m_irq_allowed = bool(BIT(val, ctrl_EnableIntReq_c));
	m_drq_allowed = bool(BIT(val, ctrl_EnableDrqInt_c));
	m_fdc->dden_w(BIT(~val, ctrl_SetMFMRecording_c));

	LOGREG("%s: motor on: %d, intrq allowed: %d, drq allowed: %d\n",
		FUNCNAME, motor_on, m_irq_allowed, m_drq_allowed);

	if (m_drq_allowed)
	{
		block_interrupts_out(1);
	}
	else
	{
		block_interrupts_out(0);
		set_drq_out(0);
	}

	if (BIT(val, ctrl_Drive_0_c))
	{
		m_fdc->set_floppy(m_floppies[0]->get_device());
		LOGDRIVE("Drive selected: 0\n");
	}
	else if (BIT(val, ctrl_Drive_1_c))
	{
		m_fdc->set_floppy(m_floppies[1]->get_device());
		LOGDRIVE("Drive selected: 1\n");
	}
	else if (BIT(val, ctrl_Drive_2_c))
	{
		m_fdc->set_floppy(m_floppies[2]->get_device());
		LOGDRIVE("Drive selected: 2\n");
	}
	else if (BIT(val, ctrl_Drive_3_c))
	{
		m_fdc->set_floppy(m_floppies[3]->get_device());
		LOGDRIVE("Drive selected: 3\n");
	}
	else
	{
		m_fdc->set_floppy(nullptr);
		LOGDRIVE("Drive selected: none\n");
	}

	for (auto &elem : m_floppies)
	{
		floppy_image_device *floppy = elem->get_device();
		if (floppy)
		{
			floppy->mon_w(!motor_on);
		}
	}
}

void heath_z37_fdc_base_device::intf_w(u8 val)
{
	m_access_track_sector = bool(BIT(val, if_SelectSectorTrack_c));

	LOGREG("access track/sector: %d\n", m_access_track_sector);
}

void heath_z37_fdc_base_device::cmd_w(u8 val)
{
	m_access_track_sector ? m_fdc->sector_w(val) : m_fdc->cmd_w(val);
}

u8 heath_z37_fdc_base_device::stat_r()
{
	return m_access_track_sector ? m_fdc->sector_r() : m_fdc->status_r();
}

void heath_z37_fdc_base_device::data_w(u8 val)
{
	m_access_track_sector ? m_fdc->track_w(val) : m_fdc->data_w(val);
}

u8 heath_z37_fdc_base_device::data_r()
{
	return m_access_track_sector ? m_fdc->track_r() : m_fdc->data_r();
}

void heath_z37_fdc_base_device::write(offs_t offset, u8 data)
{
	LOGFUNC("%s: reg: %d val: 0x%02x\n", FUNCNAME, offset, data);

	switch (offset)
	{
		case 0:
			ctrl_w(data);
			break;
		case 1:
			intf_w(data);
			break;
		case 2:
			cmd_w(data);
			break;
		case 3:
			data_w(data);
			break;
	}
}

u8 heath_z37_fdc_base_device::read(offs_t offset)
{
	// default return for the h89
	u8 value = 0xff;

	switch (offset)
	{
		case 0:
		case 1:
			// read not supported on these addresses
			break;
		case 2:
			value = stat_r();
			break;
		case 3:
			value = data_r();
			break;
	}

	LOGFUNC("%s: reg: %d val: 0x%02x\n", FUNCNAME, offset, value);

	return value;
}

void heath_z37_fdc_base_device::device_start()
{
	save_item(NAME(m_irq_allowed));
	save_item(NAME(m_drq_allowed));
	save_item(NAME(m_access_track_sector));
}

void heath_z37_fdc_base_device::device_reset()
{
	m_irq_allowed         = false;
	m_drq_allowed         = false;
	m_access_track_sector = false;

	set_irq_out(0);
	set_drq_out(0);
	block_interrupts_out(0);
}

static void z37_floppies(device_slot_interface &device)
{
	// H-17-1 -- SS 48tpi
	device.option_add("ssdd", FLOPPY_525_SSDD);
	// SS 96tpi
	device.option_add("ssqd", FLOPPY_525_SSQD);
	// DS 48tpi
	device.option_add("dd",   FLOPPY_525_DD);
	// H-17-4 / H-17-5 -- DS 96tpi
	device.option_add("qd",   FLOPPY_525_QD);
}

void heath_z37_fdc_base_device::device_add_mconfig(machine_config &config)
{
	FD1797(config, m_fdc, 16_MHz_XTAL / 16);
	m_fdc->intrq_wr_callback().set(FUNC(heath_z37_fdc_base_device::set_irq));
	m_fdc->drq_wr_callback().set(FUNC(heath_z37_fdc_base_device::set_drq));
	// Z-89-37 schematics show the ready line tied high.
	m_fdc->set_force_ready(true);

	// H-17-4, the 96 tpi double-sided drive: 640K apiece from 80 tracks on both
	// sides.  The Z-37 cabinet held two of them, "two drives for 1.28
	// megabytes".  The 48 tpi H-17-1 is the other drive this controller took -
	// the 1982 Heath catalog puts one of 160K inside the Z-90-82, a single side
	// at 48 tpi in the controller's double density format, sixteen 256 byte
	// sectors over 40 tracks, with the Z-87 expansion cabinet adding two more
	// of the same.  The Z-89-37 operation manual (595-2674-04) takes either and
	// marks every Z-89-37 row of its drive chart "either", so both are the
	// controller's own.
	//
	// Defaulting to the double-sided drive matters because nothing in this path
	// can sense head count: the FD1797 has no such input, the control register
	// has no side bit, and 5.25" drives have no two-sided sense line.  Heath's
	// FORMAT therefore offers a one- or two-side choice on any drive and
	// completes either way, so on a single-sided drive a two-sided format
	// silently lays both sides onto the one surface.
	//
	// Nothing catches that afterwards either, and the part is what makes it so.
	// Table 4 of the FD1797-02 data sheet scopes the side compare flags to the
	// 1791/3 - C at bit 1 to enable the compare, S at bit 3 to say which side -
	// and gives the 1795/7 different flags in those same positions: S at bit 1
	// is the side select that updates SSO (pin 25), b at bit 3 is the sector
	// length flag.  A 1797 has no way to be asked to check the side byte in an
	// ID field, so the fictional second side reads back clean rather than
	// raising Record Not Found.  Beware the data sheet's prose here, which
	// describes the C/S compare without naming a part and mentions an
	// "internal side compare" on the 1795/7; only Table 4 carries the
	// "1791/3 only" and "1795/7 only" qualifiers.
	//
	// What this card will not take is a mixture: write precompensation is
	// jumpered at J3 for the whole card, so one setting or the other is wrong
	// for some of the drives and "can result in reduced data reliability".
	// Change all three together, never some of them.
	//
	// Do expect 48 tpi media to come up read-only in these 96 tpi drives, and
	// everything on the software list below is 48 tpi.  That is the driver's
	// own doing rather than a fault: at log-in it seeks to track 2 and reads a
	// cylinder back out of a sector header, and being told 1 - the half-pitch
	// signature of a 48 tpi disk under a 96 tpi head - it flags the disk write
	// protected.  (It probes for side 1 first, and gives up on the disk
	// entirely at any other cylinder.)  Note the write would physically take:
	// the narrow head lays a track inside the wider one and leaves the old
	// edges unerased, so the disk still reads here, just no longer dependably
	// in the 48 tpi drive it was written on.  Refusing is the conservative
	// call, and the manuals do not give the reason.  CONFIGUR says so plainly,
	// and CP/M reports the refusal as "Bdos Err On x: Bad Sector"; select ssdd
	// on all three to write the shipped 48 tpi disks.
	FLOPPY_CONNECTOR(config, m_floppies[0], z37_floppies, "qd", floppy_image_device::default_mfm_floppy_formats);
	m_floppies[0]->enable_sound(true);
	FLOPPY_CONNECTOR(config, m_floppies[1], z37_floppies, "qd", floppy_image_device::default_mfm_floppy_formats);
	m_floppies[1]->enable_sound(true);
	FLOPPY_CONNECTOR(config, m_floppies[2], z37_floppies, "qd", floppy_image_device::default_mfm_floppy_formats);
	m_floppies[2]->enable_sound(true);
	FLOPPY_CONNECTOR(config, m_floppies[3], z37_floppies, nullptr, floppy_image_device::default_mfm_floppy_formats);
	m_floppies[3]->enable_sound(true);
}

void heath_z37_fdc_base_device::set_irq(int state)
{
	LOGLINES("set irq, allowed: %d state: %d\n", m_irq_allowed, state);

	set_irq_out(m_irq_allowed ? state : CLEAR_LINE);
}

void heath_z37_fdc_base_device::set_drq(int state)
{
	LOGLINES("set drq, allowed: %d state: %d\n", m_drq_allowed, state);

	set_drq_out(m_drq_allowed ? state : CLEAR_LINE);
}
