// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    driver.cpp

    Core driver device base class.

***************************************************************************/

#include "emu.h"
#include "image.h"
#include "drivenum.h"
#include "emuopts.h"
#include "tilemap.h"

#include <sstream>


//**************************************************************************
//  DRIVER DEVICE
//**************************************************************************

//-------------------------------------------------
//  driver_device - constructor
//-------------------------------------------------

driver_device::driver_device(const machine_config &mconfig, device_type type, const char *tag)
	: device_t(mconfig, type, tag, nullptr, 0)
	, m_system(mconfig.gamedrv())
	, m_flip_screen_x(0)
	, m_flip_screen_y(0)
{
	// set the search path to include all parents and cache it because devices search system paths
	m_searchpath.emplace_back(m_system.name);
	std::set<game_driver const *> seen;
	for (int ancestor = driver_list::clone(m_system); 0 <= ancestor; ancestor = driver_list::clone(ancestor))
	{
		if (!seen.insert(&driver_list::driver(ancestor)).second)
			throw emu_fatalerror("driver_device(%s): parent/clone relationships form a loop", m_system.name);
		m_searchpath.emplace_back(driver_list::driver(ancestor).name);
	}
}


//-------------------------------------------------
//  driver_device - destructor
//-------------------------------------------------

driver_device::~driver_device()
{
}


//-------------------------------------------------
//  static_set_callback - set the a callback in
//  the device configuration
//-------------------------------------------------

void driver_device::static_set_callback(device_t &device, callback_type type, driver_callback_delegate callback)
{
	downcast<driver_device &>(device).m_callbacks[type] = std::move(callback);
}


//-------------------------------------------------
//  empty_init - default implementation which
//  calls driver init
//-------------------------------------------------

void driver_device::empty_init()
{
}


//-------------------------------------------------
//  searchpath - return cached search path
//-------------------------------------------------

std::vector<std::string> driver_device::searchpath() const
{
	return m_searchpath;
}


//-------------------------------------------------
//  boot_device - default implementation which
//  names no boot device
//-------------------------------------------------

device_t *driver_device::boot_device(ioport_settings const &settings) const
{
	return nullptr;
}


//-------------------------------------------------
//  find_boot_device - ask a system that is not
//  running what it would boot from
//-------------------------------------------------

device_t *driver_device::find_boot_device(machine_config &config, emu_options const &options)
{
	driver_device *const driver = dynamic_cast<driver_device *>(&config.root_device());
	if (!driver)
		return nullptr;

	// the same port list ioport_manager::initialize() would build, but with
	// the user's saved switch positions over the top of the declared ones
	ioport_list portlist;
	std::ostringstream errors;
	for (device_t &device : device_enumerator(config.root_device()))
		portlist.append(device, errors);

	ioport_settings settings(portlist);
	settings.apply_system_cfg(config.gamedrv(), options);

	return driver->boot_device(settings);
}


//-------------------------------------------------
//  driver_start - default implementation which
//  does nothing
//-------------------------------------------------

void driver_device::driver_start()
{
}


//-------------------------------------------------
//  machine_start - default implementation which
//  calls to the legacy machine_start function
//-------------------------------------------------

void driver_device::machine_start()
{
}


//-------------------------------------------------
//  sound_start - default implementation which
//  calls to the legacy sound_start function
//-------------------------------------------------

void driver_device::sound_start()
{
}


//-------------------------------------------------
//  video_start - default implementation which
//  calls to the legacy video_start function
//-------------------------------------------------

void driver_device::video_start()
{
}


//-------------------------------------------------
//  driver_reset - default implementation which
//  does nothing
//-------------------------------------------------

void driver_device::driver_reset()
{
}


//-------------------------------------------------
//  machine_reset - default implementation which
//  calls to the legacy machine_reset function
//-------------------------------------------------

void driver_device::machine_reset()
{
}


//-------------------------------------------------
//  sound_reset - default implementation which
//  calls to the legacy sound_reset function
//-------------------------------------------------

void driver_device::sound_reset()
{
}


//-------------------------------------------------
//  video_reset - default implementation which
//  calls to the legacy video_reset function
//-------------------------------------------------

void driver_device::video_reset()
{
}


//-------------------------------------------------
//  device_rom_region - return a pointer to the
//  game's ROMs
//-------------------------------------------------

const tiny_rom_entry *driver_device::device_rom_region() const
{
	return m_system.rom;
}


//-------------------------------------------------
//  device_add_mconfig - add machine configuration
//-------------------------------------------------

void driver_device::device_add_mconfig(machine_config &config)
{
	m_system.machine_creator(config, *this);
}


//-------------------------------------------------
//  device_input_ports - return a pointer to the
//  game's input ports
//-------------------------------------------------

ioport_constructor driver_device::device_input_ports() const
{
	return m_system.ipt;
}


//-------------------------------------------------
//  device_start - device override which calls
//  the various helpers
//-------------------------------------------------

void driver_device::device_start()
{
	// reschedule ourselves to be last
	for (device_t &test : device_enumerator(*this))
		if (&test != this && !test.started())
			throw device_missing_dependencies();

	// call the game-specific init
	m_system.driver_init(*this);

	// finish image devices init process
	machine().image().postdevice_init();

	// start the various pieces
	driver_start();

	if (!m_callbacks[CB_MACHINE_START].isnull())
		m_callbacks[CB_MACHINE_START]();
	else
		machine_start();

	sound_start();

	if (!m_callbacks[CB_VIDEO_START].isnull())
		m_callbacks[CB_VIDEO_START]();
	else
		video_start();

	// save generic states
	save_item(NAME(m_flip_screen_x));
	save_item(NAME(m_flip_screen_y));
}


//-------------------------------------------------
//  device_reset_after_children - device override
//  which calls the various helpers; must happen
//  after all child devices are reset
//-------------------------------------------------

void driver_device::device_reset_after_children()
{
	// reset each piece
	driver_reset();

	if (!m_callbacks[CB_MACHINE_RESET].isnull())
		m_callbacks[CB_MACHINE_RESET]();
	else
		machine_reset();

	sound_reset();

	video_reset();
}



//**************************************************************************
//  INTERRUPT GENERATION CALLBACK HELPERS
//**************************************************************************

//-------------------------------------------------
//  NMI callbacks
//-------------------------------------------------

INTERRUPT_GEN_MEMBER( driver_device::nmi_line_pulse )   { device.execute().pulse_input_line(INPUT_LINE_NMI, attotime::zero); }
INTERRUPT_GEN_MEMBER( driver_device::nmi_line_assert )  { device.execute().set_input_line(INPUT_LINE_NMI, ASSERT_LINE); }


//-------------------------------------------------
//  IRQn callbacks
//-------------------------------------------------

INTERRUPT_GEN_MEMBER( driver_device::irq0_line_hold )   { device.execute().set_input_line(0, HOLD_LINE); }
INTERRUPT_GEN_MEMBER( driver_device::irq0_line_assert ) { device.execute().set_input_line(0, ASSERT_LINE); }

INTERRUPT_GEN_MEMBER( driver_device::irq1_line_hold )   { device.execute().set_input_line(1, HOLD_LINE); }
INTERRUPT_GEN_MEMBER( driver_device::irq1_line_assert ) { device.execute().set_input_line(1, ASSERT_LINE); }

INTERRUPT_GEN_MEMBER( driver_device::irq2_line_hold )   { device.execute().set_input_line(2, HOLD_LINE); }
INTERRUPT_GEN_MEMBER( driver_device::irq2_line_assert ) { device.execute().set_input_line(2, ASSERT_LINE); }

INTERRUPT_GEN_MEMBER( driver_device::irq3_line_hold )   { device.execute().set_input_line(3, HOLD_LINE); }
INTERRUPT_GEN_MEMBER( driver_device::irq3_line_assert ) { device.execute().set_input_line(3, ASSERT_LINE); }

INTERRUPT_GEN_MEMBER( driver_device::irq4_line_hold )   { device.execute().set_input_line(4, HOLD_LINE); }
INTERRUPT_GEN_MEMBER( driver_device::irq4_line_assert ) { device.execute().set_input_line(4, ASSERT_LINE); }

INTERRUPT_GEN_MEMBER( driver_device::irq5_line_hold )   { device.execute().set_input_line(5, HOLD_LINE); }
INTERRUPT_GEN_MEMBER( driver_device::irq5_line_assert ) { device.execute().set_input_line(5, ASSERT_LINE); }

INTERRUPT_GEN_MEMBER( driver_device::irq6_line_hold )   { device.execute().set_input_line(6, HOLD_LINE); }
INTERRUPT_GEN_MEMBER( driver_device::irq6_line_assert ) { device.execute().set_input_line(6, ASSERT_LINE); }

INTERRUPT_GEN_MEMBER( driver_device::irq7_line_hold )   { device.execute().set_input_line(7, HOLD_LINE); }
INTERRUPT_GEN_MEMBER( driver_device::irq7_line_assert ) { device.execute().set_input_line(7, ASSERT_LINE); }


//**************************************************************************
//  GENERIC FLIP SCREEN HANDLING
//**************************************************************************

//-------------------------------------------------
//  updateflip - handle global flipping
//-------------------------------------------------

void driver_device::updateflip()
{
	// push the flip state to all tilemaps
	machine().tilemap().set_flip_all((TILEMAP_FLIPX & m_flip_screen_x) | (TILEMAP_FLIPY & m_flip_screen_y));
}


//-------------------------------------------------
//  flip_screen_set - set global flip
//-------------------------------------------------

void driver_device::flip_screen_set(int state)
{
	// normalize to all 1
	if (state)
		state = 0xff;

	// if something's changed, handle it
	if (m_flip_screen_x != state || m_flip_screen_y != state)
	{
		m_flip_screen_x = m_flip_screen_y = state;
		updateflip();
	}
}


//-------------------------------------------------
//  flip_screen_x_set - set global horizontal flip
//-------------------------------------------------

void driver_device::flip_screen_x_set(int state)
{
	// normalize to all 1
	if (state)
		state = 0xff;

	// if something's changed, handle it
	if (m_flip_screen_x != state)
	{
		m_flip_screen_x = state;
		updateflip();
	}
}


//-------------------------------------------------
//  flip_screen_y_set - set global vertical flip
//-------------------------------------------------

void driver_device::flip_screen_y_set(int state)
{
	// normalize to all 1
	if (state)
		state = 0xff;

	// if something's changed, handle it
	if (m_flip_screen_y != state)
	{
		m_flip_screen_y = state;
		updateflip();
	}
}
