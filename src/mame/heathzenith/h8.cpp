// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

    Heathkit H8

    This system uses Octal and Split-Octal rather than the usual hexadecimal.

    STATUS:
        It runs, keyboard works, you can enter data.
        Serial console works. You can make it visible by setting Video
        Options in settings.

    Meaning of LEDs:
        PWR = Power is turned on (+5V is present at on front panel)
        MON = The front panel is being serviced by the cpu (controls should work)
        RUN = CPU is running (not halted)
        ION = Interrupts are enabled

    Pasting:
        H8    | mame key
    -----------------------
        0-F   | as is
        +     |   ^
        -     |   V
        MEM   |   -
        ALTER |   =

        Addresses must have all 6 digits entered. Data must have all 3 digits entered.
        System has a short beep for each key, and a slightly longer beep for each
        group of 3 digits. The largest number allowed is octal 377 (=256/0xFF).

    Test Paste:
        -041000=123 245 333 144 255 366 077=-041000
        Now press up-arrow to confirm the data has been entered.

    Official test program from pages 4 to 8 of the operator's manual:
        -040100=076 002 062 010 040 006 004 041 170 040 021 013 040 016 011 176
                022 043 023 015 302 117 040 016 003 076 377 315 053 000 015 302
                131 040 005 302 112 040 076 062 315 140 002 076 062 315 053 000
                076 062 315 140 002 303 105 040 377 262 270 272 275 377 222 200
                377 237 244 377 272 230 377 220 326 302 377 275 272 271 271 373
                271 240 377 236 376 362 236 376 362 236 376 362 R6=040100=4

****************************************************************************/

#include "emu.h"

#include "bus/heathzenith/h8/cards.h"
#include "bus/heathzenith/h8/h8bus.h"

namespace {

class h8_state : public driver_device
{
public:
	h8_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_h8bus(*this, "h8bus")
		, m_p1(*this, "p1")
		, m_p2(*this, "p2")
		, m_p10(*this, "p10")
		{}

	void h8(machine_config &config);
	void h8_h17(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void device_config_complete() override  ATTR_COLD;

private:
	required_device<h8bus_device> m_h8bus;
	required_device<h8bus_slot_device> m_p1;
	required_device<h8bus_slot_device> m_p2;
	required_device<h8bus_slot_device> m_p10;
};

// Input ports
static INPUT_PORTS_START( h8 )
INPUT_PORTS_END

// This machine comes with the disk system already in it, so have the HA-8-8
// boot from it rather than stopping in the monitor: XCON8 takes its commands
// from the front panel and prints nothing at all while it waits, so SW1:8 at
// Normal leaves a blank terminal that reads as a hung machine.
static DEVICE_INPUT_DEFAULTS_START( h8_h17_ha_8_8 )
	DEVICE_INPUT_DEFAULTS( "SW1", 0x80, 0x80 )
DEVICE_INPUT_DEFAULTS_END

// Put jumper X1-X2 back to the open position Heath shipped, so the monitor ROM
// stays at 0000-0FFF and cannot be switched out from the bus.  The CPU card
// closes it by default to give CP/M the ORG-0 it needs, but that trade goes the
// wrong way on a hard-sectored machine: XCON8 disables the ROM on its way to a
// boot, and every HDOS 1.x release then counts the RAM that appears underneath
// and quits with "?01 HDOS REQUIRES AT LEAST 12K!" - 50.00.00, 50.03.00 and
// 50.04.00 size 62K, 50.05.00 sizes 61K.  Only HDOS 2.0 knows to start above
// the ROM window and boots either way.  Open, all five size the usual 56K.
// Close it again to run something that wants RAM under the ROM.
static DEVICE_INPUT_DEFAULTS_START( h8_h17_cpu_8080 )
	DEVICE_INPUT_DEFAULTS( "CONFIG", 0x04, 0x00 )
DEVICE_INPUT_DEFAULTS_END

void h8_state::machine_start()
{
}

void h8_state::device_config_complete()
{
	// Connect up the p201 cable between p1 and p2 boards. This is separate from the h8bus.
	auto p1_lookup = m_p1.lookup()->get_card_device();
	auto p2_lookup = m_p2.lookup()->get_card_device();

	// avoid crash when there isn't a card installed in either slot.
	if (p1_lookup && p2_lookup)
	{
		device_p201_p1_card_interface *p1 = dynamic_cast<device_p201_p1_card_interface *>(p1_lookup);
		device_p201_p2_card_interface *p2 = dynamic_cast<device_p201_p2_card_interface *>(p2_lookup);

		p1->p201_reset_cb().set(*p2, FUNC(device_p201_p2_card_interface::p201_reset_w));
		p1->p201_int1_cb().set(*p2, FUNC(device_p201_p2_card_interface::p201_int1_w));
		p1->p201_int2_cb().set(*p2, FUNC(device_p201_p2_card_interface::p201_int2_w));

		p2->p201_inte_cb().set(*p1, FUNC(device_p201_p1_card_interface::p201_inte_w));
	}
}

void h8_state::h8(machine_config &config)
{
	H8BUS(config, m_h8bus);

	H8BUS_SLOT(config,  m_p1, "h8bus", h8_p1_cards,  "fp");
	H8BUS_SLOT(config,  m_p2, "h8bus", h8_p2_cards,  "cpu8080");
	H8BUS_SLOT(config,  "p3", "h8bus", h8_cards,     "wh_8_64");
	H8BUS_SLOT(config,  "p4", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  "p5", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  "p6", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  "p7", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  "p8", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  "p9", "h8bus", h8_cards,     "h_8_5");
	H8BUS_SLOT(config, m_p10, "h8bus", h8_p10_cards, "ha_8_8");
}

void h8_state::h8_h17(machine_config &config)
{
	h8(config);

	// The H-17 operation manual has the controller going near the back of the
	// bus, and P8 is the last free slot before the serial card in P9.
	H8BUS_SLOT(config.replace(), "p8", "h8bus", h8_cards, "h_8_17");

	// Standard PAM-8 knows nothing about disks - the H-17 operation manual has
	// you key 030 000 into the PC and press GO to reach the controller ROM - so
	// a disk machine wants one of the two monitors that boot on their own.
	// XCON8 is the Heath part (444-70, and it says so in its own image); the
	// other, PAM-8/AT, is not a dump at all but a reconstruction assembled from
	// a 27-Oct-81 listing, whose text records that its original part number is
	// unknown.  Prefer the genuine one.
	m_p2->set_option_default_bios("cpu8080", "xcon8");

	// ...and set that monitor's boot mode switch to Auto, so it boots the H-17
	// on its own rather than waiting at a blank screen.
	m_p10->set_option_device_input_defaults("ha_8_8", DEVICE_INPUT_DEFAULTS_NAME(h8_h17_ha_8_8));

	m_p2->set_option_device_input_defaults("cpu8080", DEVICE_INPUT_DEFAULTS_NAME(h8_h17_cpu_8080));
}

// ROM definition
ROM_START( h8 )
ROM_END

#define rom_h8_h17 rom_h8

} // anonymous namespace

// Driver

//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT  CLASS,    INIT        COMPANY          FULLNAME                              FLAGS
COMP( 1977, h8,     0,      0,      h8,      h8,    h8_state, empty_init, "Heath Company", "H-8",                                MACHINE_SUPPORTS_SAVE )
COMP( 1978, h8_h17, h8,     0,      h8_h17,  h8,    h8_state, empty_init, "Heath Company", "H-8 with H-17 Disk and H-19 Monitor", MACHINE_SUPPORTS_SAVE )
