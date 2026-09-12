// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

    Heathkit H8

    This system uses Octal and Split-Octal rather than the usual hexadecimal.

    THREE MACHINES ACROSS THE H8's LIFE
    -----------------------------------
    The H8 was on sale from 1977 to 1983 and changed a great deal over that run,
    so rather than one pile of cards with a disk controller swapped in and out,
    each configuration here is a machine of its day.  Every card in them is
    dated from the Heathkit catalogs, which is where the notes below come from.

        h8        The cassette machine, and the one to reach for first: PAM-8,
                  16K on a WH-8-16, and an H-8-5 carrying both the tape
                  interface and the console with an H9 on it at 600 baud.

                  This is Heath's HKS-81 Hobbyist Computer System ($995,
                  catalogs #845 and #847) in all but the memory board: H8,
                  H-8-5 Serial I/O and Cassette Interface, H9 CRT, ECP-3801
                  Cassette Recorder and the H-8-18 cassette software.  HKS-81
                  pinned the memory at 8K - one H-8-1 filled out by the H-8-3
                  chip set - and 8K runs BASIC, HASL-8, TED-8 and BUG-8 but
                  not the Cassette Operating System or Extended Benton Harbor
                  BASIC, both of which want 16K (catalog #853).  Fitting the
                  16K board instead is the one deliberate departure here, so
                  that everything on the software list will run; put an
                  "h_8_1" in P3 from the slot menu for the package as sold.

                  The other order number in this territory is HFS-800 (#842,
                  1978, $379, WHS-800 assembled $475), and it was a combo offer
                  rather than a system: "Kit H8 plus H8-18 Audio Cassette
                  Software" and nothing else - no memory, no cassette
                  interface, no terminal, so none of the four programs on the
                  tape could be run on what the SKU contained.  The H8 kit
                  really did ship with no RAM at all; catalog #847's listing
                  says "Requires at least one H8-1 memory board to operate".

        h8_hks82  HKS-82 Advanced Computer System, $1995, #845 through #849.
                  H8, H-8-4 Four-Port Serial I/O, WH-8-16 16K Memory, H-17
                  Floppy Disk, CRT terminal, H-8-17 HDOS Software.  #845 lists
                  it with an H9 and that is what is fitted here; the H19 takes
                  over from #847.

                  Heath also sold this assembled and without a terminal, as
                  WHS-83 (#848 and #849, 1980, $1495 then $1595): "WH8
                  Assembled Computer, WH8-4 Assembled Four-Port Serial I/O,
                  WH8-16 16K Wired Memory Board, WH17 Assembled Floppy Disk
                  System and H8-17 HDOS Operating Software".  Same cards as
                  this machine, so it is not given one of its own - pick the
                  terminal off the slot menu, or none, to have it.

        h8_hks85  HKS-85 H-8 Hobbyist/Word Processing System, $1795, #857
                  through #860 (1982).  H8, WH-8-64 with 32K plus an HA-8-16
                  expansion set (48K), H-8-4, HA-8-8 Extended Configuration
                  Option, 100K-byte H-17, H-19 Video Terminal.

    The three packages show their terminal on a second screen - make it visible
    with Video Options in settings.

    STATUS:
        It runs, keyboard works, you can enter data.
        Serial console works.

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
#include "bus/heathzenith/h9/h9.h"
#include "bus/rs232/rs232.h"
#include "imagedev/floppy.h"

namespace {

class h8_state : public driver_device
{
public:
	h8_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_h8bus(*this, "h8bus")
		, m_p1(*this, "p1")
		, m_p2(*this, "p2")
		, m_p3(*this, "p3")
		, m_p8(*this, "p8")
		, m_p9(*this, "p9")
		, m_p10(*this, "p10")
		{}

	void h8(machine_config &config);
	void h8_hks82(machine_config &config);
	void h8_hks85(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void device_config_complete() override  ATTR_COLD;

private:
	required_device<h8bus_device> m_h8bus;
	required_device<h8bus_slot_device> m_p1;
	required_device<h8bus_slot_device> m_p2;
	required_device<h8bus_slot_device> m_p3;
	required_device<h8bus_slot_device> m_p8;
	required_device<h8bus_slot_device> m_p9;
	required_device<h8bus_slot_device> m_p10;

	static void h_8_5_with_h9(device_t *device);
	static void h_8_4_with_h9(device_t *device);
	static void h_8_4_with_h19(device_t *device);
	static void h_8_17_two_drives(device_t *device);
};

// Input ports
static INPUT_PORTS_START( h8 )
INPUT_PORTS_END

// The H9 is the terminal Heath sold with the H8 - it is two years older than
// the H19 - and the pairing has a documented wiring: 600 baud, EIA levels, 8
// bits, no parity, called the Heathkit Computer System Interface Standard (H9
// Operations, 595-2017-03, page 10).  Pictorial 2-2 there shows this very
// card, with SER RX and SER TX both jumpered to the 600 baud tap.
static DEVICE_INPUT_DEFAULTS_START( h8_console_600 )
	DEVICE_INPUT_DEFAULTS( "JUMPERS", 0x07, 0x03 )
DEVICE_INPUT_DEFAULTS_END

// The H-8-4 has no rate jumper - each port is an 8250 whose divisor the software
// sets - but its ports do have to be given an address, and all four ship set to
// None.  Put port 4 at 350-357 octal, which is where HDOS looks for a console on
// this card.  Port 4's interrupt is already Level 3 out of the box, which is
// what HDOS console input needs.
static DEVICE_INPUT_DEFAULTS_START( h8_h_8_4_console )
	DEVICE_INPUT_DEFAULTS( "JUMPERS4", 0x07, 0x04 )
DEVICE_INPUT_DEFAULTS_END

// 48K AT 0000-BFFF, BECAUSE THE HA-8-8 MOVES THE MEMORY DOWN
// -----------------------------------------------------------
// 48K is what the package was sold as: a WH-8-64 populated to
// 32K plus the HA-8-16 expansion set.  The board's CONFIG port carries one bit
// per 16K bank, so leaving bank 3 out gives the three that are fitted.
//
// Where they answer is a separate question, and the HA-8-8 settles it.  Its
// Assembly and Operation manual (595-2509-1, in
// [https://sebhc.github.io/sebhc/documentation/hardware/H8/H8-8_As_Op_Sc.zip])
// opens "ROM DISABLE (ORG 0)" on
// page 24 with "since the system RAM must be a continuous block originating at
// zero instead of 8k, your first RAM circuit board will be addressed at 000
// instead of 040", and closes the section on page 27 with "NOTE: Make sure to
// reconfigure your memory circuit boards so the memory starts at 0 k instead of
// 8 k".  So the three
// banks run 0000-BFFF rather than the 2000-DFFF a machine without the option
// would use: bank 0 at 0000, bank 1 at 4000, bank 2 at 8000, two 8K blocks
// each.  That is the layout the SW1-SW4 settings below describe.
//
// It is also what lets this machine run CP/M, which needs RAM at 0000 for its
// vectors, and it costs HDOS nothing - page 24 says so itself: "You can also
// use this function with Microsoft Basic under HDOS", and "NOTE: If you have
// less than 64k of memory, you can use HDOS Version 1.6.  For 64k of memory,
// you must use HDOS Version 2.0."
static DEVICE_INPUT_DEFAULTS_START( h8_wh_8_64_48k_from_zero )
	DEVICE_INPUT_DEFAULTS( "CONFIG", 0x08, 0x00 )   // bank 3 not populated
	// bank 0 at 0000-3FFF
	DEVICE_INPUT_DEFAULTS( "SW4",    0x01, 0x01 )
	DEVICE_INPUT_DEFAULTS( "SW4",    0x02, 0x02 )
	DEVICE_INPUT_DEFAULTS( "SW4",    0x04, 0x00 )
	// bank 1 at 4000-7FFF
	DEVICE_INPUT_DEFAULTS( "SW3",    0x04, 0x04 )
	DEVICE_INPUT_DEFAULTS( "SW3",    0x08, 0x08 )
	DEVICE_INPUT_DEFAULTS( "SW3",    0x10, 0x00 )
	// bank 2 at 8000-BFFF
	DEVICE_INPUT_DEFAULTS( "SW2",    0x10, 0x10 )
	DEVICE_INPUT_DEFAULTS( "SW2",    0x20, 0x20 )
	DEVICE_INPUT_DEFAULTS( "SW2",    0x40, 0x00 )
DEVICE_INPUT_DEFAULTS_END

void h8_state::h_8_5_with_h9(device_t *device)
{
	device->subdevice<rs232_port_device>("rs232")->set_default_option("h9");
}

// The H-8-4's four ports are plain RS-232 connectors named rs232_1..rs232_4, so
// the terminal goes on the same one the console address was jumpered to.
//
// It also has to be told 2400 baud.  There is no jumper for that on this card -
// the 8250's divisor is set by whatever software is running, and HDOS picks
// 2400 - so the rate has to come from the terminal's side, and the H9's own
// preset tap is the only end we can reach from here.
void h8_state::h_8_4_with_h9(device_t *device)
{
	rs232_port_device *const port = device->subdevice<rs232_port_device>("rs232_4");

	port->set_default_option("h9");

	// The rate has to be set from a second callback on the port rather than
	// here, because a slot's card does not exist yet at the point its default
	// option is chosen - machine_config::resolve_slot_devices creates the card
	// and only then runs the callback that would reach inside it.
	port->set_option_machine_config("h9", [] (device_t *terminal)
			{ terminal->subdevice<heath_h9_device>("h9")->set_preset_baud(2400); });
}

void h8_state::h_8_4_with_h19(device_t *device)
{
	device->subdevice<rs232_port_device>("rs232_4")->set_default_option("h19");
}

// Two drives, which is what the H-17 cabinet held.  A third only fits after the
// H-17-3 modification kit, which brought new sheet metal, a longer cable and a
// bigger fan with it, so leave that connector empty and let anyone who wants
// three pick one from the slot menu.
void h8_state::h_8_17_two_drives(device_t *device)
{
	device->subdevice<floppy_connector>("floppy2")->set_default_option(nullptr);
}

// This machine comes with the disk system already in it, so have the HA-8-8
// boot from it rather than stopping in the monitor: XCON8 takes its commands
// from the front panel and prints nothing at all while it waits, so SW1:8 at
// Normal leaves a blank terminal that reads as a hung machine.
static DEVICE_INPUT_DEFAULTS_START( h8_ha_8_8_auto )
	DEVICE_INPUT_DEFAULTS( "SW1", 0x80, 0x80 )
DEVICE_INPUT_DEFAULTS_END

// X1-X2 OPEN, SO THE MONITOR ROM CANNOT BE SWITCHED OUT
// -------------------------------------------------------
// Put the jumper back to the open position Heath shipped, so the monitor ROM
// stays at 0000-0FFF and cannot be switched out from the bus.  All three
// machines want this, for different reasons.
//
// On h8 and h8_hks82 it is simply what the hardware was: cpu8080.cpp closes the
// jumper by default so that an HA-8-8 can bank the ROM out without anyone
// having to change a setting first, but neither machine has an HA-8-8 -
// nothing on the bus can drive /ROM DISABLE at all - so a closed jumper
// describes a machine that never existed.
//
// On h8_hks85 the HA-8-8 is fitted and the jumper decides a real trade.  Closed
// gives CP/M the ORG-0 it needs, but that goes the wrong way on a hard-sectored
// machine: XCON8 disables the ROM on its way to a boot, and every HDOS 1.x
// release then counts the RAM that appears underneath and quits with "?01 HDOS
// REQUIRES AT LEAST 12K!" - 50.00.00, 50.03.00 and 50.04.00 size 62K, 50.05.00
// sizes 61K.  Only HDOS 2.0 knows to start above the ROM window and boots
// either way.  Open, all five size the usual 56K.  Close it again to run
// something that wants RAM under the ROM.
//
// Worth knowing what the wrong setting looks like from the outside, because it
// looks like nothing: a CP/M boot with the jumper open writes its JMPs to 0000
// and 0005, they land on ROM and evaporate, and the machine carries on into a
// warm-boot vector that is really monitor code.  No message, no halt, no
// output at all - the console is never reached to complain.
static DEVICE_INPUT_DEFAULTS_START( h8_cpu_8080 )
	DEVICE_INPUT_DEFAULTS( "CONFIG", 0x04, 0x00 )
DEVICE_INPUT_DEFAULTS_END

// Closed again on the machine that has an HA-8-8, because taking up that kit's
// ROM disable function is what closes it.  595-2509-1 page 26, Pictorial 2-5,
// cuts the jumper wires at T1-T2, R1-R2, S2-S3, P2-P3 and Z2-Z3, adds bare
// wires P1-P2 and Z1-Z2, and then: "Solder a 3/4" bare wire from hole X1 to
// hole X2".  Page 27 fits the XCON8 ROM (444-70) at IC204 on the same board,
// which is the other reason this machine runs that monitor.
// The disk controller's share of the same modification, 595-2509-1 pages 24 and
// 25, Pictorials 2-3 and 2-4: C15 and R6 come off the board and a bare wire goes
// across C15's foils, after which it stops answering for 1400-1FFF and the
// memory board above covers those addresses instead.  h_8_17.cpp has the detail
// and the gate-level reading.
static DEVICE_INPUT_DEFAULTS_START( h8_h_8_17_org0 )
	DEVICE_INPUT_DEFAULTS( "CONFIG", 0x01, 0x01 )
DEVICE_INPUT_DEFAULTS_END

static DEVICE_INPUT_DEFAULTS_START( h8_cpu_8080_ha_8_8 )
	DEVICE_INPUT_DEFAULTS( "CONFIG", 0x04, 0x04 )
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

	// 16K, which covers everything on the cassette software list - the Heath
	// Cassette Operating System and Extended Benton Harbor BASIC both want 16K
	// (catalog #853), while plain BASIC, HASL-8 and TED-8 want 8K and BUG-8
	// rather less.  The board's own switches already put it at 2000-5FFF.
	H8BUS_SLOT(config,  m_p3, "h8bus", h8_cards,     "wh_8_16");

	H8BUS_SLOT(config,  "p4", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  "p5", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  "p6", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  "p7", "h8bus", h8_cards,     nullptr);
	H8BUS_SLOT(config,  m_p8, "h8bus", h8_cards,     nullptr);

	// One H-8-5 doing both of the jobs a cassette H8 needs: the tape interface
	// and the console, with an H9 on it at 600 baud.
	H8BUS_SLOT(config,  m_p9, "h8bus", h8_cards,     "h_8_5");

	// No Extended Configuration Option on any of the three 1979-82 packages:
	// it is a late 1980 product, and only the HKS-85 is late enough to have it.
	H8BUS_SLOT(config, m_p10, "h8bus", h8_p10_cards, nullptr);

	m_p2->set_option_device_input_defaults("cpu8080", DEVICE_INPUT_DEFAULTS_NAME(h8_cpu_8080));

	m_p9->set_option_machine_config("h_8_5", h_8_5_with_h9);
	m_p9->set_option_device_input_defaults("h_8_5", DEVICE_INPUT_DEFAULTS_NAME(h8_console_600));
}

void h8_state::h8_hks82(machine_config &config)
{
	h8(config);

	// The H-17 operation manual has the controller going near the back of the
	// bus, and P8 is the last free slot before the serial card in P9.
	H8BUS_SLOT(config.replace(), m_p8, "h8bus", h8_cards, "h_8_17");
	m_p8->set_option_machine_config("h_8_17", h_8_17_two_drives);

	// This package has no H-8-5 at all - the four-port card is the only serial
	// on it, and the console lives on one of its ports.  That has a visible
	// consequence: the H-8-4's ports can only sit at 320, 330, 340 or 350
	// octal, never at the 372-373 the H-17 boot ROM prints to, so the ROM's
	// "ACTION? <BOOT>" dialogue goes nowhere and the screen stays dark until
	// HDOS loads and brings up its own console.
	H8BUS_SLOT(config.replace(), m_p9, "h8bus", h8_cards, "h_8_4");
	m_p9->set_option_machine_config("h_8_4", h_8_4_with_h9);
	m_p9->set_option_device_input_defaults("h_8_4", DEVICE_INPUT_DEFAULTS_NAME(h8_h_8_4_console));

	// Standard PAM-8 knows nothing about disks: the H-17 operation manual has
	// you key 030 000 into the PC and press GO to reach the controller ROM.
	// PAM-8/GO does the first part for you, presetting the program counter so a
	// boot is one key rather than an address.
	m_p2->set_option_default_bios("cpu8080", "pam8go");
}

void h8_state::h8_hks85(machine_config &config)
{
	h8_hks82(config);

	// The 64K board and the Extended Configuration Option arrive together and
	// belong together: the WH-8-64's switches map RAM across the whole of
	// 0000-FFFF, and the part under the monitor ROM is only reachable when
	// something can assert /ROM DISABLE.  Only these two cards can.  The
	// package ships the board half populated at 32K plus an HA-8-16 expansion
	// set, which is 48K, addressed from zero as the HA-8-8 manual instructs.
	H8BUS_SLOT(config.replace(),  m_p3, "h8bus", h8_cards,     "wh_8_64");
	m_p3->set_option_device_input_defaults("wh_8_64", DEVICE_INPUT_DEFAULTS_NAME(h8_wh_8_64_48k_from_zero));

	H8BUS_SLOT(config.replace(), m_p10, "h8bus", h8_p10_cards, "ha_8_8");
	m_p10->set_option_device_input_defaults("ha_8_8", DEVICE_INPUT_DEFAULTS_NAME(h8_ha_8_8_auto));

	// THE THREE SETTINGS THAT MAKE THIS AN ORG-0 MACHINE
	//
	// The HA-8-8 has three functions - boot device selection, the status port,
	// and ROM disable - and its Assembly and Operation manual (595-2509-1) has
	// the owner pick, on page 22: "The following sections describe the three
	// functions available with the Extended Configuration Board and some
	// modifications that you must make to other circuit boards in your H8 ...
	// Decide which functions you wish to use.  Then make the necessary
	// modifications for those functions."  ROM disable is the one taken here,
	// pages 24 through 27, and it means three changes - two of them soldering,
	// one a bank of switches - which are settings here rather than inferred
	// from the cards present, because an HA-8-8 bought for the other two
	// functions sits in a machine where none of this was done.
	//
	// They only work as a set.  Any one alone boots nothing: the H-17 stops
	// answering for 1400-1FFF, so RAM has to arrive there, and the RAM under
	// the monitor ROM is only reachable once X1-X2 can switch the ROM out.
	// Together they are what lets this machine run CP/M, and they cost HDOS
	// nothing - the manual says as much, and both still boot.
	m_p2->set_option_device_input_defaults("cpu8080", DEVICE_INPUT_DEFAULTS_NAME(h8_cpu_8080_ha_8_8));
	m_p8->set_option_device_input_defaults("h_8_17", DEVICE_INPUT_DEFAULTS_NAME(h8_h_8_17_org0));

	// With the HA-8-8 fitted there is a monitor that can boot the disk on its
	// own.  XCON8 is the Heath part (444-70, and it says so in its own image);
	// the other candidate, PAM-8/AT, is not a dump at all but a reconstruction
	// assembled from a 27-Oct-81 listing, whose text records that its original
	// part number is unknown.  Prefer the genuine one.
	m_p2->set_option_default_bios("cpu8080", "xcon8");

	// Same four-port card, but by 1982 the terminal in the box is an H19.
	m_p9->set_option_machine_config("h_8_4", h_8_4_with_h19);
}

// ROM definition
ROM_START( h8 )
ROM_END

#define rom_h8_hks82 rom_h8
#define rom_h8_hks85 rom_h8

} // anonymous namespace

// Driver

//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT  CLASS,    INIT        COMPANY          FULLNAME                               FLAGS
COMP( 1977, h8,        0,  0, h8,        h8, h8_state, empty_init, "Heath Company", "H-8",                                       MACHINE_SUPPORTS_SAVE )
COMP( 1979, h8_hks82, h8,  0, h8_hks82,  h8, h8_state, empty_init, "Heath Company", "H-8 HKS-82 Advanced Computer System",        MACHINE_SUPPORTS_SAVE )
COMP( 1982, h8_hks85, h8,  0, h8_hks85,  h8, h8_state, empty_init, "Heath Company", "H-8 HKS-85 Hobbyist/Word Processing System", MACHINE_SUPPORTS_SAVE )
