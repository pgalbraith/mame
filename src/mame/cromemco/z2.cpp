// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

    Cromemco Z-2, Z-2D and Z-2H

    A rack chassis, a 21 slot S-100 motherboard and a ZPU card carrying a
    4 MHz Z-80A. There is no front panel: a rocker switch on the back is
    wired to PRESET, and the ZPU's power-on jump starts the machine at one
    of sixteen addresses chosen with a four position switch, so the first
    instruction comes from a ROM on some other card.

    Z-2 (1977)
    The chassis, the motherboard and the ZPU. Memory, I/O and firmware are
    all extra. Fitted here is what the manual calls the minimum system: a
    16KPR holding the Z-80 Monitor at E000, a 16KZ for RAM, and a TU-ART
    for the console, with the power-on jump set to E000. The manual asks
    for the monitor in an 8K Bytesaver, which lands E000 in its first
    socket; the 16KPR reaches the same address from its block at C000.

    Getting to the monitor
    Press RETURN a few times. The monitor walks a table of baud rates,
    writing each to the TU-ART and waiting for two carriage returns, and
    9600 is the second entry, so the sign-on takes about four presses. It
    then searches down from FFE9 for the highest page of RAM and puts its
    stack there, which is why the machine needs a RAM card as well.

    Z-2D (1977)
    A Z-2 with a 4FDC and one or two 5 inch drives. The 4FDC carries RDOS
    in a 1K PROM at C000, so the jump switch is set to C000 and the machine
    starts in RDOS whether or not a disk is in the drive.

    Z-2H (1979)
    A 12 slot machine with a hard disk: ZPU, 4FDC, 64KZ, PRI and WDI, and
    an 11 megabyte drive. The floppy side is the same as a Z-2D, and the
    same RDOS boots it.

    Getting to CDOS
    Set the 4FDC's switch 3 on to boot straight from disk, or leave it off
    and type B at the RDOS prompt. Either way RDOS reads track 0 sector 1
    into 0080 and jumps there, and the sector's first instruction writes 01
    to port 40H, which turns the top half of memory on and RDOS off.
    RDOS finds the console baud rate from carriage returns, so press RETURN
    a few times before expecting a prompt.

    Not emulated
    - The PRI printer interface and the WDI hard disk on a Z-2H.
    - The ZPU's wait state jumpers, which only relax memory access times.
    - The 4 MHz indicator on bus pin 98, which the 16KPR and 4FDC read to
      decide their own wait states.

    References
    - ZPU manual 023-0012, September 1978 and January 1980. Power-on jump in
      section 2.1, clock in 2.2, wait states in 2.3, mirrors in 2.4 and 2.5.
    - ZPU schematic 040-0019 Rev 6, for the jump circuit: a 74164 clocked by
      SYNC, a 74157 switching in the address switches and 74367s driving the
      CPU bus while the S-100 data in buffers are held off.
    - Z-2 and Z-2D instruction manual, 1978.
    - Z-2H brochure 023-0083, January 1980, for the card set.

***************************************************************************/

#include "emu.h"

#include "bus/s100/s100.h"
#include "bus/s100/cromemco16kpr.h"
#include "bus/s100/cromemco4fdc.h"
#include "bus/s100/cromemcoram.h"
#include "bus/s100/cromemcotuart.h"
#include "cpu/z80/z80.h"


namespace {

class z2_state : public driver_device
{
public:
	z2_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_bus(*this, "s100")
		, m_jump(*this, "JUMP")
		, m_speed(*this, "SPEED")
		, m_jump_step(0)
	{ }

	void z2(machine_config &config) ATTR_COLD;
	void z2d(machine_config &config) ATTR_COLD;
	void z2h(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void common(machine_config &config, unsigned slots) ATTR_COLD;
	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 mem_r(offs_t offset);
	void mem_w(offs_t offset, u8 data);
	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);

	void rdy_w(int state);
	IRQ_CALLBACK_MEMBER(irq_callback);

	required_device<z80_device> m_maincpu;
	required_device<s100_bus_device> m_bus;
	required_ioport m_jump;
	required_ioport m_speed;

	u8 m_jump_step;
};

void z2_state::machine_start()
{
	save_item(NAME(m_jump_step));
}

void z2_state::machine_reset()
{
	m_jump_step = 0;

	// the toggle switch on the ZPU picks 4 MHz or 2 MHz; everything on the
	// 4FDC runs from its own crystal, so disk and serial timing do not change
	m_maincpu->set_unscaled_clock(BIT(m_speed->read(), 0) ? 4_MHz_XTAL : 2_MHz_XTAL);
}


//**************************************************************************
//  ZPU
//**************************************************************************

u8 z2_state::mem_r(offs_t offset)
{
	// The card addresses memory during the jump as usual, so let the bus see
	// the cycle even though the CPU is about to ignore what comes back.
	u8 const data = m_bus->smemr_r(offset);

	// After a reset the ZPU feeds the CPU C3 00 x0, which is JP x000, over
	// its first three machine cycles, with the S-100 data in buffers held
	// off. A 74164 counts the cycles from SYNC and turns the drivers off
	// again once it reaches the fourth.
	u8 const jump = m_jump->read();
	if (BIT(jump, 4) && (m_jump_step < 3))
	{
		u8 const sequence[3] = { 0xc3, 0x00, u8((jump & 0x0f) << 4) };
		u8 const value = sequence[m_jump_step];

		if (!machine().side_effects_disabled())
			m_jump_step++;

		return value;
	}

	return data;
}

void z2_state::mem_w(offs_t offset, u8 data)
{
	m_bus->mwrt_w(offset, data);
}

u8 z2_state::io_r(offs_t offset)
{
	return m_bus->sinp_r(offset);
}

void z2_state::io_w(offs_t offset, u8 data)
{
	m_bus->sout_w(offset, data);
}

void z2_state::rdy_w(int state)
{
	// A card pulls PRDY low to stall the CPU. The 4FDC does it for auto
	// wait, where a read of port 34 waits for the disk, so the access has to
	// run again once the card lets go.
	m_maincpu->set_input_line(Z80_INPUT_LINE_WAIT, state ? CLEAR_LINE : ASSERT_LINE);

	if (!state)
		m_maincpu->retry_access();
}

IRQ_CALLBACK_MEMBER(z2_state::irq_callback)
{
	return m_bus->sinta_r(0);
}

void z2_state::mem_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(z2_state::mem_r), FUNC(z2_state::mem_w));
}

void z2_state::io_map(address_map &map)
{
	// The ZPU leaves the whole 16 bit I/O address on the bus. With the
	// address mirror fitted, which is how the card ships, the port number
	// appears in both halves as it would on an 8080; every Cromemco card
	// decodes the low byte only.
	map(0x0000, 0xffff).rw(FUNC(z2_state::io_r), FUNC(z2_state::io_w));
}


//**************************************************************************
//  machine configuration
//**************************************************************************

static void cromemco_s100_cards(device_slot_interface &device)
{
	device.option_add("4fdc", S100_CROMEMCO_4FDC);
	device.option_add("16kpr", S100_CROMEMCO_16KPR);
	device.option_add("16kz", S100_CROMEMCO_16KZ);
	device.option_add("64kz", S100_CROMEMCO_64KZ);
	device.option_add("tuart", S100_CROMEMCO_TUART);
}

void z2_state::common(machine_config &config, unsigned slots)
{
	Z80(config, m_maincpu, 4_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &z2_state::mem_map);
	m_maincpu->set_addrmap(AS_IO, &z2_state::io_map);
	m_maincpu->set_irq_acknowledge_callback(FUNC(z2_state::irq_callback));

	S100_BUS(config, m_bus, 4_MHz_XTAL);
	m_bus->irq().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	m_bus->nmi().set_inputline(m_maincpu, INPUT_LINE_NMI);
	m_bus->rdy().set(FUNC(z2_state::rdy_w));

	for (unsigned i = 1; i <= slots; i++)
		S100_SLOT(config, util::string_format("s100:%u", i).c_str(), cromemco_s100_cards, nullptr);
}

void z2_state::z2(machine_config &config)
{
	// A bare Z-2 is the chassis, the motherboard and the ZPU; the buyer adds
	// memory, a console and firmware. Fitted here is what the Z-2 manual
	// calls the minimum system: a ROM card holding the Z-80 Monitor, a RAM
	// card, and a TU-ART for the console. The power-on jump goes to E000 to
	// match, so the machine signs on with nothing on the command line.
	common(config, 21);

	S100_SLOT(config.replace(), "s100:1", cromemco_s100_cards, "tuart");
	S100_SLOT(config.replace(), "s100:2", cromemco_s100_cards, "16kz");
	S100_SLOT(config.replace(), "s100:3", cromemco_s100_cards, "16kpr");
}

void z2_state::z2d(machine_config &config)
{
	common(config, 21);

	S100_SLOT(config.replace(), "s100:1", cromemco_s100_cards, "4fdc");
	S100_SLOT(config.replace(), "s100:2", cromemco_s100_cards, "64kz");
}

void z2_state::z2h(machine_config &config)
{
	// the Z-2H motherboard has twelve slots; the PRI and the WDI that came
	// with it are not emulated yet
	common(config, 12);

	S100_SLOT(config.replace(), "s100:1", cromemco_s100_cards, "4fdc");
	S100_SLOT(config.replace(), "s100:2", cromemco_s100_cards, "64kz");
}


//**************************************************************************
//  switches
//**************************************************************************

// the sixteen addresses the ZPU's four position switch can select
#define JUMP_ADDRESSES 	PORT_CONFSETTING(0x00, "0000") 	PORT_CONFSETTING(0x01, "1000") 	PORT_CONFSETTING(0x02, "2000") 	PORT_CONFSETTING(0x03, "3000") 	PORT_CONFSETTING(0x04, "4000") 	PORT_CONFSETTING(0x05, "5000") 	PORT_CONFSETTING(0x06, "6000") 	PORT_CONFSETTING(0x07, "7000") 	PORT_CONFSETTING(0x08, "8000") 	PORT_CONFSETTING(0x09, "9000") 	PORT_CONFSETTING(0x0a, "A000") 	PORT_CONFSETTING(0x0b, "B000") 	PORT_CONFSETTING(0x0c, "C000") 	PORT_CONFSETTING(0x0d, "D000") 	PORT_CONFSETTING(0x0e, "E000") 	PORT_CONFSETTING(0x0f, "F000")

static INPUT_PORTS_START( z2d )
	PORT_START("JUMP")
	// C000 is the 4FDC's RDOS
	PORT_CONFNAME(0x0f, 0x0c, "Power-on jump address")
	JUMP_ADDRESSES
	PORT_CONFNAME(0x10, 0x10, "Power-on jump")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x10, DEF_STR(On))

	PORT_START("SPEED")
	PORT_CONFNAME(0x01, 0x01, "Clock")
	PORT_CONFSETTING(0x00, "2 MHz")
	PORT_CONFSETTING(0x01, "4 MHz")
INPUT_PORTS_END

static INPUT_PORTS_START( z2 )
	PORT_INCLUDE( z2d )

	// E000 is the Z-80 Monitor in the 16KPR
	PORT_MODIFY("JUMP")
	PORT_CONFNAME(0x0f, 0x0e, "Power-on jump address")
	JUMP_ADDRESSES
INPUT_PORTS_END


ROM_START( z2 )
ROM_END

#define rom_z2d rom_z2
#define rom_z2h rom_z2

} // anonymous namespace


//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS     INIT        COMPANY      FULLNAME  FLAGS
COMP( 1977, z2d,  0,      0,      z2d,     z2d,   z2_state, empty_init, "Cromemco",  "Z-2D",   MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW | MACHINE_SUPPORTS_SAVE )
COMP( 1977, z2,   z2d,    0,      z2,      z2,    z2_state, empty_init, "Cromemco",  "Z-2",    MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW | MACHINE_SUPPORTS_SAVE )
COMP( 1979, z2h,  z2d,    0,      z2h,     z2d,   z2_state, empty_init, "Cromemco",  "Z-2H",   MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW | MACHINE_SUPPORTS_SAVE )
