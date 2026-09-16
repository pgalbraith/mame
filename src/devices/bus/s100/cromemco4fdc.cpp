// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    Cromemco 4FDC floppy disk controller (1977)

    An FD1771 for the disks, a TMS5501 for the console serial port and
    five timers, and a 1K 2708 holding RDOS, the resident disk operating
    system. RDOS is both the boot loader for CDOS and a monitor, so a Z-2D
    needs no other firmware: the ZPU's power-on jump points at C000 and the
    card does the rest.

    Ports
    00-09  TMS5501. 00 reads status and writes the baud rate, 01 is the
           receive and transmit data, 02 writes the command register, 03
           reads the interrupt address and writes the mask, 04 is the
           parallel port, and 05-09 load timers 1 to 5.
    30-33  FD1771 status/command, track, sector and data.
    34     disk flags in, disk control out.
    40     bank select, write only.

    The status byte at port 00 is not the TMS5501's own order. A 74LS157
    (IC49) swaps D7 and D6 with D4 and D3, so software sees TBE, RDA, IPG,
    SBD, FBD, SRV, ORE, FME. The SCC board wires the same chip differently
    again, duplicating TBE and RDA instead of swapping, which is why
    mcb216.cpp uses a different bitswap.

    AUTO WAIT
    Writing bit 7 of port 34 puts the card in auto wait. A read of port 34
    then holds the CPU until the FD1771 raises DRQ or ends the command,
    which is how RDOS transfers a sector without testing DRQ itself: its
    read loop is IN 34, check end of job, INI, repeat. The card asks for
    the wait on the bus PRDY line, and the driver turns that into the Z80
    WAIT input and repeats the access.

    Not emulated
    - The interrupt priority chain on J1. The card latches INT on each M1
      and blocks boards further down the chain, but RDOS and CDOS poll.
    - The head load timer. HLD gates the drive select lines on the real
      card, which cannot be modelled directly because the FD1771 raises HLD
      itself; MAME's FD1771 assumes HLT is tied high anyway.
    - DINT, the switch 4 write protect, which blocks WRITE TRACK. MAME's
      FD1771 has no such input.
    - The PerSci eject and fast seek outputs on port 04.

    References
    - 4FDC manual 023-0005, January 1980, with the 1977 and September 1978
      editions for comparison. Ports in chapter 3, switches in chapter 1,
      theory of operation in chapter 7.
    - 4FDC schematic 020-0011 Rev 2, January 1980, for the status swap at
      IC49, the port 34 buffer IC9 and latch IC24, the 74S133 ROM decoder
      IC30 and the bank select flip-flop IC29.

**********************************************************************/

#include "emu.h"
#include "cromemco4fdc.h"

#include "bus/rs232/rs232.h"
#include "imagedev/floppy.h"
#include "machine/tms5501.h"
#include "machine/wd_fdc.h"

#include "formats/imd_dsk.h"


namespace {

class s100_cromemco_4fdc_device : public device_t, public device_s100_card_interface
{
public:
	s100_cromemco_4fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_smemr_r(offs_t offset) override;
	virtual u8 s100_sinp_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;
	virtual u8 s100_sinta_r(offs_t offset) override;

private:
	static void floppy_formats(format_registration &fr);

	u8 flags_r();
	void control_w(u8 data);
	u8 status_r();
	u8 parallel_r();

	void intrq_w(int state);
	void drq_w(int state);
	void hld_w(int state);
	void update_wait();
	void release_wait();

	required_device<tms5501_device> m_uart;
	required_device<fd1771_device> m_fdc;
	required_device_array<floppy_connector, 4> m_connectors;
	required_region_ptr<u8> m_rom;
	required_ioport m_switches;

	bool m_rom_disabled;    // set by a write to port 40 when switch 2 is on
	bool m_auto_wait;
	bool m_waiting;
	bool m_maxi;
	u8 m_drive_select;
	int m_intrq;
	int m_drq;
	int m_hld;
};

s100_cromemco_4fdc_device::s100_cromemco_4fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_CROMEMCO_4FDC, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_uart(*this, "uart")
	, m_fdc(*this, "fdc")
	, m_connectors(*this, "floppy%u", 0U)
	, m_rom(*this, "rdos")
	, m_switches(*this, "SW")
	, m_rom_disabled(false)
	, m_auto_wait(false)
	, m_waiting(false)
	, m_maxi(true)
	, m_drive_select(0)
	, m_intrq(0)
	, m_drq(0)
	, m_hld(0)
{
}

void s100_cromemco_4fdc_device::device_start()
{
	save_item(NAME(m_rom_disabled));
	save_item(NAME(m_auto_wait));
	save_item(NAME(m_waiting));
	save_item(NAME(m_maxi));
	save_item(NAME(m_drive_select));
	save_item(NAME(m_intrq));
	save_item(NAME(m_drq));
	save_item(NAME(m_hld));
}

void s100_cromemco_4fdc_device::device_reset()
{
	// RESET re-enables the ROM, sets motor on and MAXI, and deselects every
	// drive; the control latch IC24 and the auto wait flip flop IC41 are
	// both cleared from the bus reset line
	m_rom_disabled = false;
	m_auto_wait = false;
	m_maxi = true;
	m_drive_select = 0;
	m_fdc->set_floppy(nullptr);
	m_fdc->set_unscaled_clock(2_MHz_XTAL);
	m_fdc->set_force_ready(false);

	if (m_waiting)
	{
		m_waiting = false;
		m_bus->rdy_w(1);
	}
}


//**************************************************************************
//  memory
//**************************************************************************

u8 s100_cromemco_4fdc_device::s100_smemr_r(offs_t offset)
{
	// IC30, a 74S133, decodes C000-C3FF on a memory read; switch 1 takes the
	// board out of memory altogether and a write to port 40 does the same
	// until the next reset
	if (BIT(m_switches->read(), 0) || m_rom_disabled)
		return 0xff;

	if ((offset & 0xfc00) != 0xc000)
		return 0xff;

	return m_rom[offset & 0x3ff];
}


//**************************************************************************
//  I/O
//**************************************************************************

u8 s100_cromemco_4fdc_device::status_r()
{
	// IC49 swaps D7/D6 with D4/D3 on the way to the bus
	return bitswap<8>(m_uart->sta_r(), 4, 3, 5, 7, 6, 2, 1, 0);
}

u8 s100_cromemco_4fdc_device::parallel_r()
{
	// D7 is DRQ when the INTER 7 jumper is fitted, which it is not as
	// shipped; D6 is SEEK IN PROGRESS, which floats high without a PerSci
	// drive. The bottom six bits go to J4 and are free for the system.
	return 0x7f | (m_drq ? 0x80 : 0x00);
}

u8 s100_cromemco_4fdc_device::flags_r()
{
	// IC9, a 74368, puts four signals on the bus: D7 DRQ, D6 low when
	// switch 3 is set to BOOT, D5 the FD1771's head load output and D0 end
	// of job. Nothing drives D1-D4.
	u8 data = 0;

	if (m_drq)
		data |= 0x80;
	if (!BIT(m_switches->read(), 2))
		data |= 0x40;
	if (m_hld)
		data |= 0x20;
	if (m_intrq)
		data |= 0x01;

	return data;
}

void s100_cromemco_4fdc_device::control_w(u8 data)
{
	m_auto_wait = BIT(data, 7);

	bool const maxi = BIT(data, 4);
	if (maxi != m_maxi)
	{
		m_maxi = maxi;

		// the MAXI bit switches the FD1771 between 2 MHz for 8 inch drives
		// and 1 MHz for 5 inch ones, which halves the data rate and doubles
		// the step times
		m_fdc->set_unscaled_clock(maxi ? 2_MHz_XTAL : 1_MHz_XTAL);

		// IC10 only drives READY and FCLK for 8 inch drives; on 5 inch ones
		// RN2 holds the FD1771's READY pin high, so the chip is always ready
		m_fdc->set_force_ready(!maxi);
	}

	m_drive_select = data & 0x0f;

	floppy_image_device *floppy = nullptr;
	for (unsigned i = 0; i < 4; i++)
		if (BIT(m_drive_select, i))
			floppy = m_connectors[i]->get_device();

	m_fdc->set_floppy(floppy);

	if (floppy)
		floppy->mon_w(BIT(data, 5) ? 0 : 1);

	// the write only arms the board. IC41 stores the bit and nothing reaches
	// PRDY until the CPU reads port 34, so clearing the bit is the only effect
	// an OUT can have on a wait already under way.
	if (!m_auto_wait)
		release_wait();
}

u8 s100_cromemco_4fdc_device::s100_sinp_r(offs_t offset)
{
	// the card decodes the low byte only, so ignore whatever the CPU left in
	// the high half of the I/O address
	switch (offset & 0xff)
	{
	case 0x00:
		return machine().side_effects_disabled() ? bitswap<8>(m_uart->sta_r(), 4, 3, 5, 7, 6, 2, 1, 0) : status_r();
	case 0x01:
		return m_uart->rb_r();
	case 0x03:
		return m_uart->rst_r();
	case 0x04:
		return parallel_r();

	// the FD1771 has an active low data bus, and IC34, IC35, IC47 and IC48
	// invert it again during disk references, so the S-100 side sees true
	// data. MAME's fd1771_device models the chip's own pins, so the card has
	// to supply the board's inversion itself.
	case 0x30:
		return m_fdc->status_r() ^ 0xff;
	case 0x31:
		return m_fdc->track_r() ^ 0xff;
	case 0x32:
		return m_fdc->sector_r() ^ 0xff;
	case 0x33:
		return m_fdc->data_r() ^ 0xff;

	case 0x34:
		if (!machine().side_effects_disabled())
			update_wait();
		return flags_r();
	}

	return 0xff;
}

void s100_cromemco_4fdc_device::s100_sout_w(offs_t offset, u8 data)
{
	switch (offset & 0xff)
	{
	case 0x00:
		m_uart->rr_w(data);
		break;
	case 0x01:
		m_uart->tb_w(data);
		break;
	case 0x02:
		m_uart->cmd_w(data);
		break;
	case 0x03:
		m_uart->mr_w(data);
		break;
	case 0x04:
		m_uart->xo_w(data);
		break;
	case 0x05: case 0x06: case 0x07: case 0x08: case 0x09:
		m_uart->tmr_w((offset & 0xff) - 0x05, data);
		break;

	// inverted on the way in as well, by the same XOR gates
	case 0x30:
		m_fdc->cmd_w(data ^ 0xff);
		break;
	case 0x31:
		m_fdc->track_w(data ^ 0xff);
		break;
	case 0x32:
		m_fdc->sector_w(data ^ 0xff);
		break;
	case 0x33:
		m_fdc->data_w(data ^ 0xff);
		break;
	case 0x34:
		control_w(data);
		break;

	case 0x40:
		// switch 2 hands C000-C3FF to the system once CDOS is in memory; the
		// byte itself is not examined
		if (BIT(m_switches->read(), 1))
			m_rom_disabled = true;
		break;
	}
}

u8 s100_cromemco_4fdc_device::s100_sinta_r(offs_t offset)
{
	// the real card only answers when its priority latch caught INT on the
	// last M1 and PRIORITY IN is high; nothing here drives that chain
	return m_uart->get_vector();
}


//**************************************************************************
//  wait logic
//**************************************************************************

void s100_cromemco_4fdc_device::update_wait()
{
	// Only a read of port 34 starts a wait. With auto wait set and the FD1771
	// neither asking for a byte nor finished, the card pulls PRDY low and the
	// host repeats the access once DRQ or EOJ releases it.
	if (!m_auto_wait || m_drq || m_intrq || m_waiting)
		return;

	m_waiting = true;
	m_bus->rdy_w(0);
}

void s100_cromemco_4fdc_device::release_wait()
{
	// DRQ, EOJ and RESET are the only things that end a wait. DRQ dropping
	// between bytes must not stall the CPU outside a port 34 read.
	if (!m_waiting)
		return;

	m_waiting = false;
	m_bus->rdy_w(1);
}

void s100_cromemco_4fdc_device::intrq_w(int state)
{
	m_intrq = state;

	// INTRQ is end of job at port 34 bit 0, and it also clears auto wait
	if (state)
		m_auto_wait = false;

	// the FD1771's interrupt reaches the TMS5501 on SENS, which is interrupt
	// 2, the address the manual lists as D7
	m_uart->sens_w(state);

	if (state)
		release_wait();
}

void s100_cromemco_4fdc_device::drq_w(int state)
{
	m_drq = state;

	if (state)
		release_wait();
}

void s100_cromemco_4fdc_device::hld_w(int state)
{
	m_hld = state;
}


//**************************************************************************
//  machine configuration
//**************************************************************************

static void cromemco_floppies(device_slot_interface &device)
{
	device.option_add("525sssd", FLOPPY_525_SSSD);
	device.option_add("8sssd", FLOPPY_8_SSSD);
}

void s100_cromemco_4fdc_device::floppy_formats(format_registration &fr)
{
	fr.add_mfm_containers();
	fr.add(FLOPPY_IMD_FORMAT);
}

void s100_cromemco_4fdc_device::device_add_mconfig(machine_config &config)
{
	// everything on the card runs from its own 8 MHz crystal, so the CPU
	// speed does not affect disk or serial timing
	TMS5501(config, m_uart, 8_MHz_XTAL / 4);
	m_uart->xmt_callback().set("rs232", FUNC(rs232_port_device::write_txd));
	m_uart->int_callback().set([this] (int state) { m_bus->irq_w(state); });

	// J4 takes an RS-232 terminal on pins 2, 3 and 7, or a Teletype on the
	// 20 mA current loop. Cromemco sold no terminal in 1977 and told buyers
	// to bring their own: the Z-2D advertisement offers "an RS-232 serial
	// interface for interfacing your CRT terminal or teletype".
	//
	// adm3a is the default. Nothing has to be set for it: the terminal comes
	// up at its own factory 9600, which is in RDOS's rate table, and RDOS
	// finds the rate from the carriage returns the operator presses for the
	// prompt anyway.
	//
	// asr33 is the terminal Cromemco actually documented, with a 20 mA
	// wiring table in the 4FDC, TU-ART and SCC manuals, but it cannot reach
	// the RDOS prompt. RDOS reads a character at C0E4 as IN 01 / AND 7F /
	// RET, so a received 00 leaves Z set and its caller at C0EC cannot tell
	// it from an empty receiver. The Teletype runs at 110 baud while the
	// first table entry selects 2400, every character arrives as 00, and
	// RDOS never counts the two characters it needs to step to the next
	// rate. It never reaches the 110 baud entry that would have matched.
	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, "adm3a"));
	rs232.rxd_handler().set(m_uart, FUNC(tms5501_device::rcv_w));

	FD1771(config, m_fdc, 2_MHz_XTAL);
	m_fdc->intrq_wr_callback().set(FUNC(s100_cromemco_4fdc_device::intrq_w));
	m_fdc->drq_wr_callback().set(FUNC(s100_cromemco_4fdc_device::drq_w));
	m_fdc->hld_wr_callback().set(FUNC(s100_cromemco_4fdc_device::hld_w));

	// J3 takes 5 inch drives and J2 8 inch ones, driven in parallel; a Z-2D
	// came with one 5 inch drive and room for a second
	for (unsigned i = 0; i < 4; i++)
		FLOPPY_CONNECTOR(config, m_connectors[i], cromemco_floppies, (i < 2) ? "525sssd" : nullptr, floppy_formats).enable_sound(true);
}


//**************************************************************************
//  switches and ROM
//**************************************************************************

static INPUT_PORTS_START( cromemco_4fdc )
	PORT_START("SW")
	PORT_DIPNAME(0x01, 0x00, "Switch 1: RDOS disable")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x01, DEF_STR(On))
	PORT_DIPNAME(0x02, 0x02, "Switch 2: RDOS disable after boot")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x02, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "Switch 3: boot enable")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x04, DEF_STR(On))
	PORT_DIPNAME(0x08, 0x00, "Switch 4: initialization inhibit")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x08, DEF_STR(On))
INPUT_PORTS_END

ioport_constructor s100_cromemco_4fdc_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cromemco_4fdc);
}

ROM_START( cromemco_4fdc )
	ROM_REGION( 0x400, "rdos", 0 )
	ROM_DEFAULT_BIOS("rdos13")

	ROM_SYSTEM_BIOS( 0, "rdos13", "RDOS 1.3" )
	ROMX_LOAD( "rdos_1.3.bin", 0x000, 0x400, CRC(7f8f2ee0) SHA1(b402e395f371139b032185f1303a8d8045d66ede), ROM_BIOS(0) )

	// z80pack's copy of this one was retyped from the listing in the RDOS
	// manual rather than read from a part, so it may not match a real 2708
	ROM_SYSTEM_BIOS( 1, "rdos10", "RDOS 1.0" )
	ROMX_LOAD( "rdos_1.0.bin", 0x000, 0x400, BAD_DUMP CRC(2471e484) SHA1(5f9bf7d7d72bc4418f388de4917f3a998d53dad9), ROM_BIOS(1) )
ROM_END

const tiny_rom_entry *s100_cromemco_4fdc_device::device_rom_region() const
{
	return ROM_NAME(cromemco_4fdc);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_CROMEMCO_4FDC, device_s100_card_interface, s100_cromemco_4fdc_device, "s100_cromemco_4fdc", "Cromemco 4FDC Floppy Disk Controller")
