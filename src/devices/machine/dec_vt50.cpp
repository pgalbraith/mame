// license:BSD-3-Clause
// copyright-holders:AJR, Paul Galbraith
/****************************************************************************

    DEC VT50 DECscope (basic model)

    This cannot run: none of its four microcode PROMs has been dumped.

    Everything below was read off DEC's documents rather than a working
    terminal. The PROM part numbers and sockets, the keyboard matrix, the
    switch wiring and the character generator's dot order come from the
    VT50 field maintenance print set MP00036, January 1976
    [http://bitsavers.org/pdf/dec/terminal/vt50/MP00036_VT50_Maintenance_Drawings_Jan1976.pdf].
    The switch positions come from the base decal in the February 1975
    print set [http://bitsavers.org/pdf/dec/terminal/vt50/VT50-print-set.pdf],
    and the key legends and shifted characters from the DECscope Users'
    Manual EK-VT5X-OP-001
    [http://bitsavers.org/pdf/dec/terminal/vt52/EK-VT5X-OP-001_DECscope_Users_Manual_Mar77.pdf].
    Where the drawings did not settle something, the code says so.

    Only the basic VT50 is here: ROM UART and Timing board 5410902-0, with
    jumper W7 fitted. The VT50H and the VT50 with copier support fill all
    eight PROM sockets.

    The terminal is a device so that it can sit on any RS-232 port as the
    vt50 option; the vt50 driver in dec/vt52.cpp only gives it an EIA port.

****************************************************************************/

#include "emu.h"
#include "dec_vt50.h"

#include "sound/spkrdev.h"

#include "screen.h"
#include "speaker.h"


DEFINE_DEVICE_TYPE(DEC_VT50, dec_vt50_device, "dec_vt50", "DEC VT50 DECscope")

dec_vt50_device::dec_vt50_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, DEC_VT50, tag, owner, clock)
	, m_maincpu(*this, "maincpu")
	, m_uart(*this, "uart")
	, m_keys(*this, "KEY%d", 0U)
	, m_break_key(*this, "BREAK")
	, m_mode_sw(*this, "MODE")
	, m_speed_sw(*this, "SPEED")
	, m_data_sw(*this, "DATABITS")
	, m_chargen(*this, "chargen")
	, m_write_sd(*this)
	, m_serial_out(true)
	, m_rec_data(true)
	, m_9600_clock(false)
	, m_baud_divider(0)
	, m_110_baud_counter(0)
{
}

void dec_vt50_device::device_start()
{
	save_item(NAME(m_serial_out));
	save_item(NAME(m_rec_data));
	save_item(NAME(m_9600_clock));
	save_item(NAME(m_baud_divider));
	save_item(NAME(m_110_baud_counter));
}

void dec_vt50_device::device_reset()
{
	m_baud_divider = 0;
	m_110_baud_counter = 0;

	update_serial_settings();
	m_uart->write_swe(0);
}

void dec_vt50_device::update_serial_settings()
{
	u8 db = m_data_sw->read();
	m_uart->write_nb1(BIT(db, 0));
	m_uart->write_np(BIT(db, 0));
	m_uart->write_eps(BIT(db, 1));
	m_uart->write_nb2(1);
	m_uart->write_tsb(m_speed_sw->read() == SPEED_110); // second deck of S2
	m_uart->write_cs(1);

	update_uart_clocks();
	gated_serial_output();
}

INPUT_CHANGED_MEMBER(dec_vt50_device::serial_sw_changed)
{
	update_serial_settings();
}

u8 dec_vt50_device::key_r(offs_t offset)
{
	// a 7442 decodes AC0-AC2 into the matrix lines, and a 74151 picks one of seven key groups with AC3-AC5 (its eighth input is grounded)
	const unsigned group = (offset >> 3) & 7;
	return group == 7 ? 1 : BIT(m_keys[group]->read(), offset & 7);
}

void dec_vt50_device::vert_count_w(u8 data)
{
	// The CPU's vertical counter steps once every ten characters, 153.6 kHz, which is the 1402's 16x clock for 9600 baud.
	// Do not use baud_9600_callback instead: it fires once per scan line, 15.36 kHz, and every speed comes out 10x slow
	// (measured from the 1402's receive pulse count). Two 74197s ripple-divide 9600 baud down to 75 baud, and a 74161
	// reloaded with 5 divides the 1200 baud output by 11 for 110 baud.
	const u8 old = m_baud_divider;
	m_baud_divider = (m_baud_divider + 1) & 0177;
	if (!BIT(old, 2) && BIT(m_baud_divider, 2))
		m_110_baud_counter = m_110_baud_counter == 15 ? 5 : m_110_baud_counter + 1;

	// 9600 baud gets one whole clock pulse per step
	m_9600_clock = true;
	update_uart_clocks();
	m_9600_clock = false;
	update_uart_clocks();
}

void dec_vt50_device::update_uart_clocks()
{
	// S1 positions 4 to 6 give the transmitter a speed of its own; positions 1 to 3 connect it to the receiver's clock
	int xclk;
	switch (m_mode_sw->read())
	{
	case MODE_300: xclk = BIT(m_baud_divider, 4); break;
	case MODE_150: xclk = BIT(m_baud_divider, 5); break;
	case MODE_75:  xclk = BIT(m_baud_divider, 6); break;
	default:       xclk = -1; break;
	}

	int rclk;
	switch (m_speed_sw->read())
	{
	case SPEED_110:  rclk = BIT(m_110_baud_counter, 3); break;
	case SPEED_600:  rclk = BIT(m_baud_divider, 3); break;
	case SPEED_1200: rclk = BIT(m_baud_divider, 2); break;
	case SPEED_2400: rclk = BIT(m_baud_divider, 1); break;
	case SPEED_4800: rclk = BIT(m_baud_divider, 0); break;
	case SPEED_9600: rclk = m_9600_clock; break;
	default:         rclk = xclk; break; // Bell 103: receive at the transmitting speed
	}

	if (xclk < 0)
		xclk = rclk;
	if (rclk < 0)
		rclk = xclk = 1; // S1 at 1 to 3 with S2 at A leaves both clocks pulled up, halting the UART

	m_uart->write_rcp(rclk);
	m_uart->write_tcp(xclk);
}

void dec_vt50_device::uart_xd_w(u8 data)
{
	if (BIT(m_data_sw->read(), 2))
		m_uart->transmit(data | 0x80);
	else
		m_uart->transmit(data & 0x7f);
}

void dec_vt50_device::gated_serial_output()
{
	// BREAK and off-line handling copy the VT52 driver; the VT50's own E45/E21 gating was not traced gate by gate
	if (m_mode_sw->read() != MODE_OFF_LINE)
		m_write_sd(m_serial_out && m_break_key->read());
	update_serial_in();
}

void dec_vt50_device::update_serial_in()
{
	// off-line and with local copy the transmitted data is also fed to the receiver, which ignores the line when off-line
	const u8 mode = m_mode_sw->read();
	bool si = mode == MODE_OFF_LINE || m_rec_data;
	if (mode == MODE_OFF_LINE || mode == MODE_LOCAL_COPY)
		si = si && m_serial_out && m_break_key->read();
	m_uart->write_si(si);
}

void dec_vt50_device::serial_out_w(int state)
{
	if (m_serial_out != bool(state))
	{
		m_serial_out = state;
		gated_serial_output();
	}
}

void dec_vt50_device::break_w(int state)
{
	gated_serial_output();
}

void dec_vt50_device::serial_in_w(int state)
{
	m_rec_data = state;

	if (machine().ioport().safe_to_read())
		update_serial_in();
}

int dec_vt50_device::xrdy_eoc_r()
{
	return m_uart->tbmt_r() && m_uart->eoc_r();
}

u8 dec_vt50_device::chargen_r(offs_t offset)
{
	// The 2513's outputs 1 to 5 are inverted onto CD4 down to CD0, and two 7495s shift out a blank, CD0 to CD4, then two
	// more blanks, so output 5 (bit 4 of the dump) is the leftmost dot. RAM bit 6 does not reach the 2513.
	return (~m_chargen[offset & 0777] & 037) << 2 | 3;
}

void dec_vt50_device::rom_1k(address_map &map)
{
	map(00000, 01777).rom().region("program", 0);
}

void dec_vt50_device::ram_1k(address_map &map)
{
	map(00000, 01777).ram(); // seven 2102s
}

static INPUT_PORTS_START(dec_vt50)
	// Each group and bit is the key's switch number on Keyboard A VT50 (D-CS-5410893-0-1), read as two octal digits.
	PORT_START("KEY0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED) // no switch
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('Y') PORT_CODE(KEYCODE_Y)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('I') PORT_CODE(KEYCODE_I)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Space Bar") PORT_CHAR(' ') PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('C') PORT_CODE(KEYCODE_C)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('A') PORT_CODE(KEYCODE_A)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Return") PORT_CHAR(015) PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Esc (Sel)") PORT_CHAR(033) PORT_CODE(KEYCODE_ESC)

	PORT_START("KEY1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('G') PORT_CODE(KEYCODE_G)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('J') PORT_CODE(KEYCODE_J)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('B') PORT_CODE(KEYCODE_B)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('V') PORT_CODE(KEYCODE_V)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('W') PORT_CODE(KEYCODE_W)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('Z') PORT_CODE(KEYCODE_Z)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Delete") PORT_CHAR(UCHAR_MAMEKEY(DEL)) PORT_CODE(KEYCODE_DEL)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Tab") PORT_CHAR(011) PORT_CODE(KEYCODE_TAB)

	PORT_START("KEY2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('N') PORT_CODE(KEYCODE_N)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('M') PORT_CODE(KEYCODE_M)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('F') PORT_CODE(KEYCODE_F)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('E') PORT_CODE(KEYCODE_E)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('S') PORT_CODE(KEYCODE_S)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('X') PORT_CODE(KEYCODE_X)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Back Space") PORT_CHAR(010) PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('Q') PORT_CODE(KEYCODE_Q)

	PORT_START("KEY3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('T') PORT_CODE(KEYCODE_T)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('U') PORT_CODE(KEYCODE_U)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('L') PORT_CODE(KEYCODE_L)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('O') PORT_CODE(KEYCODE_O)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('D') PORT_CODE(KEYCODE_D)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('P') PORT_CODE(KEYCODE_P)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Line Feed") PORT_CHAR(012) PORT_CODE(KEYCODE_INSERT)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('\\') PORT_CODE(KEYCODE_BACKSLASH)

	PORT_START("KEY4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('H') PORT_CODE(KEYCODE_H)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('K') PORT_CODE(KEYCODE_K)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('R') PORT_CODE(KEYCODE_R)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('8') PORT_CHAR('*') PORT_CODE(KEYCODE_8)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('2') PORT_CHAR('@') PORT_CODE(KEYCODE_2)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('\'') PORT_CHAR('"') PORT_CODE(KEYCODE_QUOTE) // lettered the same as / at KEY6 0x10, so the two may be swapped
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('-') PORT_CHAR('_') PORT_CODE(KEYCODE_MINUS)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('[') PORT_CHAR(']') PORT_CODE(KEYCODE_OPENBRACE)

	PORT_START("KEY5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('3') PORT_CHAR('#') PORT_CODE(KEYCODE_3)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('5') PORT_CHAR('%') PORT_CODE(KEYCODE_5)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('.') PORT_CHAR('>') PORT_CODE(KEYCODE_STOP)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(';') PORT_CHAR(':') PORT_CODE(KEYCODE_COLON)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('1') PORT_CHAR('!') PORT_CODE(KEYCODE_1)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(',') PORT_CHAR('<') PORT_CODE(KEYCODE_COMMA)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('=') PORT_CHAR('+') PORT_CODE(KEYCODE_EQUALS)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('0') PORT_CHAR(')') PORT_CODE(KEYCODE_0)

	PORT_START("KEY6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('4') PORT_CHAR('$') PORT_CODE(KEYCODE_4)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('6') PORT_CHAR('^') PORT_CODE(KEYCODE_6)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('9') PORT_CHAR('(') PORT_CODE(KEYCODE_9)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('7') PORT_CHAR('&') PORT_CODE(KEYCODE_7)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('/') PORT_CHAR('?') PORT_CODE(KEYCODE_SLASH)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Scroll") PORT_CODE(KEYCODE_LALT) // PAGE on the keyboard drawing
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Copy") PORT_CHAR(UCHAR_MAMEKEY(PRTSCR)) PORT_CODE(KEYCODE_RCONTROL) // PRINT on the keyboard drawing; does nothing without the copier
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Shift") PORT_CHAR(UCHAR_SHIFT_1) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) // both keys

	PORT_START("CTRL") // wired to the ROM UART and Timing board through jumper W2, not through the matrix
	PORT_BIT(1, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Ctrl") PORT_CHAR(UCHAR_SHIFT_2) PORT_CODE(KEYCODE_LCONTROL)

	PORT_START("BREAK") // not in the matrix, and not readable by the CPU
	PORT_BIT(1, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Break") PORT_CODE(KEYCODE_PAUSE) PORT_WRITE_LINE_MEMBER(FUNC(dec_vt50_device::break_w))

	PORT_START("MODE") // rotary switch S1 under the keyboard, positions as named on the base decal
	PORT_DIPNAME(0x7, dec_vt50_device::MODE_OFF_LINE, "Mode") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(dec_vt50_device::serial_sw_changed), 0)
	PORT_DIPSETTING(dec_vt50_device::MODE_OFF_LINE, "Off-Line") // S1:1
	PORT_DIPSETTING(dec_vt50_device::MODE_LOCAL_COPY, "Full Duplex with Local Copy") // S1:2
	PORT_DIPSETTING(dec_vt50_device::MODE_FULL_DUPLEX, "Full Duplex") // S1:3
	PORT_DIPSETTING(dec_vt50_device::MODE_300, "Full Duplex, Transmit 300 Baud") // S1:4
	PORT_DIPSETTING(dec_vt50_device::MODE_150, "Full Duplex, Transmit 150 Baud") // S1:5
	PORT_DIPSETTING(dec_vt50_device::MODE_75, "Full Duplex, Transmit 75 Baud") // S1:6

	PORT_START("SPEED") // rotary switch S2 under the keyboard
	PORT_DIPNAME(0x7, dec_vt50_device::SPEED_9600, "Speed") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(dec_vt50_device::serial_sw_changed), 0)
	PORT_DIPSETTING(dec_vt50_device::SPEED_BELL_103, "Bell 103 (Receive at Transmit Speed)") // S2:A
	PORT_DIPSETTING(dec_vt50_device::SPEED_110, "110 Baud with 2 Stop Bits") // S2:B
	PORT_DIPSETTING(dec_vt50_device::SPEED_600, "600 Baud") // S2:C
	PORT_DIPSETTING(dec_vt50_device::SPEED_1200, "1200 Baud") // S2:D
	PORT_DIPSETTING(dec_vt50_device::SPEED_2400, "2400 Baud") // S2:E
	PORT_DIPSETTING(dec_vt50_device::SPEED_4800, "4800 Baud") // S2:F
	PORT_DIPSETTING(dec_vt50_device::SPEED_9600, "9600 Baud") // S2:G
	// Bell 103 with S1 at 1 to 3 is illegal (both UART clock lines are pulled up, halting the UART)

	PORT_START("DATABITS")
	PORT_DIPNAME(0x1, 0x1, "Parity") PORT_DIPLOCATION("S4:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(dec_vt50_device::serial_sw_changed), 0)
	PORT_DIPSETTING(0x0, "Even (7 Data Bits)")
	PORT_DIPSETTING(0x1, "None (8 Data Bits)")
	PORT_DIPNAME(0x2, 0x2, "Parity Sense") PORT_DIPLOCATION("W6:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(dec_vt50_device::serial_sw_changed), 0)
	PORT_DIPSETTING(0x2, "Even")
	PORT_DIPSETTING(0x0, "Odd")
	PORT_DIPNAME(0x4, 0x0, "Data Bit 7") PORT_DIPLOCATION("W5:1") // the notes on sheet 1 of the ROM UART and Timing drawing call this jumper W8
	PORT_DIPSETTING(0x0, "Spacing")
	PORT_DIPSETTING(0x4, "Marking") // actually the hardware default, but not as good for modern use

	PORT_START("KEYCLICK")
	PORT_DIPNAME(1, 1, "Key Click") PORT_DIPLOCATION("S5:1") // which level the firmware takes as on is unknown
	PORT_DIPSETTING(0, DEF_STR(Off))
	PORT_DIPSETTING(1, DEF_STR(On))

	PORT_START("60HJ")
	PORT_DIPNAME(1, 1, "Unit Frequency") PORT_DIPLOCATION("W1:1")
	PORT_DIPSETTING(0, "50 Hz")
	PORT_DIPSETTING(1, "60 Hz")
INPUT_PORTS_END

ioport_constructor dec_vt50_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(dec_vt50);
}

void dec_vt50_device::device_add_mconfig(machine_config &config)
{
	VT50_CPU(config, m_maincpu, 13.824_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &dec_vt50_device::rom_1k);
	m_maincpu->set_addrmap(AS_DATA, &dec_vt50_device::ram_1k);
	m_maincpu->set_screen("screen");
	m_maincpu->vert_count_callback().set(FUNC(dec_vt50_device::vert_count_w));
	m_maincpu->uart_rd_callback().set(m_uart, FUNC(ay51013_device::receive));
	m_maincpu->uart_xd_callback().set(FUNC(dec_vt50_device::uart_xd_w));
	m_maincpu->ur_flag_callback().set(m_uart, FUNC(ay51013_device::dav_r));
	m_maincpu->ut_flag_callback().set(FUNC(dec_vt50_device::xrdy_eoc_r));
	m_maincpu->ruf_callback().set(m_uart, FUNC(ay51013_device::write_rdav));
	m_maincpu->key_up_callback().set(FUNC(dec_vt50_device::key_r));
	m_maincpu->kclk_callback().set_ioport("KEYCLICK");
	m_maincpu->frq_callback().set_ioport("60HJ");
	m_maincpu->bell_callback().set("bell", FUNC(speaker_sound_device::level_w));
	m_maincpu->char_data_callback().set(FUNC(dec_vt50_device::chargen_r));
	m_maincpu->ctrl_key_callback().set_ioport("CTRL");

	AY51013(config, m_uart); // 1402 at E7
	m_uart->write_so_callback().set(FUNC(dec_vt50_device::serial_out_w));

	SCREEN(config, "screen");

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, "bell").add_route(ALL_OUTPUTS, "mono", 1.0);
}

ROM_START(dec_vt50)
	ROM_REGION(0x400, "program", ROMREGION_ERASEFF) // 5603A bipolar PROMs; sockets E8/E24 and E11/E28 (pages 2 and 3) are empty and disabled by W7
	ROM_LOAD_NIB_LOW( "23-082a2.e1",  0x000, 0x100, NO_DUMP)
	ROM_LOAD_NIB_HIGH("23-083a2.e15", 0x000, 0x100, NO_DUMP)
	ROM_LOAD_NIB_LOW( "23-084a2.e4",  0x100, 0x100, NO_DUMP)
	ROM_LOAD_NIB_HIGH("23-085a2.e19", 0x100, 0x100, NO_DUMP)

	// DEC's number for a 2513, and the engineering specification asks for a Signetics 2513 or equivalent. These are the
	// stock 2513 contents, the same as the Apple I's s2513.d2; nobody has dumped a VT50's.
	ROM_REGION(0x200, "chargen", 0)
	ROM_LOAD("23-000a7.e26", 0x000, 0x200, BAD_DUMP CRC(a7e567fc) SHA1(b18aae0a2d4f92f5a7e22640719bbc4652f3f4ee))
ROM_END

const tiny_rom_entry *dec_vt50_device::device_rom_region() const
{
	return ROM_NAME(dec_vt50);
}
