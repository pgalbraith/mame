// license:BSD-3-Clause
// copyright-holders:AJR
/****************************************************************************

    DEC VT50 terminal family

    The VT50 "DECscope" was DEC's first video terminal to contain a CPU of
    sorts, with TTL logic spanning two boards executing custom microcode.
    It displayed 12 lines of 80-column text, using a standard character
    generator that only contained uppercase letters and symbols.

    The VT52 used the same case and most of the same circuitry as the VT50,
    but quickly displaced it by supporting 24 lines of text and a full ASCII
    character generator (on a board of its own). VT50 and VT52 each had
    minor variants differing in keyboard function and printer availability.

    The VT55 DECgraphic Scope was a graphical terminal based on the same
    main boards as the VT50 and VT52.

****************************************************************************/

#include "emu.h"
#include "bus/rs232/rs232.h"
#include "cpu/vt50/vt50.h"
#include "machine/ay31015.h"
#include "sound/spkrdev.h"
#include "screen.h"
#include "speaker.h"


namespace {

class vt52_state : public driver_device
{
public:
	vt52_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_uart(*this, "uart")
		, m_eia(*this, "eia")
		, m_keys(*this, "KEY%d", 0U)
		, m_break_key(*this, "BREAK")
		, m_baud_sw(*this, "BAUD")
		, m_data_sw(*this, "DATABITS")
		, m_chargen(*this, "chargen")
		, m_serial_out(true)
		, m_rec_data(true)
		, m_110_baud_counter(0)
	{
	}

	void vt52(machine_config &config);

	void break_w(int state);
	DECLARE_INPUT_CHANGED_MEMBER(data_sw_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void update_serial_settings();

	u8 key_r(offs_t offset);
	void baud_9600_w(int state);
	void vert_count_w(u8 data);
	void uart_xd_w(u8 data);
	void gated_serial_output(bool state);
	void serial_out_w(int state);
	void rec_data_w(int state);
	int xrdy_eoc_r();
	u8 chargen_r(offs_t offset);

	void rom_1k(address_map &map) ATTR_COLD;
	void ram_2k(address_map &map) ATTR_COLD;

	required_device<vt5x_cpu_device> m_maincpu;
	required_device<ay31015_device> m_uart;
	required_device<rs232_port_device> m_eia;
	required_ioport_array<8> m_keys;
	required_ioport m_break_key;
	required_ioport m_baud_sw;
	required_ioport m_data_sw;
	required_region_ptr<u8> m_chargen;

	bool m_serial_out;
	bool m_rec_data;
	u8 m_110_baud_counter;
};

void vt52_state::machine_start()
{
	save_item(NAME(m_serial_out));
	save_item(NAME(m_rec_data));
	save_item(NAME(m_110_baud_counter));
}

void vt52_state::machine_reset()
{
	m_110_baud_counter = 0;

	update_serial_settings();
	m_uart->write_swe(0);

	m_eia->write_dtr(0);
	m_eia->write_rts(0);
}

void vt52_state::update_serial_settings()
{
	u8 db = m_data_sw->read();
	m_uart->write_nb1(BIT(db, 0));
	m_uart->write_np(BIT(db, 0));
	m_uart->write_eps(BIT(db, 1));
	m_uart->write_nb2(1);
	m_uart->write_tsb(!BIT(m_baud_sw->read(), 10));
	m_uart->write_cs(1);

	gated_serial_output(m_serial_out && m_break_key->read());
	if (!BIT(m_baud_sw->read(), 9))
		m_uart->write_si(1);
}

INPUT_CHANGED_MEMBER(vt52_state::data_sw_changed)
{
	update_serial_settings();
}

u8 vt52_state::key_r(offs_t offset)
{
	// double negative logic courtesy of 7430 NOR gates and 74150 multiplexer
	return !BIT(~m_keys[offset & 7]->read() & 0x3ff, (offset & 0170) >> 3);
}

void vt52_state::baud_9600_w(int state)
{
	u16 baud = m_baud_sw->read();
	if (!BIT(baud, 13))
	{
		m_uart->write_rcp(state);
		if ((baud & 0x0380) != 0x0380)
			m_uart->write_tcp(state);
	}
}

void vt52_state::vert_count_w(u8 data)
{
	u16 baud = m_baud_sw->read();

	// 110 baud clock is 1200 baud clock divided by 11 by yet another 74161
	if ((data & 7) == 4)
	{
		if (m_110_baud_counter == 15)
		{
			if (!BIT(baud, 10))
			{
				m_uart->write_rcp(0);
				if ((baud & 0x0380) != 0x0380)
					m_uart->write_tcp(0);
			}
			m_110_baud_counter = 5;
		}
		else
		{
			m_110_baud_counter++;
			if (m_110_baud_counter == 8 && !BIT(baud, 10))
			{
				m_uart->write_rcp(1);
				if ((baud & 0x0380) != 0x0380)
					m_uart->write_tcp(1);
			}
		}
	}

	if ((baud & 0x2400) == 0x2400)
	{
		if ((baud & 0x1b80) != 0x1b80)
		{
			bool clk = (~(baud | data) & 0x7f) == 0;
			m_uart->write_rcp(clk);
			m_uart->write_tcp(clk);
		}
		else
		{
			m_uart->write_rcp((~(baud | data) & 0x0e) == 0);
			m_uart->write_tcp((~(baud | data) & 0x71) == 0);
		}
	}
	else if ((baud & 0x0380) == 0x0380)
		m_uart->write_tcp((~(baud | data) & 0x71) == 0);
}

void vt52_state::uart_xd_w(u8 data)
{
	if (BIT(m_data_sw->read(), 2))
		m_uart->transmit(data | 0x80);
	else
		m_uart->transmit(data & 0x7f);
}

void vt52_state::gated_serial_output(bool state)
{
	ioport_value baud = m_baud_sw->read();
	if (BIT(baud, 9))
		m_eia->write_txd(state);
	if (!BIT(baud, 9) || (m_rec_data && (~baud & 0x0880) != 0))
		m_uart->write_si(state);
}

void vt52_state::serial_out_w(int state)
{
	if (m_serial_out != state)
	{
		m_serial_out = state;
		if (m_break_key->read())
			gated_serial_output(state);
	}
}

void vt52_state::break_w(int state)
{
	if (m_serial_out)
		gated_serial_output(state);
}

void vt52_state::rec_data_w(int state)
{
	m_rec_data = state;

	if (machine().ioport().safe_to_read())
	{
		ioport_value baud = m_baud_sw->read();
		if (BIT(baud, 9) && ((~baud & 0x0880) == 0 || (m_serial_out && m_break_key->read())))
			m_uart->write_si(state);
	}
}

int vt52_state::xrdy_eoc_r()
{
	return m_uart->tbmt_r() && m_uart->eoc_r();
}

u8 vt52_state::chargen_r(offs_t offset)
{
	// ROM is on its own board, shared only with 7404 inverters
	return ~m_chargen[offset];
}

void vt52_state::rom_1k(address_map &map)
{
	map(00000, 01777).rom().region("program", 0);
}

void vt52_state::ram_2k(address_map &map)
{
	map(00000, 03777).ram();
}

static INPUT_PORTS_START(vt52)
	PORT_START("KEY0")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_UNUSED) // must always be low
	PORT_BIT(0x002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(2_PAD)) PORT_CODE(KEYCODE_2_PAD) // S78
	PORT_BIT(0x004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('2') PORT_CHAR('@') PORT_CODE(KEYCODE_2) // S3
	PORT_BIT(0x008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('w') PORT_CHAR('W') PORT_CHAR(027) PORT_CODE(KEYCODE_W) // S19
	PORT_BIT(0x010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('s') PORT_CHAR('S') PORT_CHAR(023) PORT_CODE(KEYCODE_S) // S35
	PORT_BIT(0x020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('c') PORT_CHAR('C') PORT_CHAR(003) PORT_CODE(KEYCODE_C) // S52
	PORT_BIT(0x040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('x') PORT_CHAR('X') PORT_CHAR(030) PORT_CODE(KEYCODE_X) // S51
	PORT_BIT(0x080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('z') PORT_CHAR('Z') PORT_CHAR(032) PORT_CODE(KEYCODE_Z) // S50
	PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(RIGHT)) PORT_CODE(KEYCODE_RIGHT) // S76
	PORT_BIT(0x200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(0_PAD)) PORT_CODE(KEYCODE_0_PAD) // S81

	PORT_START("KEY1")
	PORT_BIT(0x001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(7_PAD)) PORT_CODE(KEYCODE_7_PAD) // S69
	PORT_BIT(0x002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('l') PORT_CHAR('L') PORT_CHAR(014) PORT_CODE(KEYCODE_L) // S42
	PORT_BIT(0x004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(';') PORT_CHAR(':') PORT_CODE(KEYCODE_COLON) // S43
	PORT_BIT(0x008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('o') PORT_CHAR('O') PORT_CHAR(017) PORT_CODE(KEYCODE_O) // S26
	PORT_BIT(0x010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('9') PORT_CHAR('(') PORT_CODE(KEYCODE_9) // S10
	PORT_BIT(0x020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('p') PORT_CHAR('P') PORT_CHAR(020) PORT_CODE(KEYCODE_P) // S27
	PORT_BIT(0x040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Line Feed") PORT_CHAR(012) PORT_CODE(KEYCODE_INSERT) // S30
	PORT_BIT(0x080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('0') PORT_CHAR(')') PORT_CODE(KEYCODE_0) // S11
	PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(9_PAD)) PORT_CODE(KEYCODE_9_PAD) // S71
	PORT_BIT(0x200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD)) PORT_CODE(KEYCODE_ENTER_PAD) // S83

	PORT_START("KEY2")
	PORT_BIT(0x001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Delete") PORT_CHAR(UCHAR_MAMEKEY(DEL)) PORT_CODE(KEYCODE_DEL) // S31
	PORT_BIT(0x002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('5') PORT_CHAR('%') PORT_CODE(KEYCODE_5) // S6
	PORT_BIT(0x004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('t') PORT_CHAR('T') PORT_CHAR(024) PORT_CODE(KEYCODE_T) // S22
	PORT_BIT(0x008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('4') PORT_CHAR('$') PORT_CODE(KEYCODE_4) // S5
	PORT_BIT(0x010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('h') PORT_CHAR('H') PORT_CODE(KEYCODE_H) // S39
	PORT_BIT(0x020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('r') PORT_CHAR('R') PORT_CHAR(022) PORT_CODE(KEYCODE_R) // S21
	PORT_BIT(0x040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('\\') PORT_CHAR('|') PORT_CHAR(034) PORT_CODE(KEYCODE_BACKSLASH) // S29
	PORT_BIT(0x080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G  Bell") PORT_CHAR('g') PORT_CHAR('G') PORT_CHAR(007) PORT_CODE(KEYCODE_G) // S38
	PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(DOWN)) PORT_CODE(KEYCODE_DOWN) // S72
	PORT_BIT(0x200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(8_PAD)) PORT_CODE(KEYCODE_8_PAD) // S70

	PORT_START("KEY3")
	PORT_BIT(0x001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Back Space") PORT_CHAR(010) PORT_CODE(KEYCODE_BACKSPACE) // S15
	PORT_BIT(0x002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('3') PORT_CHAR('#') PORT_CODE(KEYCODE_3) // S4
	PORT_BIT(0x004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('f') PORT_CHAR('F') PORT_CHAR(006) PORT_CODE(KEYCODE_F) // S37
	PORT_BIT(0x008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('v') PORT_CHAR('V') PORT_CHAR(026) PORT_CODE(KEYCODE_V) // S53
	PORT_BIT(0x010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('b') PORT_CHAR('B') PORT_CHAR(002) PORT_CODE(KEYCODE_B) // S54
	PORT_BIT(0x020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('d') PORT_CHAR('D') PORT_CHAR(004) PORT_CODE(KEYCODE_D) // S36
	PORT_BIT(0x040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('=') PORT_CHAR('+') PORT_CHAR(035) PORT_CODE(KEYCODE_EQUALS) // S13
	PORT_BIT(0x080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('e') PORT_CHAR('E') PORT_CHAR(005) PORT_CODE(KEYCODE_E) // S20
	PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Keypad Blank (right)") PORT_CHAR(UCHAR_MAMEKEY(F3)) PORT_CODE(KEYCODE_ASTERISK) // S67
	PORT_BIT(0x200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Keypad Blank (left)") PORT_CHAR(UCHAR_MAMEKEY(F1)) PORT_CODE(KEYCODE_NUMLOCK) // S65

	PORT_START("KEY4")
	PORT_BIT(0x001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Keypad Blank (center)") PORT_CHAR(UCHAR_MAMEKEY(F2)) PORT_CODE(KEYCODE_SLASH_PAD) // S66
	PORT_BIT(0x002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('8') PORT_CHAR('*') PORT_CODE(KEYCODE_8) // S9
	PORT_BIT(0x004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('i') PORT_CHAR('I') PORT_CODE(KEYCODE_I) // S25
	PORT_BIT(0x008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('u') PORT_CHAR('U') PORT_CHAR(025) PORT_CODE(KEYCODE_U) // S24
	PORT_BIT(0x010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('7') PORT_CHAR('&') PORT_CODE(KEYCODE_7) // S8
	PORT_BIT(0x020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('6') PORT_CHAR('^') PORT_CODE(KEYCODE_6) // S7
	PORT_BIT(0x040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('`') PORT_CHAR('~') PORT_CODE(KEYCODE_TILDE) // S14
	PORT_BIT(0x080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('k') PORT_CHAR('K') PORT_CHAR(013) PORT_CODE(KEYCODE_K) // S41
	PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(UP)) PORT_CODE(KEYCODE_UP) // S68
	PORT_BIT(0x200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK)) PORT_CODE(KEYCODE_CAPSLOCK) PORT_TOGGLE // S33

	PORT_START("KEY5")
	PORT_BIT(0x001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(1_PAD)) PORT_CODE(KEYCODE_1_PAD) // S77
	PORT_BIT(0x002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Space Bar") PORT_CHAR(040) PORT_CODE(KEYCODE_SPACE) // S63
	PORT_BIT(0x004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('n') PORT_CHAR('N') PORT_CHAR(016) PORT_CODE(KEYCODE_N) // S55
	PORT_BIT(0x008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('m') PORT_CHAR('M') PORT_CODE(KEYCODE_M) // S56
	PORT_BIT(0x010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('j') PORT_CHAR('J') PORT_CODE(KEYCODE_J) // S40
	PORT_BIT(0x020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('y') PORT_CHAR('Y') PORT_CHAR(031) PORT_CODE(KEYCODE_Y) // S23
	PORT_BIT(0x040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('.') PORT_CHAR('>') PORT_CODE(KEYCODE_STOP) // S58
	PORT_BIT(0x080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(',') PORT_CHAR('<') PORT_CODE(KEYCODE_COMMA) // S57
	PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(3_PAD)) PORT_CODE(KEYCODE_3_PAD) // S79
	PORT_BIT(0x200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Repeat") PORT_CODE(KEYCODE_RALT) // S61

	PORT_START("KEY6")
	PORT_BIT(0x001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(4_PAD)) PORT_CODE(KEYCODE_4_PAD) // S73
	PORT_BIT(0x002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Scroll") PORT_CODE(KEYCODE_LALT) // S48
	PORT_BIT(0x004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Tab") PORT_CHAR(011) PORT_CODE(KEYCODE_TAB) // S17
	PORT_BIT(0x008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Esc (Sel)") PORT_CHAR(033) PORT_CODE(KEYCODE_ESC) // S1
	PORT_BIT(0x010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('a') PORT_CHAR('A') PORT_CHAR(001) PORT_CODE(KEYCODE_A) // S34
	PORT_BIT(0x020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('q') PORT_CHAR('Q') PORT_CHAR(021) PORT_CODE(KEYCODE_Q) // S18
	PORT_BIT(0x040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Return") PORT_CHAR(015) PORT_CODE(KEYCODE_ENTER) // S64/S47/S46
	PORT_BIT(0x080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('1') PORT_CHAR('!') PORT_CODE(KEYCODE_1) // S2
	PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(6_PAD)) PORT_CODE(KEYCODE_6_PAD) // S75
	PORT_BIT(0x200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Ctrl") PORT_CHAR(UCHAR_SHIFT_2) PORT_CODE(KEYCODE_LCONTROL) // S32

	PORT_START("KEY7")
	PORT_BIT(0x001, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(5_PAD)) PORT_CODE(KEYCODE_5_PAD) // S74
	PORT_BIT(0x002, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD)) PORT_CODE(KEYCODE_DEL_PAD) // S82
	PORT_BIT(0x004, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('{') PORT_CHAR('}') PORT_CODE(KEYCODE_CLOSEBRACE) // S45
	PORT_BIT(0x008, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('/') PORT_CHAR('?') PORT_CODE(KEYCODE_SLASH) // S59
	PORT_BIT(0x010, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('\'') PORT_CHAR('"') PORT_CODE(KEYCODE_QUOTE) // S44
	PORT_BIT(0x020, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('-') PORT_CHAR('_') PORT_CODE(KEYCODE_MINUS) // S12
	PORT_BIT(0x040, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Copy") PORT_CHAR(UCHAR_MAMEKEY(PRTSCR)) PORT_CODE(KEYCODE_RCONTROL) // S62
	PORT_BIT(0x080, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR('[') PORT_CHAR(']') PORT_CODE(KEYCODE_OPENBRACE) // S28
	PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CHAR(UCHAR_MAMEKEY(LEFT)) PORT_CODE(KEYCODE_LEFT) // S80
	PORT_BIT(0x200, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Shift") PORT_CHAR(UCHAR_SHIFT_1) PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) // S49(L)/S60(R)

	PORT_START("BREAK") // on keyboard but divorced from matrix (position taken over by Caps Lock) and not readable by CPU
	PORT_BIT(1, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Break") PORT_CODE(KEYCODE_PAUSE) PORT_WRITE_LINE_MEMBER(FUNC(vt52_state::break_w)) // S16

	PORT_START("BAUD") // 7-position rotary switches under keyboard, set in combination (positions on SW2 are actually labeled A through G)
	PORT_DIPNAME(0x03f1, 0x01f1, "Transmitting Speed") PORT_DIPLOCATION("S1:7,4,5,6,2,3,1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt52_state::data_sw_changed), 0)
	PORT_DIPSETTING(0x01f1, "Off-Line (XCLK = RCLK)") // S1:1
	PORT_DIPSETTING(0x02f1, "Full Duplex (XCLK = RCLK)") // S1:3
	PORT_DIPSETTING(0x0371, "Full Duplex, Local Copy (XCLK = RCLK)") // S1:2
	PORT_DIPSETTING(0x03b1, "75 Baud") // S1:6
	PORT_DIPSETTING(0x03d1, "150 Baud") // S1:5
	PORT_DIPSETTING(0x03e1, "300 Baud") // S1:4
	PORT_DIPSETTING(0x03f0, "4800 Baud") // S1:7
	PORT_DIPNAME(0x3c0e, 0x1c0e, "Receiving Speed") PORT_DIPLOCATION("S2:6,5,4,2,1,3,7") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt52_state::data_sw_changed), 0)
	PORT_DIPSETTING(0x2c0e, "Match (Bell 103) (RCLK = XCLK)") // S2:C
	PORT_DIPSETTING(0x340e, "Match (Bell 103), Local Copy (RCLK = XCLK)") // S2:A
	PORT_DIPSETTING(0x380e, "110 Baud with 2 Stop Bits") // S2:B
	PORT_DIPSETTING(0x3c06, "600 Baud") // S2:D
	PORT_DIPSETTING(0x3c0a, "1200 Baud") // S2:E
	PORT_DIPSETTING(0x3c0c, "2400 Baud") // S2:F
	PORT_DIPSETTING(0x1c0e, "9600 Baud") // S2:G
	// Any combination of XCLK = RCLK with RCLK = XCLK is illegal (both lines are pulled up, halting the UART)

	PORT_START("DATABITS")
	PORT_DIPNAME(0x1, 0x1, "Data Bits") PORT_DIPLOCATION("S3:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt52_state::data_sw_changed), 0)
	PORT_DIPSETTING(0x0, "7 (with parity)")
	PORT_DIPSETTING(0x1, "8 (no parity)")
	PORT_DIPNAME(0x2, 0x2, "Parity") PORT_DIPLOCATION("W6:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt52_state::data_sw_changed), 0)
	PORT_DIPSETTING(0x2, "Even")
	PORT_DIPSETTING(0x0, "Odd")
	PORT_DIPNAME(0x4, 0x0, "Data Bit 7") PORT_DIPLOCATION("W5:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt52_state::data_sw_changed), 0)
	PORT_DIPSETTING(0x0, "Spacing")
	PORT_DIPSETTING(0x4, "Marking") // actually the hardware default, but not as good for modern use

	PORT_START("KEYCLICK")
	PORT_DIPNAME(1, 1, DEF_STR(Unused)) PORT_DIPLOCATION("S4:1") // not tested by VT52, and possibly not even populated
	PORT_DIPSETTING(0, DEF_STR(Off))
	PORT_DIPSETTING(1, DEF_STR(On))

	PORT_START("60HJ")
	PORT_DIPNAME(1, 1, "Unit Frequency") PORT_DIPLOCATION("W7:1")
	PORT_DIPSETTING(0, "50 Hz")
	PORT_DIPSETTING(1, "60 Hz")
INPUT_PORTS_END

void vt52_state::vt52(machine_config &config)
{
	VT52_CPU(config, m_maincpu, 13.824_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &vt52_state::rom_1k);
	m_maincpu->set_addrmap(AS_DATA, &vt52_state::ram_2k);
	m_maincpu->set_screen("screen");
	m_maincpu->baud_9600_callback().set(FUNC(vt52_state::baud_9600_w));
	m_maincpu->vert_count_callback().set(FUNC(vt52_state::vert_count_w));
	m_maincpu->uart_rd_callback().set(m_uart, FUNC(ay51013_device::receive));
	m_maincpu->uart_xd_callback().set(FUNC(vt52_state::uart_xd_w));
	m_maincpu->ur_flag_callback().set(m_uart, FUNC(ay51013_device::dav_r));
	m_maincpu->ut_flag_callback().set(FUNC(vt52_state::xrdy_eoc_r));
	m_maincpu->ruf_callback().set(m_uart, FUNC(ay51013_device::write_rdav));
	m_maincpu->key_up_callback().set(FUNC(vt52_state::key_r));
	m_maincpu->kclk_callback().set_ioport("KEYCLICK");
	m_maincpu->frq_callback().set_ioport("60HJ");
	m_maincpu->bell_callback().set("bell", FUNC(speaker_sound_device::level_w));
	m_maincpu->char_data_callback().set(FUNC(vt52_state::chargen_r));

	AY51013(config, m_uart); // TR1402 or equivalent
	m_uart->write_so_callback().set(FUNC(vt52_state::serial_out_w));

	SCREEN(config, "screen");

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, "bell").add_route(ALL_OUTPUTS, "mono", 1.0); // FIXME: uses a flyback diode circuit

	RS232_PORT(config, m_eia, default_rs232_devices, nullptr);
	m_eia->rxd_handler().set(FUNC(vt52_state::rec_data_w));
}

ROM_START(vt52)
	ROM_REGION(0x400, "program", 0) // bipolar PROMs
	ROM_LOAD_NIB_LOW( "23-124a9.e29", 0x000, 0x200, CRC(3f5f3b92) SHA1(244c3100f277da3fce5513a92529a2c3e26a80b4))
	ROM_LOAD_NIB_HIGH("23-125a9.e26", 0x000, 0x200, CRC(b2a670c9) SHA1(fa8dd031dcafe4facff41e79603bdb388a6df928))
	ROM_LOAD_NIB_LOW( "23-126a9.e37", 0x200, 0x200, CRC(4883a600) SHA1(c5d9b0c21493065c75b4a7d52d5bd47f9851dfe7))
	ROM_LOAD_NIB_HIGH("23-127a9.e21", 0x200, 0x200, CRC(56c1c0d6) SHA1(ab0eb6e7bbafcc3d28481b62de3d3490f01c0174))
	// K1 or L1 version uses PROMs 23-119A9 through 23-122A9

	ROM_REGION(0x400, "chargen", 0) // 2608 (non-JEDEC) character generator
	ROM_LOAD("23-002b4.e1", 0x000, 0x400, CRC(b486500c) SHA1(029f07424d6c23ee083db42d9f9c252ac728ccd0))
	// K1 or L1 version may use either 23-001B4 or 23-002B4
ROM_END


/****************************************************************************

    DEC VT50 (basic model)

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

****************************************************************************/

class vt50_state : public driver_device
{
public:
	enum : u8
	{
		MODE_OFF_LINE = 0,
		MODE_LOCAL_COPY,
		MODE_FULL_DUPLEX,
		MODE_300,
		MODE_150,
		MODE_75
	};

	enum : u8
	{
		SPEED_BELL_103 = 0,
		SPEED_110,
		SPEED_600,
		SPEED_1200,
		SPEED_2400,
		SPEED_4800,
		SPEED_9600
	};

	vt50_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_uart(*this, "uart")
		, m_eia(*this, "eia")
		, m_keys(*this, "KEY%d", 0U)
		, m_break_key(*this, "BREAK")
		, m_mode_sw(*this, "MODE")
		, m_speed_sw(*this, "SPEED")
		, m_data_sw(*this, "DATABITS")
		, m_chargen(*this, "chargen")
		, m_serial_out(true)
		, m_rec_data(true)
		, m_9600_clock(false)
		, m_baud_divider(0)
		, m_110_baud_counter(0)
	{
	}

	void vt50(machine_config &config);

	void break_w(int state);
	DECLARE_INPUT_CHANGED_MEMBER(serial_sw_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void update_serial_settings();
	void update_uart_clocks();
	void gated_serial_output();
	void update_serial_in();

	u8 key_r(offs_t offset);
	void baud_9600_w(int state);
	void uart_xd_w(u8 data);
	void serial_out_w(int state);
	void rec_data_w(int state);
	int xrdy_eoc_r();
	u8 chargen_r(offs_t offset);

	void rom_1k(address_map &map) ATTR_COLD;
	void ram_1k(address_map &map) ATTR_COLD;

	required_device<vt50_cpu_device> m_maincpu;
	required_device<ay31015_device> m_uart;
	required_device<rs232_port_device> m_eia;
	required_ioport_array<7> m_keys;
	required_ioport m_break_key;
	required_ioport m_mode_sw;
	required_ioport m_speed_sw;
	required_ioport m_data_sw;
	required_region_ptr<u8> m_chargen;

	bool m_serial_out;
	bool m_rec_data;
	bool m_9600_clock;
	u8 m_baud_divider;
	u8 m_110_baud_counter;
};

void vt50_state::machine_start()
{
	save_item(NAME(m_serial_out));
	save_item(NAME(m_rec_data));
	save_item(NAME(m_9600_clock));
	save_item(NAME(m_baud_divider));
	save_item(NAME(m_110_baud_counter));
}

void vt50_state::machine_reset()
{
	m_baud_divider = 0;
	m_110_baud_counter = 0;

	update_serial_settings();
	m_uart->write_swe(0);

	m_eia->write_dtr(0);
	m_eia->write_rts(0);
}

void vt50_state::update_serial_settings()
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

INPUT_CHANGED_MEMBER(vt50_state::serial_sw_changed)
{
	update_serial_settings();
}

u8 vt50_state::key_r(offs_t offset)
{
	// a 7442 decodes AC0-AC2 into the matrix lines, and a 74151 picks one of seven key groups with AC3-AC5 (its eighth input is grounded)
	const unsigned group = (offset >> 3) & 7;
	return group == 7 ? 1 : BIT(m_keys[group]->read(), offset & 7);
}

void vt50_state::baud_9600_w(int state)
{
	if (m_9600_clock == bool(state))
		return;
	m_9600_clock = state;

	// two 74197s ripple-divide the 9600 baud clock down to 75 baud, and a 74161 reloaded with 5 divides 1200 baud by 11 for 110 baud
	if (!state)
	{
		const u8 old = m_baud_divider;
		m_baud_divider = (m_baud_divider + 1) & 0177;
		if (!BIT(old, 2) && BIT(m_baud_divider, 2))
			m_110_baud_counter = m_110_baud_counter == 15 ? 5 : m_110_baud_counter + 1;
	}

	update_uart_clocks();
}

void vt50_state::update_uart_clocks()
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

void vt50_state::uart_xd_w(u8 data)
{
	if (BIT(m_data_sw->read(), 2))
		m_uart->transmit(data | 0x80);
	else
		m_uart->transmit(data & 0x7f);
}

void vt50_state::gated_serial_output()
{
	// BREAK and off-line handling copy the VT52 driver; the VT50's own E45/E21 gating was not traced gate by gate
	if (m_mode_sw->read() != MODE_OFF_LINE)
		m_eia->write_txd(m_serial_out && m_break_key->read());
	update_serial_in();
}

void vt50_state::update_serial_in()
{
	// off-line and with local copy the transmitted data is also fed to the receiver, which ignores the line when off-line
	const u8 mode = m_mode_sw->read();
	bool si = mode == MODE_OFF_LINE || m_rec_data;
	if (mode == MODE_OFF_LINE || mode == MODE_LOCAL_COPY)
		si = si && m_serial_out && m_break_key->read();
	m_uart->write_si(si);
}

void vt50_state::serial_out_w(int state)
{
	if (m_serial_out != bool(state))
	{
		m_serial_out = state;
		gated_serial_output();
	}
}

void vt50_state::break_w(int state)
{
	gated_serial_output();
}

void vt50_state::rec_data_w(int state)
{
	m_rec_data = state;

	if (machine().ioport().safe_to_read())
		update_serial_in();
}

int vt50_state::xrdy_eoc_r()
{
	return m_uart->tbmt_r() && m_uart->eoc_r();
}

u8 vt50_state::chargen_r(offs_t offset)
{
	// The 2513's outputs 1 to 5 are inverted onto CD4 down to CD0, and two 7495s shift out a blank, CD0 to CD4, then two
	// more blanks, so output 5 (bit 4 of the dump) is the leftmost dot. RAM bit 6 does not reach the 2513.
	return (~m_chargen[offset & 0777] & 037) << 2 | 3;
}

void vt50_state::rom_1k(address_map &map)
{
	map(00000, 01777).rom().region("program", 0);
}

void vt50_state::ram_1k(address_map &map)
{
	map(00000, 01777).ram(); // seven 2102s
}

static INPUT_PORTS_START(vt50)
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
	PORT_BIT(1, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Break") PORT_CODE(KEYCODE_PAUSE) PORT_WRITE_LINE_MEMBER(FUNC(vt50_state::break_w))

	PORT_START("MODE") // rotary switch S1 under the keyboard, positions as named on the base decal
	PORT_DIPNAME(0x7, vt50_state::MODE_OFF_LINE, "Mode") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt50_state::serial_sw_changed), 0)
	PORT_DIPSETTING(vt50_state::MODE_OFF_LINE, "Off-Line") // S1:1
	PORT_DIPSETTING(vt50_state::MODE_LOCAL_COPY, "Full Duplex with Local Copy") // S1:2
	PORT_DIPSETTING(vt50_state::MODE_FULL_DUPLEX, "Full Duplex") // S1:3
	PORT_DIPSETTING(vt50_state::MODE_300, "Full Duplex, Transmit 300 Baud") // S1:4
	PORT_DIPSETTING(vt50_state::MODE_150, "Full Duplex, Transmit 150 Baud") // S1:5
	PORT_DIPSETTING(vt50_state::MODE_75, "Full Duplex, Transmit 75 Baud") // S1:6

	PORT_START("SPEED") // rotary switch S2 under the keyboard
	PORT_DIPNAME(0x7, vt50_state::SPEED_9600, "Speed") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt50_state::serial_sw_changed), 0)
	PORT_DIPSETTING(vt50_state::SPEED_BELL_103, "Bell 103 (Receive at Transmit Speed)") // S2:A
	PORT_DIPSETTING(vt50_state::SPEED_110, "110 Baud with 2 Stop Bits") // S2:B
	PORT_DIPSETTING(vt50_state::SPEED_600, "600 Baud") // S2:C
	PORT_DIPSETTING(vt50_state::SPEED_1200, "1200 Baud") // S2:D
	PORT_DIPSETTING(vt50_state::SPEED_2400, "2400 Baud") // S2:E
	PORT_DIPSETTING(vt50_state::SPEED_4800, "4800 Baud") // S2:F
	PORT_DIPSETTING(vt50_state::SPEED_9600, "9600 Baud") // S2:G
	// Bell 103 with S1 at 1 to 3 is illegal (both UART clock lines are pulled up, halting the UART)

	PORT_START("DATABITS")
	PORT_DIPNAME(0x1, 0x1, "Parity") PORT_DIPLOCATION("S4:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt50_state::serial_sw_changed), 0)
	PORT_DIPSETTING(0x0, "Even (7 Data Bits)")
	PORT_DIPSETTING(0x1, "None (8 Data Bits)")
	PORT_DIPNAME(0x2, 0x2, "Parity Sense") PORT_DIPLOCATION("W6:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt50_state::serial_sw_changed), 0)
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

void vt50_state::vt50(machine_config &config)
{
	VT50_CPU(config, m_maincpu, 13.824_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &vt50_state::rom_1k);
	m_maincpu->set_addrmap(AS_DATA, &vt50_state::ram_1k);
	m_maincpu->set_screen("screen");
	m_maincpu->baud_9600_callback().set(FUNC(vt50_state::baud_9600_w));
	m_maincpu->uart_rd_callback().set(m_uart, FUNC(ay51013_device::receive));
	m_maincpu->uart_xd_callback().set(FUNC(vt50_state::uart_xd_w));
	m_maincpu->ur_flag_callback().set(m_uart, FUNC(ay51013_device::dav_r));
	m_maincpu->ut_flag_callback().set(FUNC(vt50_state::xrdy_eoc_r));
	m_maincpu->ruf_callback().set(m_uart, FUNC(ay51013_device::write_rdav));
	m_maincpu->key_up_callback().set(FUNC(vt50_state::key_r));
	m_maincpu->kclk_callback().set_ioport("KEYCLICK");
	m_maincpu->frq_callback().set_ioport("60HJ");
	m_maincpu->bell_callback().set("bell", FUNC(speaker_sound_device::level_w));
	m_maincpu->char_data_callback().set(FUNC(vt50_state::chargen_r));
	m_maincpu->ctrl_key_callback().set_ioport("CTRL");

	AY51013(config, m_uart); // 1402 at E7
	m_uart->write_so_callback().set(FUNC(vt50_state::serial_out_w));

	SCREEN(config, "screen");

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, "bell").add_route(ALL_OUTPUTS, "mono", 1.0);

	RS232_PORT(config, m_eia, default_rs232_devices, nullptr);
	m_eia->rxd_handler().set(FUNC(vt50_state::rec_data_w));
}

ROM_START(vt50)
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

} // anonymous namespace


COMP(1974, vt50, 0, 0, vt50, vt50, vt50_state, empty_init, "Digital Equipment Corporation", "VT50 DECscope", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND)
COMP(1975, vt52, 0, 0, vt52, vt52, vt52_state, empty_init, "Digital Equipment Corporation", "VT52 Video Display Terminal (M4)", MACHINE_SUPPORTS_SAVE | MACHINE_IMPERFECT_SOUND | MACHINE_NODEVICE_PRINTER)
