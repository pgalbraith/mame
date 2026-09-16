// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Lear Siegler ADM-3A Dumb Terminal

  Twenty-four lines of eighty characters on a 12" CRT, 75 to 19200 baud, and
  almost nothing else: Lear Siegler called it the Dumb Terminal on purpose and
  put that on the case.  It sold in enormous numbers from 1976 because it was
  cheap, and it became the terminal everything else was measured against - vi
  still moves the cursor with the ADM-3A's CTRL H, J, K and L, and termcap
  still describes cursor addressing the ADM-3A's way.

  WHAT IS MODELLED, AND WHY IT LOOKS NOTHING LIKE THE OTHER TERMINALS HERE
  -----------------------------------------------------------------------
  There is no CPU.  The whole terminal is TTL on one board: a string of
  counters generates the raster, a UART moves characters in and out, and a
  handful of decoders turn the fourteen control codes it understands into
  directives for three counters - a row counter, a column counter and an
  offset counter.  So this is a behavioural model of those counters and
  decoders, written from the ADM 3A/3A+ Maintenance Manual (DP3050683F, June
  1983), sections 4.3.2 through 4.3.12 and 6.3, not a gate-level one.

  The three counters are kept rather than flattened because scrolling falls
  out of how they are combined.  The row counter holds the cursor's apparent
  row on the screen, the offset counter holds how far the display has slid
  away from the start of memory, and the two are added to reach a physical
  row.  A line feed on the bottom row increments the offset instead of the
  row, which scrolls the whole page and hands back one row of memory to be
  erased - which is exactly what ERASEF does on the real board (4.3.8).

  The one piece of real silicon that survives is the character generator: a
  stock RO-3-2513 at L15, with the matching lower case part at L14 when the
  option is fitted.

  WHAT IS NOT MODELLED
  --------------------
  The EXTENSION port, the answerback PROM, the numeric keypad, and the
  modem turnaround modes (S5-1 through S5-6, RTS/CTS/DCD and the secondary
  channel).  Nothing on the MAME side would connect to them, and the
  turnaround modes only matter against a real 202-type modem.  The 20 mA
  current loop is not modelled separately either: it carries the same bits as
  the RS-232 interface and only the electrical levels differ.

****************************************************************************/

#include "emu.h"

#include "adm3a.h"

#include "machine/keyboard.ipp"


namespace {

// The oscillator on the main board.  Seven dots per character, 96 character
// times per scan line and nine scan lines per character row give the manual's
// quoted 16.2 kHz horizontal and a 60 Hz frame of 30 character rows (figure
// 4-4).
static constexpr XTAL ADM3A_DOT_CLOCK = 10.8864_MHz_XTAL;

static constexpr int ADM3A_DOTS_PER_CHAR  = 7;
static constexpr int ADM3A_CHARS_PER_LINE = 96;
static constexpr int ADM3A_COLUMNS        = 80;
static constexpr int ADM3A_SCANS_PER_ROW  = 9;
static constexpr int ADM3A_ROWS           = 24;

// Character rows in a whole frame, visible ones included: six of vertical
// retrace on a 60 Hz line, twelve on a 50 Hz one.
static constexpr int ADM3A_ROWS_60HZ = 30;
static constexpr int ADM3A_ROWS_50HZ = 36;

static constexpr int ADM3A_HTOTAL         = ADM3A_CHARS_PER_LINE * ADM3A_DOTS_PER_CHAR;
static constexpr int ADM3A_VISIBLE_WIDTH  = ADM3A_COLUMNS * ADM3A_DOTS_PER_CHAR;
static constexpr int ADM3A_VISIBLE_HEIGHT = ADM3A_ROWS * ADM3A_SCANS_PER_ROW;

// The beeper is the character row rate gated by a one-shot.  The manual does
// not give the one-shot's period, but it says the beeper would sound
// continuously above 2400 baud because the next column 72 would arrive first
// (4.3.11), and that puts it between one line time at 4800 baud (167 ms) and
// one at 2400 baud (333 ms).
static constexpr int ADM3A_BELL_HZ = 3600;
static constexpr int ADM3A_BELL_MS = 200;

// Column the cursor has to reach for the near-end-of-line warning.  S7 calls
// it the 72 column beep, which is this count one-based.
static constexpr int ADM3A_BEEP_COLUMN = 71;

// Control codes the command decoder recognises (figure 4-12 and the quick
// reference chart in appendix E).  Everything else below 040 is thrown away.
enum : u8
{
	ADM3A_ENQ = 0x05,   // HERE IS, answerback
	ADM3A_BEL = 0x07,
	ADM3A_BS  = 0x08,   // backspace
	ADM3A_LF  = 0x0a,   // downline
	ADM3A_VT  = 0x0b,   // upline
	ADM3A_FF  = 0x0c,   // forespace
	ADM3A_CR  = 0x0d,
	ADM3A_SO  = 0x0e,   // unlock keyboard, or enable the extension port
	ADM3A_SI  = 0x0f,   // lock keyboard, or disable the extension port
	ADM3A_SUB = 0x1a,   // clear screen
	ADM3A_ESC = 0x1b,   // first of the four character load cursor sequence
	ADM3A_RS  = 0x1e    // home
};

} // anonymous namespace


DEFINE_DEVICE_TYPE(ADM3A, adm3a_device, "adm3a", "Lear Siegler ADM-3A Dumb Terminal")


adm3a_device::adm3a_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ADM3A, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, device_matrix_keyboard_interface(mconfig, *this, "X0", "X1", "X2", "X3", "X4")
	, m_screen(*this, "screen")
	, m_palette(*this, "palette")
	, m_bell(*this, "bell")
	, m_chargen(*this, "chargen")
	, m_modifiers(*this, "MODIFIERS")
	, m_baud(*this, "BAUD")
	, m_switches(*this, "SWITCHES")
	, m_options(*this, "OPTIONS")
	, m_write_sd(*this)
{
	std::fill(std::begin(m_ram), std::end(m_ram), ' ');
}


/***************************************************************************
    CONFIGURATION SWITCHES

    S1 through S8 are DIP switches on the main logic board, listed in tables
    2-1 and 2-2.  They are read live rather than latched at reset, so changing
    one from the Machine Configuration menu behaves the way reaching into the
    terminal and moving it does.
***************************************************************************/

bool adm3a_device::half_duplex() const          { return BIT(m_switches->read(), 0); }
bool adm3a_device::auto_new_line() const        { return BIT(m_switches->read(), 1); }
bool adm3a_device::lower_case_enabled() const   { return BIT(m_switches->read(), 2); }
bool adm3a_device::clear_screen_enabled() const { return BIT(m_switches->read(), 9); }
bool adm3a_device::keyboard_lock_enabled() const{ return BIT(m_switches->read(), 10); }
bool adm3a_device::destructive_space() const    { return BIT(m_switches->read(), 12); }
bool adm3a_device::cursor_control() const       { return BIT(m_switches->read(), 13); }
bool adm3a_device::twelve_line() const          { return BIT(m_switches->read(), 7); }
bool adm3a_device::fifty_hertz() const          { return BIT(m_switches->read(), 8); }
bool adm3a_device::column_beep() const          { return BIT(m_options->read(), 1); }
bool adm3a_device::gated_extension() const      { return BIT(m_options->read(), 2); }
bool adm3a_device::lower_case_fitted() const    { return BIT(m_options->read(), 0); }

bool adm3a_device::lower_case_displayed() const
{
	// Three things have to line up before a lower case letter reaches the
	// screen as one: the option board has to be there, S3-1 has to let the
	// keyboard generate lower case codes, and S4-5 has to switch the second
	// character ROM in.
	return lower_case_fitted() && lower_case_enabled() && BIT(m_switches->read(), 11);
}

u32 adm3a_device::baud_rate() const
{
	if (m_preset_baud)
		return m_preset_baud;

	static const u32 RATE[12] =
	{
		75, 110, 150, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 9600
	};

	return RATE[m_baud->read() & 0x0f];
}


/***************************************************************************
    THE ROW, OFFSET AND COLUMN COUNTERS

    In 12-line mode both counters have their least significant bit held, so
    the row counter only ever holds an odd row and the offset only ever an
    even one; their sum, the physical row, stays odd and the even rows of the
    display are blanked (4.3.3).
***************************************************************************/

void adm3a_device::erase_row(u8 row)
{
	for (int col = 0; col < ADM3A_COLUMNS; col++)
		ram(row, col) = ' ';
}

void adm3a_device::clear_screen()
{
	// START walks the offset counter right round the page writing spaces, and
	// leaves both counters at minimum count with the true and virtual row
	// addresses the same (4.3.3).
	std::fill(std::begin(m_ram), std::end(m_ram), ' ');

	m_offset = 0;
	m_col = 0;
	m_no_write = false;
	m_esc_state = 0;

	// With cursor control off the row counter is not held at minimum: it keeps
	// taking line feeds until it reaches 23 and stops there, which is what
	// pins the cursor to the bottom line.
	m_row = cursor_control() ? (twelve_line() ? 1 : 0) : (ADM3A_ROWS - 1);
}

void adm3a_device::down_line()
{
	const u8 step = twelve_line() ? 2 : 1;

	if (cursor_control() && ((m_row + step) <= (ADM3A_ROWS - 1)))
	{
		m_row += step;
	}
	else
	{
		// Scroll: the offset counter takes the line feed instead, and the row
		// of memory that has just come round to the bottom of the screen is
		// erased behind it.
		m_offset = (m_offset + step) % ADM3A_ROWS;
		erase_row(entry_row());
	}

	// A line feed ends the window in which a space code does not erase.
	m_no_write = false;
}

void adm3a_device::up_line()
{
	if (!cursor_control())
		return;

	const u8 step = twelve_line() ? 2 : 1;
	const u8 floor = twelve_line() ? 1 : 0;

	// Decrementing past minimum count borrows, and the borrow clears the
	// counter rather than wrapping it.
	m_row = (m_row >= (floor + step)) ? (m_row - step) : floor;
}

void adm3a_device::carriage_return()
{
	m_col = 0;

	// SPACE-ADV: with S4-6 open the space code stops erasing what it passes
	// over until the next line feed, so a program can write a line, return and
	// then space the cursor back across it without wiping it out (4.3.9).
	if (!destructive_space())
		m_no_write = true;
}

void adm3a_device::home_cursor()
{
	m_col = 0;

	if (cursor_control())
		m_row = twelve_line() ? 1 : 0;
}

void adm3a_device::backspace()
{
	// Underflow from column 0 clears the counter rather than wrapping it, so
	// backspacing off the left of the line just stops there.
	if (m_col)
		m_col--;
}

void adm3a_device::forespace()
{
	if (m_col < (ADM3A_COLUMNS - 1))
	{
		m_col++;

		if ((m_col == ADM3A_BEEP_COLUMN) && column_beep())
			ring_bell();

		return;
	}

	// Overflow past column 79 loads either 79 again or, with AUTO NL closed,
	// column 0 plus a line feed.
	if (auto_new_line())
	{
		m_col = 0;
		down_line();
	}
}

void adm3a_device::write_char(u8 data)
{
	// The one code that can be non-destructive is space, between a carriage
	// return and the line feed that follows it.
	if ((data != ' ') || !m_no_write)
		ram(entry_row(), m_col) = data;

	forespace();
}


/***************************************************************************
    LOAD CURSOR

    ESC = row column, four characters in all.  None of the three that follow
    ESC produces a forespace (4.3.4).
***************************************************************************/

void adm3a_device::load_cursor_row(u8 data)
{
	if (!cursor_control())
		return;

	// Only DATA1 through DATA5 reach the row counter, and a value above 23
	// clears it to minimum count (6.3.3).
	u8 row = data & 0x1f;

	if (row > (ADM3A_ROWS - 1))
		row = 0;

	m_row = twelve_line() ? (row | 1) : row;
}

void adm3a_device::load_cursor_column(u8 data)
{
	if (!cursor_control())
		return;

	// The two most significant bits are modified so that codes 040 through
	// 157 octal - space through lower case o - become columns 0 through 79.
	const u8 col = (data - 0x20) & 0x7f;

	m_col = (col < ADM3A_COLUMNS) ? col : (auto_new_line() ? 0 : (ADM3A_COLUMNS - 1));
}


/***************************************************************************
    THE COMMAND DECODER
***************************************************************************/

void adm3a_device::ring_bell()
{
	m_bell->set_state(1);
	m_bell_timer->adjust(attotime::from_msec(ADM3A_BELL_MS));
}

TIMER_CALLBACK_MEMBER(adm3a_device::bell_off)
{
	m_bell->set_state(0);
}

void adm3a_device::handle_char(u8 data)
{
	// The UART presents seven bits to the latches; IN8 is not connected.
	data &= 0x7f;

	switch (m_esc_state)
	{
	case 1:
		// Anything but = after ESC is not a load cursor, and is swallowed
		// along with the ESC.
		m_esc_state = (data == '=') ? 2 : 0;
		return;

	case 2:
		load_cursor_row(data);
		m_esc_state = 3;
		return;

	case 3:
		load_cursor_column(data);
		m_esc_state = 0;
		return;

	default:
		break;
	}

	switch (data)
	{
	case ADM3A_ENQ:
		// Answerback: the identification message lives in a PROM on an option
		// board that is not fitted here, so nothing is sent.
		break;

	case ADM3A_BEL:
		ring_bell();
		break;

	case ADM3A_BS:
		backspace();
		break;

	case ADM3A_LF:
		down_line();
		break;

	case ADM3A_VT:
		up_line();
		break;

	case ADM3A_FF:
		forespace();
		break;

	case ADM3A_CR:
		carriage_return();
		break;

	case ADM3A_SO:
	case ADM3A_SI:
		// With S8 in the GT position these two gate the EXTENSION port, which
		// is not modelled, and the keyboard is left alone either way.
		if (!gated_extension() && keyboard_lock_enabled())
			m_keyboard_locked = (data == ADM3A_SI);
		break;

	case ADM3A_SUB:
		if (clear_screen_enabled())
			clear_screen();
		break;

	case ADM3A_ESC:
		m_esc_state = 1;
		break;

	case ADM3A_RS:
		home_cursor();
		break;

	default:
		// Control characters are not displayed when they are generated, and
		// rubout is one of them.
		if ((data >= 0x20) && (data < 0x7f))
			write_char(data);
		break;
	}
}


/***************************************************************************
    SERIAL PORT
***************************************************************************/

void adm3a_device::serial_in_w(int state)
{
	rx_w(state);
}

void adm3a_device::rcv_complete()
{
	receive_register_extract();

	handle_char(get_received_char());
}

void adm3a_device::send_char(u8 data)
{
	// The UART has one holding register behind the serialiser and that is all
	// the buffering there is.
	if (m_tx_busy || !is_transmit_register_empty())
	{
		if (!m_tx_full)
		{
			m_tx_char = data;
			m_tx_full = true;
		}

		return;
	}

	m_tx_busy = true;
	transmit_register_setup(data);
}

void adm3a_device::tra_callback()
{
	// BREAK holds the transmit data line in the spacing state under whatever
	// the serialiser is doing.
	m_write_sd(m_break ? 0 : transmit_register_get_data_bit());
}

void adm3a_device::tra_complete()
{
	if (m_tx_full)
	{
		m_tx_full = false;
		transmit_register_setup(m_tx_char);
		return;
	}

	m_tx_busy = false;
	m_write_sd(m_break ? 0 : 1);
}

INPUT_CHANGED_MEMBER(adm3a_device::break_key)
{
	// A real spacing condition on the transmit line, not an ASCII character,
	// held for as long as the key is down.
	m_break = bool(newval);

	if (m_break)
		m_write_sd(0);
	else if (!m_tx_busy)
		m_write_sd(1);
}

INPUT_CHANGED_MEMBER(adm3a_device::frame_changed)
{
	update_frame();
}

INPUT_CHANGED_MEMBER(adm3a_device::refresh_changed)
{
	update_screen_params();
}

INPUT_CHANGED_MEMBER(adm3a_device::clear_key)
{
	// SHIFT CLEAR is a local clear: it erases the page and resets the keyboard
	// lock, and sends nothing (4.3.10).
	if (!newval)
		return;

	if (!BIT(m_modifiers->read(), 0) && !BIT(m_modifiers->read(), 1))
		return;

	m_keyboard_locked = false;
	clear_screen();
}

void adm3a_device::update_frame()
{
	const ioport_value sw = m_switches->read();

	set_data_frame(1, BIT(sw, 4) ? 8 : 7,
			BIT(sw, 6) ? (BIT(sw, 3) ? PARITY_ODD : PARITY_EVEN) : PARITY_NONE,
			BIT(sw, 5) ? STOP_BITS_2 : STOP_BITS_1);
	set_rate(baud_rate());
}


/***************************************************************************
    KEYBOARD

    There is no key matrix in the usual sense.  A counter sweeps the whole
    7-bit USASCII chart at the character rate and stops when the code it is
    generating matches the key that is held down, so a key's position in the
    chart is the code it makes (4.3.10).  That is what the tables below hold:
    one base code per key, with SHIFT and CTRL doing to it exactly what the
    invert and force gates do on the board.
***************************************************************************/

u8 adm3a_device::shifted(u8 code, bool shift) const
{
	if (code >= 0x60)
	{
		// The alpha keys sit in columns 6 and 7, so SHIFT inverting bit 6
		// walks them up to columns 4 and 5.  With the lower case option
		// switched out INVERT6 is held on and they are upper case whatever
		// SHIFT does.
		if (!lower_case_enabled())
			return code ^ 0x20;

		return shift ? (code ^ 0x20) : code;
	}

	if (code >= 0x40)
		return shift ? (code ^ 0x20) : code;   // @ [ \ ] shift up to ` { | }

	// Columns 2 and 3, where SHIFT inverts bit 5 instead.  Row 0 of those two
	// columns is space and zero, which have nothing printed above them and are
	// excluded from the inversion.
	if (shift && ((code & 0x0f) != 0))
		return code ^ 0x10;

	return code;
}

void adm3a_device::key_char(u8 data)
{
	send_char(data);

	// Half duplex ORs what the terminal is transmitting back into its own
	// receiver, which is how a typed character reaches the screen without the
	// host echoing it.
	if (half_duplex())
		handle_char(data);
}

void adm3a_device::key_make(u8 row, u8 column)
{
	// One base code per key, in the order the keys sit on the keyboard.  The
	// alpha keys are held at their lower case codes because that is where
	// their switches are in the chart.
	static const u8 BASE[4][14] =
	{
		{ '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', ':', '-', '[', ']' },
		{ ADM3A_ESC, 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', 0, 0, 0 },
		{ 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '@', '\\', 0, 0 },
		{ 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', ' ', 0, 0, 0 },
	};

	// KBLOCK stops the keyboard logic making GO, so no key does anything.
	if (m_keyboard_locked)
		return;

	const ioport_value modifiers = m_modifiers->read();
	const bool shift = BIT(modifiers, 0) || BIT(modifiers, 1);
	const bool ctrl = BIT(modifiers, 2);
	const bool repeat = BIT(modifiers, 3);

	u8 code = 0;

	if (row < 4)
	{
		code = BASE[row][column];

		if (!code)
			return;

		// ESC has no upper case and no control code of its own; it is already
		// in column 1 of the chart.
		if (code != ADM3A_ESC)
		{
			code = shifted(code, shift);

			// CTRL forces bits 6 and 7 of the generated code to zero, which is
			// what puts DC2 on the line for CTRL R.
			if (ctrl)
				code &= 0x1f;
		}
	}
	else
	{
		// The function keys sit in columns 0 and 1 of the chart, where there
		// is nothing for SHIFT or CTRL to invert.
		static const u8 FUNCTION[5] =
		{
			ADM3A_RS,   // HOME
			ADM3A_LF,   // LINE FEED
			ADM3A_CR,   // RETURN
			ADM3A_ENQ,  // HERE IS
			0x7f        // RUB
		};

		if (column >= (sizeof(FUNCTION) / sizeof(FUNCTION[0])))
			return;

		code = FUNCTION[column];
	}

	key_char(code);

	if (repeat)
	{
		// REPEAT clocks the key through the bounce counter from the power line
		// rate, five pulses to a character.  Table 1-1 quotes 22 characters a
		// second, but 4.3.10 describes the circuit and the rate it gives.
		const attotime period = attotime::from_hz((fifty_hertz() ? 50.0 : 60.0) / 5.0);

		typematic_start(row, column, period, period);
	}
}

void adm3a_device::key_repeat(u8 row, u8 column)
{
	// It is the REPEAT key that does the repeating; let go of it and the held
	// key stops.
	if (!BIT(m_modifiers->read(), 3))
	{
		typematic_stop();
		return;
	}

	key_make(row, column);
}

void adm3a_device::key_break(u8 row, u8 column)
{
	if (typematic_is(row, column))
		typematic_stop();
}


/***************************************************************************
    VIDEO
***************************************************************************/

u8 adm3a_device::glyph_row(u8 chr, u8 row) const
{
	// Bits 6 and 7 of the buffered character pick the ROM: both set means a
	// code of 140 octal or above, which is the lower case part at L14 (4.3.7).
	if (lower_case_displayed())
	{
		const u16 base = ((chr & 0x60) == 0x60) ? 0x200 : 0x000;

		return m_chargen->base()[base | ((chr & 0x3f) << 3) | (row & 0x07)];
	}

	// Without the option the refresh memory is six bits wide, so bit 6 is
	// dropped and regenerated from bit 7.  That is what puts an upper case
	// letter on the screen when the host sends a lower case one.
	const u8 index = (chr & 0x1f) | ((~chr & 0x40) >> 1);

	return m_chargen->base()[(index << 3) | (row & 0x07)];
}

u32 adm3a_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0, cliprect);

	const bool twelve = twelve_line();
	const bool block_cursor = cursor_control();

	for (int row = 0; row < ADM3A_ROWS; row++)
	{
		// In 12-line mode the even character rows are blanked, so the twelve
		// rows that do carry text end up double spaced down the screen.
		if (twelve && !BIT(row, 0))
			continue;

		const u8 line = (row + m_offset) % ADM3A_ROWS;

		for (int scan = 0; scan < ADM3A_SCANS_PER_ROW; scan++)
		{
			const int y = (row * ADM3A_SCANS_PER_ROW) + scan;

			for (int col = 0; col < ADM3A_COLUMNS; col++)
			{
				// Line counts 0 and 8 are the blank rows that space one
				// character row from the next; 1 through 7 are the dot rows.
				u8 dots = (scan < 8) ? glyph_row(ram(line, col), scan) : 0;

				const bool at_cursor = (row == m_row) && (col == m_col);

				if (at_cursor)
				{
					if (block_cursor)
					{
						// CUR CTL on: the coincidence signal is exclusive-ORed
						// with the character video, so the cursor is a reverse
						// image of whatever it sits on.
						dots ^= 0x1f;
					}
					else if (scan >= 7)
					{
						// CUR CTL off: a double underline, five dots wide, on
						// the two line counts below the character.
						dots = 0x1f;
					}
				}

				for (int i = 0; i < 5; i++)
				{
					const int x = (col * ADM3A_DOTS_PER_CHAR) + i;

					if (cliprect.contains(x, y))
						bitmap.pix(y, x) = BIT(dots, 4 - i);
				}
			}
		}
	}

	return 0;
}

void adm3a_device::update_screen_params()
{
	// S4-2 sets the refresh rate to match the power line, which changes how
	// many character rows of vertical retrace there are and nothing else.
	const int vtotal = (fifty_hertz() ? ADM3A_ROWS_50HZ : ADM3A_ROWS_60HZ) * ADM3A_SCANS_PER_ROW;

	const rectangle visarea(0, ADM3A_VISIBLE_WIDTH - 1, 0, ADM3A_VISIBLE_HEIGHT - 1);

	m_screen->configure(ADM3A_HTOTAL, vtotal, visarea,
			attotime::from_ticks(u64(ADM3A_HTOTAL) * vtotal, ADM3A_DOT_CLOCK));
}


/***************************************************************************
    DEVICE
***************************************************************************/

void adm3a_device::device_start()
{
	m_bell_timer = timer_alloc(FUNC(adm3a_device::bell_off), this);

	save_item(NAME(m_ram));
	save_item(NAME(m_row));
	save_item(NAME(m_col));
	save_item(NAME(m_offset));
	save_item(NAME(m_no_write));
	save_item(NAME(m_esc_state));
	save_item(NAME(m_keyboard_locked));
	save_item(NAME(m_tx_char));
	save_item(NAME(m_tx_full));
	save_item(NAME(m_tx_busy));
	save_item(NAME(m_break));
}

void adm3a_device::device_reset()
{
	// The general clear circuit runs a START sequence at power up that writes
	// a space into every character cell and leaves both row counters at
	// minimum count (4.3.1, 4.3.3).
	clear_screen();

	m_keyboard_locked = false;
	m_tx_full = false;
	m_tx_busy = false;
	m_break = false;

	update_frame();
	update_screen_params();

	receive_register_reset();
	transmit_register_reset();

	m_write_sd(1);

	m_bell->set_state(0);

	// The key bounce counter gives a key 5.56 ms to settle, so a full sweep of
	// the five key rows at this rate takes about as long.
	reset_key_state();
	start_processing(attotime::from_hz(1200));
}

void adm3a_device::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen);
	m_screen->set_raw(ADM3A_DOT_CLOCK,
			ADM3A_HTOTAL, 0, ADM3A_VISIBLE_WIDTH,
			ADM3A_ROWS_60HZ * ADM3A_SCANS_PER_ROW, 0, ADM3A_VISIBLE_HEIGHT);
	m_screen->set_screen_update(FUNC(adm3a_device::screen_update));
	m_screen->set_palette(m_palette);

	// The raster is 560 by 216 but the picture on the tube is 8.3 inches wide
	// by 5.8 high (table 1-1), so a dot is nearly twice as tall as it is wide.
	// Say so, or MAME takes the aspect from the visible area, renders square
	// pixels and squashes the display into a letterbox.
	m_screen->set_physical_aspect(83, 58);

	// 12 inch diagonal, P4 phosphor, behind a nonglare surface
	PALETTE(config, m_palette, palette_device::MONOCHROME);

	SPEAKER(config, "mono").front_center();
	BEEP(config, m_bell, ADM3A_BELL_HZ).add_route(ALL_OUTPUTS, "mono", 0.05);
}


static INPUT_PORTS_START( adm3a )

	PORT_START("X0")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('"')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('&')
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('\'')
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8) PORT_CHAR('8') PORT_CHAR('(')
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9) PORT_CHAR('9') PORT_CHAR(')')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR('0')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE) PORT_CHAR(':') PORT_CHAR('*')
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('=')
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('[') PORT_CHAR('{')
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']') PORT_CHAR('}')

	PORT_START("X1")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ESC) PORT_NAME("ESC") PORT_CHAR(UCHAR_MAMEKEY(ESC))
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R')
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T')
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y')
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U')
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P')

	PORT_START("X2")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A) PORT_CHAR('a') PORT_CHAR('A')
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F')
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G')
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H')
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J')
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K')
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR('+')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE) PORT_CHAR('@') PORT_CHAR('`')
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|')

	PORT_START("X3")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z) PORT_CHAR('z') PORT_CHAR('Z')
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V')
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B')
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N')
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M')
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ')

	PORT_START("X4")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_HOME) PORT_NAME("HOME") PORT_CHAR(UCHAR_MAMEKEY(HOME))
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_INSERT) PORT_NAME("LINE FEED") PORT_CHAR(10)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER) PORT_NAME("RETURN") PORT_CHAR(13)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PGUP) PORT_NAME("HERE IS")
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("RUB") PORT_CHAR(UCHAR_MAMEKEY(DEL))

	PORT_START("MODIFIERS")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT) PORT_NAME("Left SHIFT") PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RSHIFT) PORT_NAME("Right SHIFT")
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LCONTROL) PORT_NAME("CTRL") PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RCONTROL) PORT_NAME("REPEAT")
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F12) PORT_NAME("BREAK") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::break_key), 0)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_END) PORT_NAME("CLEAR (with SHIFT)") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::clear_key), 0)

	// S1 AND S2, THE BAUD RATE SWITCHES (TABLE 2-1)
	// --------------------------------------------
	// Eleven switches with one closed at a time.  9600 is the factory setting.
	PORT_START("BAUD")
	PORT_CONFNAME(0x0f, 0x09, "Baud rate") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::frame_changed), 0)
	PORT_CONFSETTING(   0x00, "75")
	PORT_CONFSETTING(   0x01, "110")
	PORT_CONFSETTING(   0x02, "150")
	PORT_CONFSETTING(   0x03, "300")
	PORT_CONFSETTING(   0x04, "600")
	PORT_CONFSETTING(   0x05, "1200")
	PORT_CONFSETTING(   0x06, "1800")
	PORT_CONFSETTING(   0x07, "2400")
	PORT_CONFSETTING(   0x08, "4800")
	PORT_CONFSETTING(   0x09, "9600")
	PORT_CONFSETTING(   0x0a, "19200")

	// S2 THROUGH S5, THE CONFIGURATION CONTROL SWITCHES (TABLE 2-2)
	// ------------------------------------------------------------
	// The word structure defaults to eight bits with no parity rather than the
	// seven the terminal was usually set to, because that is the frame almost
	// every host in MAME sends and the terminal throws the eighth bit away on
	// the way in anyway.
	PORT_START("SWITCHES")
	PORT_CONFNAME(0x0001, 0x0000, "S2-5 Duplex")
	PORT_CONFSETTING(     0x0000, "FDX")
	PORT_CONFSETTING(     0x0001, "HDX")
	PORT_CONFNAME(0x0002, 0x0002, "S2-7 Auto new line")
	PORT_CONFSETTING(     0x0000, DEF_STR( Off ))
	PORT_CONFSETTING(     0x0002, DEF_STR( On ))
	PORT_CONFNAME(0x0004, 0x0004, "S3-1 Lower case codes")
	PORT_CONFSETTING(     0x0000, "UC")
	PORT_CONFSETTING(     0x0004, "LC EN")
	PORT_CONFNAME(0x0008, 0x0000, "S3-2 Parity sense") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::frame_changed), 0)
	PORT_CONFSETTING(     0x0000, "Even")
	PORT_CONFSETTING(     0x0008, "Odd")
	PORT_CONFNAME(0x0010, 0x0010, "S3-3 Data bits") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::frame_changed), 0)
	PORT_CONFSETTING(     0x0000, "7")
	PORT_CONFSETTING(     0x0010, "8")
	PORT_CONFNAME(0x0020, 0x0000, "S3-4 Stop bits") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::frame_changed), 0)
	PORT_CONFSETTING(     0x0000, "1")
	PORT_CONFSETTING(     0x0020, "2")
	PORT_CONFNAME(0x0040, 0x0000, "S3-5 Parity") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::frame_changed), 0)
	PORT_CONFSETTING(     0x0000, "INH")
	PORT_CONFSETTING(     0x0040, "PARITY")
	PORT_CONFNAME(0x0080, 0x0000, "S4-1 Display format") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::refresh_changed), 0)
	PORT_CONFSETTING(     0x0000, "24 line")
	PORT_CONFSETTING(     0x0080, "12 line")
	PORT_CONFNAME(0x0100, 0x0000, "S4-2 Refresh rate") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(adm3a_device::refresh_changed), 0)
	PORT_CONFSETTING(     0x0000, "60 Hz")
	PORT_CONFSETTING(     0x0100, "50 Hz")
	PORT_CONFNAME(0x0200, 0x0200, "S4-3 Remote clear screen")
	PORT_CONFSETTING(     0x0000, "DISABLE")
	PORT_CONFSETTING(     0x0200, "CLR SCRN")
	PORT_CONFNAME(0x0400, 0x0400, "S4-4 Remote keyboard lock")
	PORT_CONFSETTING(     0x0000, "DISABLE")
	PORT_CONFSETTING(     0x0400, "KB LOCK")
	PORT_CONFNAME(0x0800, 0x0800, "S4-5 Display case")
	PORT_CONFSETTING(     0x0000, "UC DISP")
	PORT_CONFSETTING(     0x0800, "U/L DISP")
	PORT_CONFNAME(0x1000, 0x1000, "S4-6 Space code")
	PORT_CONFSETTING(     0x0000, "ADV (nondestructive)")
	PORT_CONFSETTING(     0x1000, "SPACE (destructive)")
	PORT_CONFNAME(0x2000, 0x2000, "S5-7 Cursor control")
	PORT_CONFSETTING(     0x0000, "OFF (bottom line, underline cursor)")
	PORT_CONFSETTING(     0x2000, "CUR CTL (addressable, block cursor)")

	// S7, S8 AND THE OPTION BOARDS
	// ----------------------------
	// The lower case option is fitted by default: it is what makes the second
	// character ROM at L14 do anything, and a terminal without it answers a
	// lower case code with an upper case letter.
	PORT_START("OPTIONS")
	PORT_CONFNAME(0x01, 0x01, "Lower case option")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x01, "Fitted")
	PORT_CONFNAME(0x02, 0x02, "S7 Column 72 beep")
	PORT_CONFSETTING(   0x00, "DEFEAT 72 COL BEEP")
	PORT_CONFSETTING(   0x02, "72 COL BEEP")
	PORT_CONFNAME(0x04, 0x00, "S8 CTRL N and CTRL O")
	PORT_CONFSETTING(   0x00, "LK (lock keyboard)")
	PORT_CONFSETTING(   0x04, "GT (gate extension port)")

INPUT_PORTS_END

ioport_constructor adm3a_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(adm3a);
}


// The character generator at L15 is a stock RO-3-2513, the same dump MAME
// already carries as the Apple I's s2513.d2.  bitsavers has a read of an
// ADM-3A's own pair of parts at learSiegler/ADM_3/ADM3_Char_ROMs; the upper
// case half of that read, majority voted over the four copies it contains and
// masked to the five bits the part has, comes back byte for byte identical to
// this file.
//
// The lower case half is the matching RO-3-2513-005 at L14, which the option
// board adds.  bitsavers' read of it has its six address lines inverted, so
// the file holds rubout where 'a' should be; the bytes below are that read
// with the inversion taken out.  That puts the glyphs where the
// RO-3-2513/CGR-005 data sheet says they belong, 040 through 137 octal in the
// first half and 140 through 177 in the second, and it puts lower case 's' at
// address 110011, which is the worked example the data sheet itself prints.
ROM_START( adm3a )
	ROM_REGION( 0x0400, "chargen", 0 )
	ROM_LOAD( "ro-3-2513.l15",     0x0000, 0x0200, CRC(a7e567fc) SHA1(b18aae0a2d4f92f5a7e22640719bbc4652f3f4ee) )
	ROM_LOAD( "ro-3-2513-005.l14", 0x0200, 0x0200, CRC(7788ab1c) SHA1(b164f1d33eb9790fb466cf57c51169be6510bc1d) )
ROM_END

const tiny_rom_entry *adm3a_device::device_rom_region() const
{
	return ROM_NAME( adm3a );
}
