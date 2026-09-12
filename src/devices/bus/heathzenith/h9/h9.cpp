// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heathkit H9 Video Terminal

  Heath's first terminal, and the one the H8 was sold with: 12 lines of 80
  upper-case characters on a 12" CRT, 110 to 9600 baud.  Heath's own interface
  standard pairs it with the H8's H-8-5 serial card at 600 baud, EIA levels,
  8 bits, no parity (H9 Operations, page 10).

  WHAT IS MODELLED, AND WHY IT LOOKS NOTHING LIKE THE OTHER HEATH TERMINALS
  ------------------------------------------------------------------------
  There is no CPU here to run.  The H9 predates the H19 by two years and is
  built entirely from TTL across five boards - power supply, video, character
  generator, RAM and counter, I/O, and the timing and processing unit (TPU)
  that sequences the lot during vertical retrace.  So this device is a
  behavioural model of what those boards do, written from the circuit
  description in the H9 Operations manual (595-2017-03, pages 47-67), not a
  gate-level one.

  The one piece of real silicon that survives is the character generator: a
  stock RO-3-2513 at IC205, whose 64 glyphs the manual prints in full.

  The parts of the TPU that are kept rather than flattened are the three RAM
  address counters - divide by 20, by 4 and by 12 - because every difference
  between long form and short form, and the whole of scrolling, falls out of
  how they are cascaded.  Flattening them into a row/column pair would mean
  special-casing all of that by hand.

  WHAT IS NOT MODELLED
  --------------------
  The 8-bit parallel port and its four handshake lines (the reader/punch
  interface), and the reader start/stop current loop.  Nothing on the MAME
  side would connect to them.

****************************************************************************/

#include "emu.h"

#include "h9.h"

#include "machine/keyboard.ipp"


namespace {

// Master clock on the character generator board (IC210B, crystal Y201).  Eight
// dots per character, 100 character times per scan line and 256 scan lines per
// frame give the manual's quoted 15494 Hz horizontal and 60.5 Hz vertical.
static constexpr XTAL H9_DOT_CLOCK  = 12.395_MHz_XTAL;

static constexpr int H9_DOTS_PER_CHAR   = 8;
static constexpr int H9_CHARS_PER_LINE  = 100;
static constexpr int H9_COLUMNS         = 80;
static constexpr int H9_SCANS_PER_FRAME = 256;
static constexpr int H9_SCANS_PER_ROW   = 16;
static constexpr int H9_ROWS            = 12;

static constexpr int H9_VISIBLE_WIDTH   = H9_COLUMNS * H9_DOTS_PER_CHAR;
static constexpr int H9_VISIBLE_HEIGHT  = H9_ROWS * H9_SCANS_PER_ROW;

// Plot mode divides the top eight character rows into 128 individually
// addressable scan lines and repeats character row 0 in the bottom four.
static constexpr int H9_PLOT_LINES      = 128;

// IC736 holds the speaker on for about 30 ms, driven from the 300 baud clock.
static constexpr int H9_BELL_HZ         = 300;
static constexpr int H9_BELL_MS         = 30;

// Scan line within the 16-line character cell that the cursor's two dashes
// appear on (IC222C decodes the row counter for exactly these two).
static constexpr int H9_CURSOR_FIRST_SCAN = 12;
static constexpr int H9_CURSOR_LAST_SCAN  = 13;

} // anonymous namespace


DEFINE_DEVICE_TYPE(HEATH_H9, heath_h9_device, "heath_h9", "Heathkit H9 Video Terminal")


heath_h9_device::heath_h9_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, HEATH_H9, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, device_matrix_keyboard_interface(mconfig, *this, "X0", "X1", "X2", "X3", "X4")
	, m_screen(*this, "screen")
	, m_palette(*this, "palette")
	, m_bell(*this, "bell")
	, m_chargen(*this, "chargen")
	, m_modifiers(*this, "MODIFIERS")
	, m_modes(*this, "MODES")
	, m_jumpers(*this, "JUMPERS")
	, m_write_sd(*this)
{
	std::fill(std::begin(m_ram), std::end(m_ram), ' ');
}


/***************************************************************************
    FRONT PANEL AND JUMPER STATE

    The seven latching keys are read live rather than latched at reset, so
    flipping one from the Machine Configuration menu behaves the way pushing
    the key does on the real terminal.
***************************************************************************/

bool heath_h9_device::short_form() const  { return BIT(m_modes->read(), 0); }
bool heath_h9_device::auto_carry() const  { return BIT(m_modes->read(), 1); }
bool heath_h9_device::scroll_key() const  { return BIT(m_modes->read(), 2); }
bool heath_h9_device::plot_mode() const   { return BIT(m_modes->read(), 3); }
bool heath_h9_device::full_duplex() const { return BIT(m_modes->read(), 4); }
bool heath_h9_device::off_line() const    { return BIT(m_modes->read(), 5); }

u32 heath_h9_device::baud_rate() const
{
	// With the BAUD RATE key released the terminal runs at 110 baud whatever
	// else is set.  Pushed in, the rear panel switch picks either 300 or the
	// rate wired by the jumper on the I/O circuit board.
	if (!BIT(m_modes->read(), 6))
		return 110;

	static const u32 PRESET[8] = { 600, 1200, 2400, 4800, 9600, 9600, 9600, 9600 };

	const ioport_value jumpers = m_jumpers->read();

	if (BIT(jumpers, 3))
		return 300;

	// a machine can wire the preset tap itself; see set_preset_baud
	if (m_preset_baud)
		return m_preset_baud;

	return PRESET[jumpers & 0x07];
}


/***************************************************************************
    RAM, THE CURSOR COUNTERS AND SCROLLING

    RAM holds 960 characters as 12 lines of four 20-character blocks.  The
    cursor is the three counters m_c20, m_c4 and m_c12; the start of page is
    m_scroll12 (long form) or m_scroll4 (short form).
***************************************************************************/

void heath_h9_device::advance_cursor()
{
	if (m_c20 < 19)
	{
		m_c20++;
		return;
	}

	m_c20 = 0;

	if (short_form())
	{
		// +20 cascades into +12, and +12 into +4 only with auto carry
		if (m_c12 < 11)
		{
			m_c12++;
			return;
		}

		if (!auto_carry())
		{
			// no carry out: the cursor sits on the last character of the block
			m_c20 = 19;
			return;
		}

		m_c12 = 0;
		m_c4 = (m_c4 + 1) & 3;
	}
	else
	{
		// +20 cascades into +4, and +4 into +12 only with auto carry
		if (m_c4 < 3)
		{
			m_c4++;
			return;
		}

		if (!auto_carry())
		{
			// no carry out: the cursor sits on the last character of the line
			m_c20 = 19;
			return;
		}

		m_c4 = 0;
		m_c12 = (m_c12 + 1) % H9_ROWS;
	}

	check_scroll();
}

void heath_h9_device::cursor_left()
{
	// The +20 counter counts down, and in long form its borrow cascades into
	// +4 the same way its carry does, so backing up off the start of a block
	// lands on the last character of the one before.
	if (m_c20)
	{
		m_c20--;
		return;
	}

	m_c20 = 19;

	if (!short_form())
		m_c4 = (m_c4 - 1) & 3;
}

void heath_h9_device::cursor_down()
{
	if (short_form())
	{
		// Down a line inside the 20-character block, and out of the bottom of
		// the block into the next one - which, like the end of a line, only
		// happens with auto carry.
		if (m_c12 < 11)
		{
			m_c12++;
			return;
		}

		if (!auto_carry())
			return;

		m_c12 = 0;
		m_c4 = (m_c4 + 1) & 3;
		check_scroll();
		return;
	}

	m_c12 = (m_c12 + 1) % H9_ROWS;
	check_scroll();
}

void heath_h9_device::check_scroll()
{
	// Call this only where the counter that carries the start of page has just
	// been counted up: +12 in long form, +4 in short form.  Calling it after
	// any other movement would find the cursor sitting on the start of page
	// for a whole line or block at a time and scroll over and over.
	const bool at_start_of_page = short_form() ? (m_c4 == m_scroll4) : (m_c12 == m_scroll12);

	if (!at_start_of_page)
		return;

	if (scroll_key())
	{
		do_scroll();
		return;
	}

	// SCROLL released: hold screen.  The cursor drops back to the start of
	// page and blinks there, and everything arriving from outside is dropped
	// until SCROLL or ERASE PAGE.
	m_hold_screen = true;
	home_cursor();
}

void heath_h9_device::do_scroll()
{
	// Blank the line (long form) or block (short form) the cursor has just
	// landed on, then step the start of page past it, which brings that line
	// back as the bottom one - or that block back as the right-hand one.
	erase_line();

	if (short_form())
		m_scroll4 = (m_scroll4 + 1) & 3;
	else
		m_scroll12 = (m_scroll12 + 1) % H9_ROWS;
}

void heath_h9_device::erase_line()
{
	if (short_form())
	{
		for (u8 line = 0; line < H9_ROWS; line++)
			for (u8 chr = 0; chr < 20; chr++)
				m_ram[ram_addr(line, m_c4, chr)] = ' ';
	}
	else
	{
		for (u8 block = 0; block < 4; block++)
			for (u8 chr = 0; chr < 20; chr++)
				m_ram[ram_addr(m_c12, block, chr)] = ' ';
	}
}

void heath_h9_device::erase_to_end_of_line()
{
	// The RAM counter is stopped by the +20 end of line in short form, and by
	// +20 and +4 together in long form, so "line" means the 20-character block
	// in short form and the whole 80 characters in long form.
	u8 block = m_c4;
	u8 chr = m_c20;

	for (;;)
	{
		m_ram[ram_addr(m_c12, block, chr)] = ' ';

		if (chr < 19)
		{
			chr++;
			continue;
		}

		if (short_form() || block == 3)
			break;

		chr = 0;
		block++;
	}
}

void heath_h9_device::erase_page()
{
	std::fill(std::begin(m_ram), std::end(m_ram), ' ');
	home_cursor();
	m_hold_screen = false;
}

void heath_h9_device::home_cursor()
{
	m_c20 = 0;
	m_c4 = short_form() ? m_scroll4 : 0;
	m_c12 = short_form() ? 0 : m_scroll12;
}


/***************************************************************************
    CHARACTER HANDLING

    Pictorial 4-15 in the H9 Operations manual is the whole of this: which
    characters reach RAM, and how each one moves the cursor.
***************************************************************************/

void heath_h9_device::handle_char(u8 data, bool from_keyboard)
{
	const u8 ch = data & 0x7f;

	if (plot_mode())
	{
		// In plot mode every character is written, including control
		// characters and rubout, and none of them do anything but move the
		// cursor one place to the right.
		write_ram(ch);
		advance_cursor();
		return;
	}

	switch (ch)
	{
	case 0x0d: // carriage return
		// The only control character that reaches RAM in normal mode, and only
		// where the cursor is already over a space.  Transmit page looks for
		// these to find where a line ends.
		if (read_ram() == ' ')
			write_ram(ch);

		// The RETURN key "moves the cursor to the first character position of
		// the line it is currently in", which in long form means the start of
		// all 80 characters, so the block counter has to go back to zero as
		// well.  The circuit description only mentions the +20 counter being
		// cleared (H9 Operations, "SPECIAL CURSOR MOVEMENTS"); clearing +20
		// alone lands the cursor at the start of the current 20-character
		// block instead, 20, 40 or 60 characters into the line.  The schematic
		// fold-in would settle which signal clears +4.
		m_c20 = 0;
		if (!short_form())
			m_c4 = 0;
		break;

	case 0x0a: // line feed
		cursor_down();
		break;

	case 0x08: // back space
		cursor_left();
		break;

	case 0x07: // bell
		ring_bell();
		break;

	case 0x7f: // rub out - recognised, but neither written nor moving
		break;

	default:
		if (ch < 0x20)
			break;

		write_ram(ch);

		// The bell also sounds seven characters from the end of a line, but
		// only for keyboard input (H9 Operations, "BELL DETECT").
		if (from_keyboard && m_c20 == 13 && (short_form() || m_c4 == 3))
			ring_bell();

		advance_cursor();
		break;
	}
}

void heath_h9_device::ring_bell()
{
	m_bell->set_state(1);
	m_bell_timer->adjust(attotime::from_msec(H9_BELL_MS));
}

TIMER_CALLBACK_MEMBER(heath_h9_device::bell_off)
{
	m_bell->set_state(0);
}


/***************************************************************************
    SERIAL PORT
***************************************************************************/

void heath_h9_device::serial_in_w(int state)
{
	rx_w(state);
}

INPUT_CHANGED_MEMBER(heath_h9_device::break_key)
{
	// Held down, the BREAK key drives a continuous space at the serial output
	// - not an ASCII space, a real spacing condition - which is how you
	// interrupt a program on the host.
	if (newval)
		m_write_sd(0);
	else if (!m_tx_busy)
		m_write_sd(1);
}

INPUT_CHANGED_MEMBER(heath_h9_device::baud_changed)
{
	set_rate(baud_rate());
}

INPUT_CHANGED_MEMBER(heath_h9_device::scroll_key_changed)
{
	// Pushing SCROLL in is one of the two ways out of hold screen; the other
	// is ERASE PAGE, which clears the flag in erase_page().
	if (newval)
		m_hold_screen = false;
}

void heath_h9_device::rcv_complete()
{
	receive_register_extract();

	// OFF LINE holds a continuous mark on the serial input, and hold screen
	// drops the handshake so nothing more arrives.  Either way the character
	// never reaches the TPU.
	if (off_line() || m_hold_screen)
		return;

	handle_char(get_received_char(), false);
}

void heath_h9_device::send_char(u8 data)
{
	if (off_line())
		return;

	const u16 next = (m_tx_tail + 1) % TX_FIFO_SIZE;

	if (next == m_tx_head)
		return; // queue full; the real UART would simply not be loaded yet

	m_tx_fifo[m_tx_tail] = data;
	m_tx_tail = next;

	if (!m_tx_busy && is_transmit_register_empty())
	{
		m_tx_busy = true;
		transmit_register_setup(m_tx_fifo[m_tx_head]);
		m_tx_head = (m_tx_head + 1) % TX_FIFO_SIZE;
	}
}

void heath_h9_device::tra_callback()
{
	m_write_sd(transmit_register_get_data_bit());
}

void heath_h9_device::tra_complete()
{
	if (m_tx_head != m_tx_tail)
	{
		transmit_register_setup(m_tx_fifo[m_tx_head]);
		m_tx_head = (m_tx_head + 1) % TX_FIFO_SIZE;
		return;
	}

	m_tx_busy = false;
	m_write_sd(1);
}


/***************************************************************************
    TRANSMIT PAGE

    Sends RAM from the cursor to the end of the page, turning a stored
    carriage return - or the end of a line with auto carry off - into a
    carriage return and line feed, then homes the cursor and stops.
***************************************************************************/

void heath_h9_device::start_transmit_page()
{
	u8 line = m_c12;
	u8 block = m_c4;
	u8 chr = m_c20;

	for (int sent = 0; sent < 960; sent++)
	{
		const u8 ch = m_ram[ram_addr(line, block, chr)];
		const bool end_of_line = (chr == 19) && (short_form() || block == 3);

		if (ch == 0x0d)
		{
			send_char(0x0d);
			send_char(0x0a);
		}
		else
		{
			send_char(ch);

			if (end_of_line && !auto_carry())
			{
				send_char(0x0d);
				send_char(0x0a);
			}
		}

		// step to the next RAM location the way the cursor would
		if (chr < 19)
		{
			chr++;
			continue;
		}

		chr = 0;

		if (short_form())
		{
			if (line < 11)
			{
				line++;
				continue;
			}
			line = 0;
			if (block == 3)
				break;
			block++;
		}
		else
		{
			if (block < 3)
			{
				block++;
				continue;
			}
			block = 0;
			if (line == H9_ROWS - 1)
				break;
			line++;
		}
	}

	home_cursor();
}


/***************************************************************************
    KEYBOARD
***************************************************************************/

void heath_h9_device::key_char(u8 data)
{
	// Half duplex writes the key into RAM as well as sending it; full duplex
	// only sends, and waits for the host to echo it back.
	send_char(data);

	if (!full_duplex())
		handle_char(data, true);
}

void heath_h9_device::key_make(u8 row, u8 column)
{
	// Rows 0 to 3 are the ASCII keys, laid out unshifted/shifted; rows 4 and
	// up are the local function keys, which send nothing.
	static const char ASCII[4][16][2] =
	{
		{ {'1','!'}, {'2','"'}, {'3','#'}, {'4','$'}, {'5','%'}, {'6','&'}, {'7','\''}, {'8','('},
		  {'9',')'}, {'0','_'}, {':','*'}, {';','+'}, {'-','='}, {0,0}, {0,0}, {0,0} },
		{ {'Q','Q'}, {'W','W'}, {'E','E'}, {'R','R'}, {'T','T'}, {'Y','Y'}, {'U','U'}, {'I','I'},
		  {'O','O'}, {'P','P'}, {'@','@'}, {'[','['}, {'\\','\\'}, {0,0}, {0,0}, {0,0} },
		{ {'A','A'}, {'S','S'}, {'D','D'}, {'F','F'}, {'G','G'}, {'H','H'}, {'J','J'}, {'K','K'},
		  {'L','L'}, {']',']'}, {'^','^'}, {0x1b,0x1b}, {0x0d,0x0d}, {0,0}, {0,0}, {0,0} },
		{ {'Z','Z'}, {'X','X'}, {'C','C'}, {'V','V'}, {'B','B'}, {'N','N'}, {'M','M'}, {',','<'},
		  {'.','>'}, {'/','?'}, {' ',' '}, {0x0a,0x0a}, {0x7f,0x7f}, {0,0}, {0,0}, {0,0} },
	};

	const u16 modifiers = m_modifiers->read();
	const bool shift = BIT(modifiers, 0) || BIT(modifiers, 1);
	const bool ctrl = BIT(modifiers, 2);
	const bool repeat = BIT(modifiers, 3);

	if (row < 4)
	{
		u8 ch = ASCII[row][column][shift ? 1 : 0];

		if (!ch)
			return;

		// CTRL clears bits 6 and 7.  CTRL/SHIFT P is documented as producing
		// a null; so does CTRL with the @ key, which is the same code.
		if (ctrl)
			ch &= 0x1f;

		key_char(ch);

		if (repeat)
		{
			static const int RATE[4] = { 37, 75, 150, 300 };
			const attotime period = attotime::from_hz(RATE[(m_jumpers->read() >> 6) & 0x03] / 10.0);
			typematic_start(row, column, period, period);
		}

		return;
	}

	// Local function keys.  None of them put anything on the ASCII bus.
	switch ((row << 4) | column)
	{
	case 0x40: home_cursor(); break;
	case 0x41: m_c12 = m_c12 ? (m_c12 - 1) : (H9_ROWS - 1); break;   // cursor up
	case 0x42: cursor_down(); break;                                 // cursor down
	case 0x43: cursor_left(); break;                                 // cursor left
	case 0x44: advance_cursor(); break;                              // cursor right
	case 0x45: erase_to_end_of_line(); break;
	case 0x46: erase_page(); break;
	case 0x47: start_transmit_page(); break;
	default: break;
	}
}

void heath_h9_device::key_repeat(u8 row, u8 column)
{
	// The REPT key does the repeating on this keyboard; let go of it and the
	// held key stops.
	if (!BIT(m_modifiers->read(), 3))
	{
		typematic_stop();
		return;
	}

	key_make(row, column);
}

void heath_h9_device::key_break(u8 row, u8 column)
{
	if (typematic_is(row, column))
		typematic_stop();
}


/***************************************************************************
    VIDEO
***************************************************************************/

u8 heath_h9_device::glyph_row(u8 chr, u8 row) const
{
	// Only the six least significant bits of the 7-bit ASCII word address the
	// character generator, so bit 6 is dropped and lower case comes out as
	// upper case (H9 Operations, Pictorial 4-4).
	const u8 index = (chr & 0x1f) | ((~chr & 0x40) >> 1);

	return m_chargen->base()[(index << 3) | (row & 0x07)];
}

void heath_h9_device::draw_dots(bitmap_ind16 &bitmap, const rectangle &cliprect, int x, int y, u8 dots)
{
	// Five dots, then the three blanked dot times that space one character
	// from the next.
	for (int i = 0; i < 5; i++)
	{
		const int px = x + i;

		if (cliprect.contains(px, y))
			bitmap.pix(y, px) = BIT(dots, 4 - i);
	}
}

u32 heath_h9_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0, cliprect);

	// IC207B's preset comes from a 4 Hz clock, which unblanks a stored control
	// character four times a second so it appears to blink.
	const bool control_visible = BIT(int(machine().time().as_double() * 8.0), 0);

	// The cursor is steady except in hold screen, where it flashes to say the
	// terminal has stopped taking data.
	const bool cursor_visible = !m_hold_screen || BIT(int(machine().time().as_double() * 4.0), 0);

	const bool plot = plot_mode();

	// Where the cursor lands on the screen, given where the start of page is.
	const int cursor_row = short_form() ? m_c12 : ((m_c12 - m_scroll12 + H9_ROWS) % H9_ROWS);
	const int cursor_col = short_form()
			? (((m_c4 - m_scroll4 + 4) & 3) * 20) + m_c20
			: (m_c4 * 20) + m_c20;

	for (int row = 0; row < H9_ROWS; row++)
	{
		// In plot mode the top eight character rows are given over to the
		// plot raster and the bottom four all show character row 0.
		const int source_row = (plot && row >= 8) ? 0 : row;

		const int line = short_form() ? source_row : ((m_scroll12 + source_row) % H9_ROWS);

		for (int scan = 0; scan < H9_SCANS_PER_ROW; scan++)
		{
			const int y = (row * H9_SCANS_PER_ROW) + scan;

			if (plot && y < H9_PLOT_LINES)
				continue; // filled in below, a dash per column

			for (int col = 0; col < H9_COLUMNS; col++)
			{
				const int block = short_form() ? ((m_scroll4 + (col / 20)) & 3) : (col / 20);
				const u8 chr = m_ram[ram_addr(line, block, col % 20)] & 0x7f;

				u8 dots = 0;

				if (scan < 8)
				{
					// Bits 6 and 7 both zero means a control character, which
					// is blanked except when the 4 Hz blink unblanks it.
					if ((chr & 0x60) != 0 || control_visible)
						dots = glyph_row(chr, scan);
				}

				if (cursor_visible && !plot
						&& scan >= H9_CURSOR_FIRST_SCAN && scan <= H9_CURSOR_LAST_SCAN
						&& row == cursor_row && col == cursor_col)
					dots = 0x1f;

				draw_dots(bitmap, cliprect, col * H9_DOTS_PER_CHAR, y, dots);
			}
		}
	}

	if (plot)
	{
		// A dash above each of the 80 characters of the first line, on the
		// plot line matching that character's 7-bit value.  Line 0 is at the
		// bottom of the 128, line 127 at the top.
		const int line = short_form() ? 0 : m_scroll12;

		for (int col = 0; col < H9_COLUMNS; col++)
		{
			const int block = short_form() ? (m_scroll4 & 3) : (col / 20);
			const u8 chr = m_ram[ram_addr(line, block, col % 20)] & 0x7f;
			const int y = (H9_PLOT_LINES - 1) - chr;

			draw_dots(bitmap, cliprect, col * H9_DOTS_PER_CHAR, y, 0x1f);
		}
	}

	return 0;
}


/***************************************************************************
    DEVICE
***************************************************************************/

void heath_h9_device::device_start()
{
	m_bell_timer = timer_alloc(FUNC(heath_h9_device::bell_off), this);

	save_item(NAME(m_ram));
	save_item(NAME(m_c20));
	save_item(NAME(m_c4));
	save_item(NAME(m_c12));
	save_item(NAME(m_scroll12));
	save_item(NAME(m_scroll4));
	save_item(NAME(m_hold_screen));
	save_item(NAME(m_tx_fifo));
	save_item(NAME(m_tx_head));
	save_item(NAME(m_tx_tail));
	save_item(NAME(m_tx_busy));
}

void heath_h9_device::device_reset()
{
	// IC717A's 200 ms power up pulse resets the UART, holds the erase flip
	// flop set so the whole page is blanked, and momentarily forces short form
	// so the +4 scroll counter clears and block zero ends up at the left.
	m_scroll12 = 0;
	m_scroll4 = 0;
	m_hold_screen = false;
	erase_page();

	m_tx_head = 0;
	m_tx_tail = 0;
	m_tx_busy = false;

	const ioport_value jumpers = m_jumpers->read();

	set_data_frame(1, 5 + ((jumpers >> 4) & 0x03),
			BIT(jumpers, 8) ? (BIT(jumpers, 9) ? PARITY_EVEN : PARITY_ODD) : PARITY_NONE,
			STOP_BITS_1);
	set_rate(baud_rate());

	receive_register_reset();
	transmit_register_reset();

	m_write_sd(1);

	m_bell->set_state(0);

	reset_key_state();
	start_processing(attotime::from_hz(2400));
}

void heath_h9_device::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen);
	m_screen->set_raw(H9_DOT_CLOCK,
			H9_CHARS_PER_LINE * H9_DOTS_PER_CHAR, 0, H9_VISIBLE_WIDTH,
			H9_SCANS_PER_FRAME, 0, H9_VISIBLE_HEIGHT);
	m_screen->set_screen_update(FUNC(heath_h9_device::screen_update));
	m_screen->set_palette(m_palette);

	// The raster is only 192 lines tall but it fills a 12" 4:3 tube, so a dot
	// is about two and a half times taller than it is wide and the twelve
	// lines of text are spaced well apart.  Say so, or MAME takes the aspect
	// from the 640x192 visible area and renders square pixels, which squashes
	// the display into a letterbox nothing like the real terminal.
	m_screen->set_physical_aspect(4, 3);

	// 12" diagonal, P4 phosphor - white, behind a neutral glare filter
	PALETTE(config, m_palette, palette_device::MONOCHROME);

	SPEAKER(config, "mono").front_center();
	BEEP(config, m_bell, H9_BELL_HZ).add_route(ALL_OUTPUTS, "mono", 0.05);
}


static INPUT_PORTS_START( h9 )

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
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR('_')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE) PORT_CHAR(':') PORT_CHAR('*')
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR('+')
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('=')

	PORT_START("X1")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q) PORT_CHAR('Q')
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W) PORT_CHAR('W')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E) PORT_CHAR('E')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R) PORT_CHAR('R')
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_CHAR('T')
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y) PORT_CHAR('Y')
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U) PORT_CHAR('U')
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I) PORT_CHAR('I')
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O) PORT_CHAR('O')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_CHAR('P')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE) PORT_CHAR('@')
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('[')
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\')

	PORT_START("X2")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A) PORT_CHAR('A')
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S) PORT_CHAR('S')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D) PORT_CHAR('D')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F) PORT_CHAR('F')
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G) PORT_CHAR('G')
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H) PORT_CHAR('H')
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J) PORT_CHAR('J')
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K) PORT_CHAR('K')
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_CHAR('L')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('^')
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ESC) PORT_NAME("ESC") PORT_CHAR(UCHAR_MAMEKEY(ESC))
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER) PORT_NAME("RETURN") PORT_CHAR(13)

	PORT_START("X3")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z) PORT_CHAR('Z')
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X) PORT_CHAR('X')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_CHAR('C')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V) PORT_CHAR('V')
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B) PORT_CHAR('B')
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N) PORT_CHAR('N')
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M) PORT_CHAR('M')
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<')
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ')
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_INSERT) PORT_NAME("LINE FEED") PORT_CHAR(10)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE) PORT_NAME("RUB OUT") PORT_CHAR(UCHAR_MAMEKEY(DEL))

	PORT_START("X4")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_HOME)  PORT_NAME("HOME")
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_UP)    PORT_NAME("Cursor Up")
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_DOWN)  PORT_NAME("Cursor Down")
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LEFT)  PORT_NAME("Cursor Left")
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RIGHT) PORT_NAME("Cursor Right")
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_END)   PORT_NAME("ERASE EOL")
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PGDN)  PORT_NAME("ERASE PAGE")
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_PGUP)  PORT_NAME("XMIT PAGE")

	PORT_START("MODIFIERS")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT) PORT_NAME("Left SHIFT") PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RSHIFT) PORT_NAME("Right SHIFT")
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_LCONTROL) PORT_NAME("CTRL") PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_RCONTROL) PORT_NAME("REPT")
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F12) PORT_NAME("BREAK") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(heath_h9_device::break_key), 0)

	// THE SEVEN LATCHING FRONT PANEL KEYS
	// -----------------------------------
	// These are push-push switches on the real keyboard, not settings, but
	// they only ever select a mode, so they are offered here as machine
	// configuration.  The defaults are how you would leave an H9 sitting on a
	// desk next to an H8: scrolling, carrying over at the end of a line, and
	// running at the rate the rear panel switch selects rather than 110 baud.
	//
	// FULL DUPLEX starts pushed in because everything this terminal is likely
	// to be plugged into echoes - HDOS and the H8 monitor ROMs do, and so does
	// the loopback the standalone driver comes up with.  Released, the
	// terminal writes each key into RAM itself as well as sending it, so
	// against an echoing host every character you type appears twice.
	PORT_START("MODES")
	PORT_CONFNAME(0x01, 0x00, "SHORT FORM")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x01, DEF_STR( On ))
	PORT_CONFNAME(0x02, 0x02, "AUTO CARRY")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x02, DEF_STR( On ))
	PORT_CONFNAME(0x04, 0x04, "SCROLL") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(heath_h9_device::scroll_key_changed), 0)
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x04, DEF_STR( On ))
	PORT_CONFNAME(0x08, 0x00, "PLOT")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x08, DEF_STR( On ))
	PORT_CONFNAME(0x10, 0x10, "FULL DUPLEX")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x10, DEF_STR( On ))
	PORT_CONFNAME(0x20, 0x00, "OFF LINE")
	PORT_CONFSETTING(   0x00, DEF_STR( Off ))
	PORT_CONFSETTING(   0x20, DEF_STR( On ))
	PORT_CONFNAME(0x40, 0x40, "BAUD RATE key") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(heath_h9_device::baud_changed), 0)
	PORT_CONFSETTING(   0x00, "Out (110 baud)")
	PORT_CONFSETTING(   0x40, "In (rear panel switch)")

	// THE I/O BOARD JUMPERS AND THE REAR PANEL SWITCH
	// -----------------------------------------------
	// 600 baud, 8 bits, no parity is Heath's own Computer System Interface
	// Standard for an H9 on an H8 (H9 Operations, page 10), and matches the
	// H-8-5 wired as Pictorial 2-2 shows it.
	PORT_START("JUMPERS")
	PORT_CONFNAME(0x07, 0x00, "Preset baud rate") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(heath_h9_device::baud_changed), 0)
	PORT_CONFSETTING(   0x00, "600")
	PORT_CONFSETTING(   0x01, "1200")
	PORT_CONFSETTING(   0x02, "2400")
	PORT_CONFSETTING(   0x03, "4800")
	PORT_CONFSETTING(   0x04, "9600")
	PORT_CONFNAME(0x08, 0x00, "Rear panel BAUD RATE switch") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(heath_h9_device::baud_changed), 0)
	PORT_CONFSETTING(   0x00, "Preset")
	PORT_CONFSETTING(   0x08, "300")
	PORT_CONFNAME(0x30, 0x30, "Word length")
	PORT_CONFSETTING(   0x00, "5 bits")
	PORT_CONFSETTING(   0x10, "6 bits")
	PORT_CONFSETTING(   0x20, "7 bits")
	PORT_CONFSETTING(   0x30, "8 bits")
	PORT_CONFNAME(0xc0, 0x80, "Repeat rate")
	PORT_CONFSETTING(   0x00, "3.7 characters/second")
	PORT_CONFSETTING(   0x40, "7.5 characters/second")
	PORT_CONFSETTING(   0x80, "15 characters/second")
	PORT_CONFSETTING(   0xc0, "30 characters/second")
	PORT_CONFNAME(0x100, 0x000, "Parity")
	PORT_CONFSETTING(    0x000, DEF_STR( None ))
	PORT_CONFSETTING(    0x100, "Generated")
	PORT_CONFNAME(0x200, 0x000, "Parity sense")
	PORT_CONFSETTING(    0x000, "Odd")
	PORT_CONFSETTING(    0x200, "Even")

INPUT_PORTS_END

ioport_constructor heath_h9_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(h9);
}


// The character generator at IC205 is a stock RO-3-2513, Heath part 443-812.
// The bytes here are the dump already in MAME as the Apple I's s2513.d2, and
// they are the right ones: reading the 64 dot matrices printed in Pictorial
// 4-4 of the H9 Operations manual off the page reproduces this file exactly,
// hash for hash.  Re-run that check with the font.py grid sampler if the ROM
// is ever replaced.
ROM_START( h9 )
	ROM_REGION( 0x0200, "chargen", 0 )
	ROM_LOAD( "443-812.ic205", 0x0000, 0x0200, CRC(a7e567fc) SHA1(b18aae0a2d4f92f5a7e22640719bbc4652f3f4ee) )
ROM_END

const tiny_rom_entry *heath_h9_device::device_rom_region() const
{
	return ROM_NAME( h9 );
}
