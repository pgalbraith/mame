// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

    Teletype Model 33 ASR

    The terminal that minicomputers and the first microcomputers were sold
    with: a keyboard, a 72 column page printer, and a paper tape reader and
    punch, all at 110 baud. MITS sold it for the Altair 8800 as the 88-TTY.

    The line
    - 110 baud: a start bit, eight data bits and two stop bits, so ten
      characters a second.
    - The keyboard's eighth bit is even parity as Teletype shipped it, or
      wired always marking or always spacing. The printer ignores it.

    The printer
    - A 64 character type cylinder. Only code bits 1-5 and 7 choose the
      character, so the lower case codes print as upper case. Control codes
      and RUB OUT print nothing and do not move the carriage.
    - 72 columns. At the right margin the carriage stops and later
      characters print on top of the last one, until a carriage return.
    - A carriage return is a spring pulling the carriage back, and a
      character that arrives before it is home prints part way across the
      line. Teletype says a return needs two fill characters after it, so a
      return from the right margin takes two character times here. That
      figure comes from the fill character rule, not from a measurement.

    The keyboard
    - Upper case only. SHIFT and CTRL give the codes on Teletype's own code
      chart; the combinations the keyboard mechanically locks out send
      nothing.
    - REPT repeats the key held with it. BREAK holds the line spacing for as
      long as it is down. HERE IS sends the 20 character answer-back drum,
      which is blank here, so it punches two inches of blank tape.

    Controls
    - The LINE/LOCAL/OFF knob, full or half duplex, what the keyboard puts
      in the eighth bit, the two reader versions, and the automatic punch
      and answer-back options are all in Machine Configuration.
    - The reader lever and punch buttons are inputs, and buttons on the
      Teletype view.
    - The manual reader runs from START to STOP. The automatic reader also
      starts on DC1 and stops on DC3. Either stops when the tape runs out.
    - The punch turns on and off with its buttons, and on DC2 and off on DC4
      when that option is fitted, which Teletype shipped disabled.

    Paper tape
    - A tape image is raw bytes, one per punched frame.
    - The punch records what the printer receives. In LINE with full duplex
      that is what the computer sends; in LOCAL, or half duplex, it is the
      keyboard and the reader too. The one DC2 or DC4 that switches the
      punch is not punched when it turns it on, and is when it turns it off;
      no source says which it was.

    Not emulated
    - Sound, apart from the bell.
    - The automatic carriage return and line feed option, and the margin
      bell.
    - The reader's FREE position and the punch's REL button.
    - The reader and the keyboard garbling each other's characters when both
      are used at once; the keyboard is ignored while the reader runs.
    - A coded answer-back drum.

    References
    - Teletype Bulletin 310B, Model 33 Technical Manual, Volume 1, September
      1974 [https://bitsavers.org/communications/teletype/33/310B_Vol_1_33_Teletypewriter_Sets_Technical_Manual_Sep74.pdf]
      and Volume 2, February 1974 [https://bitsavers.org/communications/teletype/33/310B_Vol_2_33_Teletypewriter_Sets_Technical_Manual_Feb74.pdf]

***************************************************************************/

#include "emu.h"
#include "asr33.h"

#include "imagedev/papertape.h"
#include "sound/beep.h"

#include "screen.h"
#include "speaker.h"

#include "asr33.lh"


//**************************************************************************
//  PAPER TAPE READER AND PUNCH
//**************************************************************************

class asr33_tape_reader_device;
class asr33_tape_punch_device;

DECLARE_DEVICE_TYPE(ASR33_TAPE_READER, asr33_tape_reader_device)
DECLARE_DEVICE_TYPE(ASR33_TAPE_PUNCH, asr33_tape_punch_device)

class asr33_tape_reader_device : public paper_tape_reader_device
{
public:
	asr33_tape_reader_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0)
		: paper_tape_reader_device(mconfig, ASR33_TAPE_READER, tag, owner, clock)
	{
	}

	// a name of its own, so the reader and punch are -ptr and -ptp rather than numbered
	virtual const char *image_type_name() const noexcept override { return "tapereader"; }
	virtual const char *image_brief_type_name() const noexcept override { return "ptr"; }
	virtual const char *file_extensions() const noexcept override { return "tap,ptp"; }

	bool read(u8 &data) { return is_loaded() && (fread(&data, 1) == 1); }

protected:
	virtual void device_start() override ATTR_COLD { }
};

class asr33_tape_punch_device : public paper_tape_punch_device
{
public:
	asr33_tape_punch_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0)
		: paper_tape_punch_device(mconfig, ASR33_TAPE_PUNCH, tag, owner, clock)
	{
	}

	virtual const char *image_brief_type_name() const noexcept override { return "ptp"; }
	virtual const char *file_extensions() const noexcept override { return "tap,ptp"; }

	virtual std::pair<std::error_condition, std::string> call_load() override
	{
		// punching carries on at the end of the tape
		fseek(0, SEEK_END);
		m_start = ftell();
		m_holes.clear();
		return std::make_pair(std::error_condition(), std::string());
	}

	virtual std::pair<std::error_condition, std::string> call_create(int format_type, util::option_resolution *format_options) override
	{
		m_start = 0;
		m_holes.clear();
		return std::make_pair(std::error_condition(), std::string());
	}

	virtual void call_unload() override { m_holes.clear(); }

	void punch(u8 data)
	{
		if (!is_loaded())
			return;

		// A hole cannot be unpunched. After B.SP. the new character's holes
		// are added to the old ones, which is how RUB OUT obliterates a
		// mistake: it punches every hole.
		u64 const index = ftell() - m_start;
		if (index < m_holes.size())
			data = m_holes[index] |= data;
		else
			m_holes.push_back(data);
		fwrite(&data, 1);
	}

	void backspace()
	{
		if (is_loaded() && (ftell() > m_start))
			fseek(-1, SEEK_CUR);
	}

protected:
	virtual void device_start() override ATTR_COLD { }

private:
	std::vector<u8> m_holes;    // what has been punched since the tape was mounted
	u64 m_start = 0;            // where punching started in the image
};

DEFINE_DEVICE_TYPE(ASR33_TAPE_READER, asr33_tape_reader_device, "asr33_tape_reader", "Teletype Model 33 ASR Tape Reader")
DEFINE_DEVICE_TYPE(ASR33_TAPE_PUNCH, asr33_tape_punch_device, "asr33_tape_punch", "Teletype Model 33 ASR Tape Punch")


namespace {

//**************************************************************************
//  THE TELETYPE
//**************************************************************************

class asr33_device : public device_t, public device_serial_interface, public device_rs232_port_interface
{
public:
	asr33_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	virtual void input_txd(int state) override { rx_w(state); }

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_serial_interface implementation
	virtual void rcv_complete() override;
	virtual void tra_callback() override;
	virtual void tra_complete() override;

private:
	static constexpr unsigned COLUMNS = 72;
	static constexpr unsigned LINES = 32;           // lines of paper on show
	static constexpr unsigned STRIKES = 4;          // characters kept when printed on top of each other
	static constexpr unsigned CELL_WIDTH = 18;      // ten characters to the inch
	static constexpr unsigned CELL_HEIGHT = 30;     // six lines to the inch
	static constexpr unsigned MARGIN_X = 24;
	static constexpr unsigned MARGIN_Y = 12;
	static constexpr unsigned SCREEN_WIDTH = MARGIN_X * 2 + COLUMNS * CELL_WIDTH;
	static constexpr unsigned SCREEN_HEIGHT = 1008;
	static constexpr unsigned ANSWERBACK_LENGTH = 20;
	static constexpr unsigned QUEUE_LENGTH = 256;
	static constexpr unsigned KEY_HERE_IS = 12;
	static constexpr u8 LOCKED = 0xff;              // key cannot be pressed with these modifiers

	// a full return, right margin to left, takes two character times
	static constexpr double RETURN_SECONDS_PER_COLUMN = 0.2 / (COLUMNS - 1);

	// CONFIG port
	enum : u32
	{
		CONFIG_MODE = 0x03,
		CONFIG_HALF_DUPLEX = 0x04,
		CONFIG_EIGHTH_BIT = 0x18,
		CONFIG_AUTOMATIC_READER = 0x20,
		CONFIG_AUTOMATIC_PUNCH = 0x40,
		CONFIG_ANSWERBACK = 0x80
	};
	enum : u32 { MODE_LINE = 0, MODE_LOCAL, MODE_OFF };
	enum : u32 { EIGHTH_BIT_PARITY = 0x00, EIGHTH_BIT_MARK = 0x08, EIGHTH_BIT_SPACE = 0x10 };

	// CONTROLS port
	enum : u32
	{
		CONTROL_READER_START = 0x01,
		CONTROL_READER_STOP = 0x02,
		CONTROL_PUNCH_ON = 0x04,
		CONTROL_PUNCH_OFF = 0x08,
		CONTROL_PUNCH_BACKSPACE = 0x10
	};

	// MODIFIERS port
	enum : u32 { MOD_SHIFT = 0x01, MOD_CTRL = 0x02, MOD_REPT = 0x04, MOD_BREAK = 0x08 };

	struct key_codes { u8 plain, shift, ctrl, shift_ctrl; };
	static const key_codes KEY_CODES[48];
	static const u8 FONT[64][11];

	u32 mode() const { return m_config->read() & CONFIG_MODE; }

	TIMER_CALLBACK_MEMBER(scan);
	TIMER_CALLBACK_MEMBER(bell_off) { m_bell->set_state(0); }

	void key_pressed(unsigned key, u32 modifiers);
	u8 eighth_bit(u8 code) const;
	void queue(u8 data);
	void start_sending();
	void drive_line();
	void set_reader(bool running);

	void typing_unit(u8 data);
	void print(u8 ascii);
	void space();
	void carriage_return();
	void line_feed();
	unsigned carriage_column();

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect);

	required_device<screen_device> m_screen;
	required_device<beep_device> m_bell;
	required_device<asr33_tape_reader_device> m_reader;
	required_device<asr33_tape_punch_device> m_punch;
	required_ioport_array<2> m_keys;
	required_ioport m_modifiers;
	required_ioport m_controls;
	required_ioport m_config;

	emu_timer *m_scan_timer = nullptr;
	emu_timer *m_bell_timer = nullptr;

	// The paper on show, LINES by COLUMNS. Each place holds up to STRIKES
	// characters, seven bits apiece, as glyph number plus one. m_top is the
	// oldest line; the one being printed is the line above it, cyclically.
	std::unique_ptr<u32[]> m_paper;
	u32 m_line_number[LINES];   // counts line feeds, to vary the ink from line to line
	u32 m_lines_fed = 0;
	unsigned m_top = 0;
	unsigned m_column = 0;
	bool m_returning = false;
	unsigned m_return_from = 0;
	attotime m_return_start;

	// the distributor: the character going out, and those waiting
	u8 m_sending = 0;
	bool m_busy = false;
	int m_line_bit = 1;
	u8 m_queue[QUEUE_LENGTH];
	unsigned m_queue_head = 0;
	unsigned m_queue_count = 0;

	u32 m_key_state[2] = { 0, 0 };
	u32 m_last_controls = 0;
	int m_held_key = -1;        // last key pressed, while it is still down, for REPT
	u8 m_held_code = 0;
	bool m_break = false;
	bool m_reader_running = false;
	bool m_punch_on = false;

	u8 m_glyph_ink[64][CELL_HEIGHT][CELL_WIDTH];
};


// The type cylinder, as 7 by 11 dot patterns: nine rows of capital height
// and two below the line for the comma and semicolon. Drawn for this device
// in the manner of the Teletype's lettering, not traced from it. 0x5e and
// 0x5f are the up and left arrows the cylinder has in place of ^ and _.
const u8 asr33_device::FONT[64][11] =
{
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // space
	{ 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00 }, // !
	{ 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // "
	{ 0x14, 0x14, 0x7f, 0x14, 0x14, 0x14, 0x7f, 0x14, 0x14, 0x00, 0x00 }, // #
	{ 0x08, 0x3e, 0x48, 0x48, 0x3e, 0x09, 0x09, 0x3e, 0x08, 0x00, 0x00 }, // $
	{ 0x31, 0x4a, 0x32, 0x04, 0x08, 0x10, 0x26, 0x29, 0x46, 0x00, 0x00 }, // %
	{ 0x18, 0x24, 0x24, 0x18, 0x31, 0x4a, 0x44, 0x4a, 0x31, 0x00, 0x00 }, // &
	{ 0x08, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // '
	{ 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, 0x00, 0x00 }, // (
	{ 0x10, 0x08, 0x04, 0x04, 0x04, 0x04, 0x04, 0x08, 0x10, 0x00, 0x00 }, // )
	{ 0x00, 0x08, 0x49, 0x2a, 0x1c, 0x2a, 0x49, 0x08, 0x00, 0x00, 0x00 }, // *
	{ 0x00, 0x08, 0x08, 0x08, 0x7f, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00 }, // +
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x08, 0x10 }, // ,
	{ 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // -
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00 }, // .
	{ 0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x40, 0x00, 0x00 }, // /
	{ 0x1c, 0x22, 0x43, 0x45, 0x49, 0x51, 0x61, 0x22, 0x1c, 0x00, 0x00 }, // 0
	{ 0x08, 0x18, 0x28, 0x08, 0x08, 0x08, 0x08, 0x08, 0x3e, 0x00, 0x00 }, // 1
	{ 0x3e, 0x41, 0x01, 0x02, 0x0c, 0x10, 0x20, 0x40, 0x7f, 0x00, 0x00 }, // 2
	{ 0x7f, 0x02, 0x04, 0x08, 0x1c, 0x02, 0x01, 0x41, 0x3e, 0x00, 0x00 }, // 3
	{ 0x04, 0x0c, 0x14, 0x24, 0x44, 0x7f, 0x04, 0x04, 0x04, 0x00, 0x00 }, // 4
	{ 0x7f, 0x40, 0x40, 0x7e, 0x01, 0x01, 0x01, 0x41, 0x3e, 0x00, 0x00 }, // 5
	{ 0x1c, 0x20, 0x40, 0x40, 0x7e, 0x41, 0x41, 0x41, 0x3e, 0x00, 0x00 }, // 6
	{ 0x7f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00 }, // 7
	{ 0x3e, 0x41, 0x41, 0x41, 0x3e, 0x41, 0x41, 0x41, 0x3e, 0x00, 0x00 }, // 8
	{ 0x3e, 0x41, 0x41, 0x41, 0x3f, 0x01, 0x01, 0x02, 0x3c, 0x00, 0x00 }, // 9
	{ 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00 }, // :
	{ 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x08, 0x10, 0x00 }, // ;
	{ 0x02, 0x04, 0x08, 0x10, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00 }, // <
	{ 0x00, 0x00, 0x00, 0x7f, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00 }, // =
	{ 0x20, 0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00 }, // >
	{ 0x3e, 0x41, 0x01, 0x02, 0x04, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00 }, // ?
	{ 0x3e, 0x41, 0x4d, 0x55, 0x55, 0x4e, 0x40, 0x41, 0x3e, 0x00, 0x00 }, // @
	{ 0x08, 0x14, 0x22, 0x41, 0x41, 0x7f, 0x41, 0x41, 0x41, 0x00, 0x00 }, // A
	{ 0x7e, 0x41, 0x41, 0x41, 0x7e, 0x41, 0x41, 0x41, 0x7e, 0x00, 0x00 }, // B
	{ 0x3e, 0x41, 0x40, 0x40, 0x40, 0x40, 0x40, 0x41, 0x3e, 0x00, 0x00 }, // C
	{ 0x7c, 0x42, 0x41, 0x41, 0x41, 0x41, 0x41, 0x42, 0x7c, 0x00, 0x00 }, // D
	{ 0x7f, 0x40, 0x40, 0x40, 0x7c, 0x40, 0x40, 0x40, 0x7f, 0x00, 0x00 }, // E
	{ 0x7f, 0x40, 0x40, 0x40, 0x7c, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00 }, // F
	{ 0x3e, 0x41, 0x40, 0x40, 0x47, 0x41, 0x41, 0x41, 0x3e, 0x00, 0x00 }, // G
	{ 0x41, 0x41, 0x41, 0x41, 0x7f, 0x41, 0x41, 0x41, 0x41, 0x00, 0x00 }, // H
	{ 0x3e, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x3e, 0x00, 0x00 }, // I
	{ 0x07, 0x02, 0x02, 0x02, 0x02, 0x02, 0x42, 0x42, 0x3c, 0x00, 0x00 }, // J
	{ 0x41, 0x42, 0x44, 0x48, 0x70, 0x48, 0x44, 0x42, 0x41, 0x00, 0x00 }, // K
	{ 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7f, 0x00, 0x00 }, // L
	{ 0x41, 0x63, 0x55, 0x49, 0x49, 0x41, 0x41, 0x41, 0x41, 0x00, 0x00 }, // M
	{ 0x41, 0x61, 0x61, 0x51, 0x49, 0x45, 0x43, 0x43, 0x41, 0x00, 0x00 }, // N
	{ 0x3e, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x3e, 0x00, 0x00 }, // O
	{ 0x7e, 0x41, 0x41, 0x41, 0x7e, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00 }, // P
	{ 0x3e, 0x41, 0x41, 0x41, 0x41, 0x45, 0x42, 0x43, 0x3d, 0x00, 0x00 }, // Q
	{ 0x7e, 0x41, 0x41, 0x41, 0x7e, 0x48, 0x44, 0x42, 0x41, 0x00, 0x00 }, // R
	{ 0x3e, 0x41, 0x40, 0x40, 0x3e, 0x01, 0x01, 0x41, 0x3e, 0x00, 0x00 }, // S
	{ 0x7f, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00 }, // T
	{ 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x3e, 0x00, 0x00 }, // U
	{ 0x41, 0x41, 0x41, 0x22, 0x22, 0x22, 0x14, 0x14, 0x08, 0x00, 0x00 }, // V
	{ 0x41, 0x41, 0x41, 0x49, 0x49, 0x49, 0x55, 0x63, 0x41, 0x00, 0x00 }, // W
	{ 0x41, 0x41, 0x22, 0x14, 0x08, 0x14, 0x22, 0x41, 0x41, 0x00, 0x00 }, // X
	{ 0x41, 0x41, 0x22, 0x14, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00 }, // Y
	{ 0x7f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x7f, 0x00, 0x00 }, // Z
	{ 0x1e, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1e, 0x00, 0x00 }, // [
	{ 0x40, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0x01, 0x00, 0x00 }, // backslash
	{ 0x3c, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x3c, 0x00, 0x00 }, // ]
	{ 0x08, 0x1c, 0x2a, 0x49, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00 }, // up arrow
	{ 0x00, 0x00, 0x10, 0x20, 0x7f, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00 }, // left arrow
};


// What each key sends, plain, with SHIFT, with CTRL, and with both, from
// Figure 3 of Bulletin 310B Volume 1 section 574-121-100. The order is the
// order of the keys in the KEYS0 and KEYS1 ports: the four rows, left to
// right, with the modifiers, REPT and BREAK in their own port.
#define LETTER(c)              { u8(c), LOCKED, u8((c) & 0x1f), LOCKED }
#define SHIFTED(c, s, sc)      { u8(c), u8(s), u8((c) & 0x1f), u8(sc) }
#define SYMBOL(c, s)           { u8(c), u8(s), LOCKED, LOCKED }
#define ANY(c)                 { u8(c), u8(c), u8(c), u8(c) }

const asr33_device::key_codes asr33_device::KEY_CODES[48] =
{
	// 1 to 0, :, -, HERE IS
	SYMBOL('1', '!'), SYMBOL('2', '"'), SYMBOL('3', '#'), SYMBOL('4', '$'), SYMBOL('5', '%'),
	SYMBOL('6', '&'), SYMBOL('7', '\''), SYMBOL('8', '('), SYMBOL('9', ')'), SYMBOL('0', LOCKED),
	SYMBOL(':', '*'), SYMBOL('-', '='), { LOCKED, LOCKED, LOCKED, LOCKED },

	// ESC, Q to P, LINE FEED, RETURN
	ANY(0x1b),
	LETTER('Q'), LETTER('W'), LETTER('E'), LETTER('R'), LETTER('T'), LETTER('Y'), LETTER('U'), LETTER('I'),
	SHIFTED('O', 0x5f, 0x1f), SHIFTED('P', '@', 0x00),
	ANY(0x0a), ANY(0x0d),

	// A to L, ;, RUB OUT
	LETTER('A'), LETTER('S'), LETTER('D'), LETTER('F'), LETTER('G'), LETTER('H'), LETTER('J'),
	SHIFTED('K', '[', 0x1b), SHIFTED('L', '\\', 0x1c),
	SYMBOL(';', '+'), ANY(0x7f),

	// Z to M, comma, period, slash, space bar
	LETTER('Z'), LETTER('X'), LETTER('C'), LETTER('V'), LETTER('B'),
	SHIFTED('N', 0x5e, 0x1e), SHIFTED('M', ']', 0x1d),
	SYMBOL(',', '<'), SYMBOL('.', '>'), SYMBOL('/', '?'), SYMBOL(' ', ' ')
};

#undef LETTER
#undef SHIFTED
#undef SYMBOL
#undef ANY


asr33_device::asr33_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SERIAL_TERMINAL_ASR33, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, device_rs232_port_interface(mconfig, *this)
	, m_screen(*this, "screen")
	, m_bell(*this, "bell")
	, m_reader(*this, "reader")
	, m_punch(*this, "punch")
	, m_keys(*this, "KEYS%u", 0U)
	, m_modifiers(*this, "MODIFIERS")
	, m_controls(*this, "CONTROLS")
	, m_config(*this, "CONFIG")
	, m_line_number{ }
	, m_queue{ }
	, m_glyph_ink{ }
{
}


void asr33_device::device_start()
{
	m_paper = make_unique_clear<u32[]>(LINES * COLUMNS);

	// Each dot of the type pattern inks a 2 by 2 square, and the ribbon
	// spreads a lighter pixel around the edge of every stroke.
	for (unsigned glyph = 0; glyph < 64; glyph++)
	{
		auto &ink = m_glyph_ink[glyph];
		for (unsigned row = 0; row < 11; row++)
			for (unsigned col = 0; col < 7; col++)
				if (BIT(FONT[glyph][row], 6 - col))
					for (unsigned y = 0; y < 2; y++)
						for (unsigned x = 0; x < 2; x++)
							ink[4 + row * 2 + y][2 + col * 2 + x] = 255;

		for (unsigned y = 0; y < CELL_HEIGHT; y++)
			for (unsigned x = 0; x < CELL_WIDTH; x++)
				if (ink[y][x] == 0)
				{
					bool const edge =
							((x > 0) && (ink[y][x - 1] == 255)) || ((x + 1 < CELL_WIDTH) && (ink[y][x + 1] == 255)) ||
							((y > 0) && (ink[y - 1][x] == 255)) || ((y + 1 < CELL_HEIGHT) && (ink[y + 1][x] == 255));
					if (edge)
						ink[y][x] = 100;
				}
	}

	m_scan_timer = timer_alloc(FUNC(asr33_device::scan), this);
	m_scan_timer->adjust(attotime::from_msec(10), 0, attotime::from_msec(10));
	m_bell_timer = timer_alloc(FUNC(asr33_device::bell_off), this);

	save_pointer(NAME(m_paper), LINES * COLUMNS);
	save_item(NAME(m_line_number));
	save_item(NAME(m_lines_fed));
	save_item(NAME(m_top));
	save_item(NAME(m_column));
	save_item(NAME(m_returning));
	save_item(NAME(m_return_from));
	save_item(NAME(m_return_start));
	save_item(NAME(m_sending));
	save_item(NAME(m_busy));
	save_item(NAME(m_line_bit));
	save_item(NAME(m_queue));
	save_item(NAME(m_queue_head));
	save_item(NAME(m_queue_count));
	save_item(NAME(m_key_state));
	save_item(NAME(m_last_controls));
	save_item(NAME(m_held_key));
	save_item(NAME(m_held_code));
	save_item(NAME(m_break));
	save_item(NAME(m_reader_running));
	save_item(NAME(m_punch_on));
}

void asr33_device::device_reset()
{
	// The Teletype is its own machine, and the computer resetting does not
	// reset it. Only the line interface starts over.
	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_2);
	set_rcv_rate(110);
	set_tra_rate(110);
	receive_register_reset();
	transmit_register_reset();
	m_busy = false;
	m_line_bit = 1;
	m_queue_count = 0;

	// a current loop has no handshaking, so hold the RS-232 lines asserted
	drive_line();
	output_dcd(0);
	output_dsr(0);
	output_cts(0);
}


//**************************************************************************
//  KEYBOARD AND CONTROLS
//**************************************************************************

TIMER_CALLBACK_MEMBER(asr33_device::scan)
{
	u32 const modifiers = m_modifiers->read();
	bool const off = mode() == MODE_OFF;

	bool const held_break = BIT(modifiers, 3) && !off;
	if (held_break != m_break)
	{
		m_break = held_break;
		drive_line();
	}

	u32 const controls = m_controls->read();
	u32 const pressed = controls & ~m_last_controls;
	m_last_controls = controls;

	if (off)
	{
		set_reader(false);
	}
	else
	{
		if (pressed & CONTROL_READER_START)
			set_reader(true);
		if (pressed & CONTROL_READER_STOP)
			set_reader(false);
		if (pressed & CONTROL_PUNCH_ON)
			m_punch_on = true;
		if (pressed & CONTROL_PUNCH_OFF)
			m_punch_on = false;
		if (pressed & CONTROL_PUNCH_BACKSPACE)
			m_punch->backspace();
	}

	for (unsigned port = 0; port < 2; port++)
	{
		u32 const state = m_keys[port]->read();
		u32 const down = state & ~m_key_state[port];
		m_key_state[port] = state;
		for (unsigned bit = 0; bit < 32; bit++)
			if (BIT(down, bit))
				key_pressed(port * 32 + bit, modifiers);
	}

	if ((m_held_key >= 0) && !BIT(m_key_state[m_held_key / 32], m_held_key % 32))
		m_held_key = -1;
}

void asr33_device::key_pressed(unsigned key, u32 modifiers)
{
	// the keyboard and the reader share the distributor
	if ((mode() == MODE_OFF) || m_reader_running || (key >= std::size(KEY_CODES)))
		return;

	if (key == KEY_HERE_IS)
	{
		// the answer-back drum, uncoded: every tine left in, so nothing but spacing
		for (unsigned i = 0; i < ANSWERBACK_LENGTH; i++)
			queue(0x00);
		return;
	}

	key_codes const &codes = KEY_CODES[key];
	bool const shift = modifiers & MOD_SHIFT;
	bool const ctrl = modifiers & MOD_CTRL;
	u8 const code = ctrl ? (shift ? codes.shift_ctrl : codes.ctrl) : (shift ? codes.shift : codes.plain);
	if (code == LOCKED)
		return;

	m_held_key = key;
	m_held_code = eighth_bit(code);
	queue(m_held_code);
}

u8 asr33_device::eighth_bit(u8 code) const
{
	switch (m_config->read() & CONFIG_EIGHTH_BIT)
	{
	case EIGHTH_BIT_MARK:
		return code | 0x80;
	case EIGHTH_BIT_SPACE:
		return code;
	default:
		// 0x6996 holds the parity of each 4 bit value, bit n for value n
		return BIT(0x6996, (code ^ (code >> 4)) & 0x0f) ? (code | 0x80) : code;
	}
}

void asr33_device::set_reader(bool running)
{
	m_reader_running = running && m_reader->exists();
	if (m_reader_running)
		start_sending();
}


//**************************************************************************
//  DISTRIBUTOR - SENDING
//**************************************************************************

void asr33_device::queue(u8 data)
{
	if (m_queue_count < QUEUE_LENGTH)
	{
		m_queue[(m_queue_head + m_queue_count) % QUEUE_LENGTH] = data;
		m_queue_count++;
	}
	start_sending();
}

void asr33_device::start_sending()
{
	if (m_busy)
		return;

	u8 data;
	if (m_queue_count)
	{
		data = m_queue[m_queue_head];
		m_queue_head = (m_queue_head + 1) % QUEUE_LENGTH;
		m_queue_count--;
	}
	else if (m_reader_running)
	{
		if (!m_reader->read(data))
		{
			m_reader_running = false;  // out of tape
			return;
		}
	}
	else
	{
		return;
	}

	m_sending = data;
	m_busy = true;
	transmit_register_setup(data);
}

void asr33_device::drive_line()
{
	// LOCAL shunts the line, so the computer sees it idle
	if (mode() != MODE_LINE)
		output_rxd(1);
	else
		output_rxd(m_break ? 0 : m_line_bit);
}

void asr33_device::tra_callback()
{
	m_line_bit = transmit_register_get_data_bit();
	drive_line();
}

void asr33_device::tra_complete()
{
	m_busy = false;
	m_line_bit = 1;

	// In LOCAL, or wired for half duplex, the distributor also feeds the
	// typing unit; in full duplex the computer's echo is what prints.
	u32 const current_mode = mode();
	if ((current_mode == MODE_LOCAL) || ((current_mode == MODE_LINE) && (m_config->read() & CONFIG_HALF_DUPLEX)))
		typing_unit(m_sending);

	// REPT sends the held key again once each character is out
	bool const repeat = (m_held_key >= 0) && (m_modifiers->read() & MOD_REPT) && !m_reader_running;
	if (!m_queue_count && repeat)
		queue(m_held_code);
	else
		start_sending();
}


//**************************************************************************
//  TYPING UNIT - PRINTING
//**************************************************************************

void asr33_device::rcv_complete()
{
	receive_register_extract();
	if (mode() == MODE_LINE)
		typing_unit(get_received_char());
}

void asr33_device::typing_unit(u8 data)
{
	// the punch copies all eight levels of whatever reaches the typing unit
	if (m_punch_on)
		m_punch->punch(data);

	u32 const config = m_config->read();
	u8 const code = data & 0x7f;
	switch (code)
	{
	case 0x05: // ENQ calls for the answer-back, if the set answers
		if (config & CONFIG_ANSWERBACK)
		{
			set_reader(false);
			for (unsigned i = 0; i < ANSWERBACK_LENGTH; i++)
				queue(0x00);
		}
		break;

	case 0x07:
		m_bell->set_state(1);
		m_bell_timer->adjust(attotime::from_msec(150));
		break;

	case 0x0a:
		line_feed();
		break;

	case 0x0d:
		carriage_return();
		break;

	case 0x11: // DC1, reader on
		if (config & CONFIG_AUTOMATIC_READER)
			set_reader(true);
		break;

	case 0x12: // DC2, punch on
		if (config & CONFIG_AUTOMATIC_PUNCH)
			m_punch_on = true;
		break;

	case 0x13: // DC3, reader off
		if (config & CONFIG_AUTOMATIC_READER)
			set_reader(false);
		break;

	case 0x14: // DC4, punch off
		if (config & CONFIG_AUTOMATIC_PUNCH)
			m_punch_on = false;
		break;

	case 0x20:
		space();
		break;

	default:
		// The cylinder is chosen by bits 1-5 and 7, and with both 6 and 7
		// spacing nothing is selected. So the control codes print nothing,
		// and the lower case columns print as the upper case ones.
		if ((code & 0x60) && (code != 0x7f))
			print((code & 0x40) ? (0x40 | (code & 0x1f)) : (0x20 | (code & 0x1f)));
		break;
	}
}

unsigned asr33_device::carriage_column()
{
	if (m_returning)
	{
		double const travelled = (machine().time() - m_return_start).as_double() / RETURN_SECONDS_PER_COLUMN;
		if (travelled < m_return_from)
			return m_return_from - unsigned(travelled);
		m_returning = false;
	}
	return m_column;
}

void asr33_device::print(u8 ascii)
{
	unsigned const line = (m_top + LINES - 1) % LINES;
	u32 &place = m_paper[line * COLUMNS + carriage_column()];
	u32 const glyph = ascii - 0x20 + 1;

	unsigned strike = 0;
	while ((strike < STRIKES - 1) && ((place >> (7 * strike)) & 0x7f))
		strike++;
	place = (place & ~(u32(0x7f) << (7 * strike))) | (glyph << (7 * strike));

	space();
}

void asr33_device::space()
{
	// no spacing while the carriage is returning, or at the right margin
	if (!m_returning && (m_column < COLUMNS - 1))
		m_column++;
}

void asr33_device::carriage_return()
{
	unsigned const from = carriage_column();
	m_column = 0;
	m_returning = from != 0;
	m_return_from = from;
	m_return_start = machine().time();
}

void asr33_device::line_feed()
{
	unsigned const line = m_top;
	m_top = (m_top + 1) % LINES;
	std::fill_n(&m_paper[line * COLUMNS], COLUMNS, 0);
	m_line_number[line] = ++m_lines_fed;
}


//**************************************************************************
//  PAPER
//**************************************************************************

u32 asr33_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect)
{
	static constexpr int PAPER[3] = { 247, 243, 228 };
	static constexpr int INK[3] = { 40, 36, 64 };

	bitmap.fill(rgb_t(PAPER[0], PAPER[1], PAPER[2]), cliprect);

	for (unsigned row = 0; row < LINES; row++)
	{
		unsigned const line = (m_top + row) % LINES;
		int const top = MARGIN_Y + row * CELL_HEIGHT;

		for (unsigned col = 0; col < COLUMNS; col++)
		{
			u32 const place = m_paper[line * COLUMNS + col];
			if (!place)
				continue;

			// Add up the ink of every character struck here. Each strike is a
			// little lighter or darker than the last, as a worn ribbon gives.
			u16 cover[CELL_HEIGHT][CELL_WIDTH] = { };
			for (unsigned strike = 0; strike < STRIKES; strike++)
			{
				u32 const glyph = (place >> (7 * strike)) & 0x7f;
				if (!glyph)
					break;

				u32 hash = (m_line_number[line] * 2654435761U) ^ (col * 40503U) ^ (strike * 9973U);
				hash ^= hash >> 15;
				u32 const strength = 205 + (hash % 51);

				auto const &ink = m_glyph_ink[glyph - 1];
				for (unsigned y = 0; y < CELL_HEIGHT; y++)
					for (unsigned x = 0; x < CELL_WIDTH; x++)
						cover[y][x] = std::min<u32>(255, cover[y][x] + ink[y][x] * strength / 255);
			}

			int const left = MARGIN_X + col * CELL_WIDTH;
			for (unsigned y = 0; y < CELL_HEIGHT; y++)
			{
				int const py = top + y;
				if ((py < cliprect.top()) || (py > cliprect.bottom()))
					continue;
				for (unsigned x = 0; x < CELL_WIDTH; x++)
				{
					int const px = left + x;
					u32 const c = cover[y][x];
					if (c && (px >= cliprect.left()) && (px <= cliprect.right()))
						bitmap.pix(py, px) = rgb_t(
								PAPER[0] - (PAPER[0] - INK[0]) * c / 255,
								PAPER[1] - (PAPER[1] - INK[1]) * c / 255,
								PAPER[2] - (PAPER[2] - INK[2]) * c / 255);
				}
			}
		}
	}

	// a small mark under the column the typebox is at
	int const mark_x = MARGIN_X + carriage_column() * CELL_WIDTH;
	int const mark_y = MARGIN_Y + LINES * CELL_HEIGHT + 2;
	rectangle mark(mark_x + 3, mark_x + CELL_WIDTH - 4, mark_y, mark_y + 4);
	mark &= cliprect;
	if (!mark.empty())
		bitmap.fill(rgb_t(170, 166, 156), mark);

	return 0;
}


//**************************************************************************
//  CONFIGURATION
//**************************************************************************

#define ASR33_KEY(bit, name, code) \
	PORT_BIT(1U << (bit), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(name) PORT_CODE(code)

static INPUT_PORTS_START( asr33 )
	PORT_START("KEYS0")
	ASR33_KEY( 0, "1  !", KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!')
	ASR33_KEY( 1, "2  \"", KEYCODE_2) PORT_CHAR('2') PORT_CHAR('"')
	ASR33_KEY( 2, "3  #", KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#')
	ASR33_KEY( 3, "4  $", KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$')
	ASR33_KEY( 4, "5  %", KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%')
	ASR33_KEY( 5, "6  &", KEYCODE_6) PORT_CHAR('6') PORT_CHAR('&')
	ASR33_KEY( 6, "7  '", KEYCODE_7) PORT_CHAR('7') PORT_CHAR('\'')
	ASR33_KEY( 7, "8  (", KEYCODE_8) PORT_CHAR('8') PORT_CHAR('(')
	ASR33_KEY( 8, "9  )", KEYCODE_9) PORT_CHAR('9') PORT_CHAR(')')
	ASR33_KEY( 9, "0", KEYCODE_0) PORT_CHAR('0')
	ASR33_KEY(10, ":  *", KEYCODE_MINUS) PORT_CHAR(':') PORT_CHAR('*')
	ASR33_KEY(11, "-  =", KEYCODE_EQUALS) PORT_CHAR('-') PORT_CHAR('=')
	ASR33_KEY(12, "HERE IS", KEYCODE_BACKSPACE)
	ASR33_KEY(13, "ESC", KEYCODE_TAB) PORT_CHAR(27)
	ASR33_KEY(14, "Q", KEYCODE_Q) PORT_CHAR('Q')
	ASR33_KEY(15, "W", KEYCODE_W) PORT_CHAR('W')
	ASR33_KEY(16, "E", KEYCODE_E) PORT_CHAR('E')
	ASR33_KEY(17, "R", KEYCODE_R) PORT_CHAR('R')
	ASR33_KEY(18, "T", KEYCODE_T) PORT_CHAR('T')
	ASR33_KEY(19, "Y", KEYCODE_Y) PORT_CHAR('Y')
	ASR33_KEY(20, "U", KEYCODE_U) PORT_CHAR('U')
	ASR33_KEY(21, "I", KEYCODE_I) PORT_CHAR('I')
	ASR33_KEY(22, "O  \xe2\x86\x90", KEYCODE_O) PORT_CHAR('O') PORT_CHAR('_')
	ASR33_KEY(23, "P  @", KEYCODE_P) PORT_CHAR('P') PORT_CHAR('@')
	ASR33_KEY(24, "LINE FEED", KEYCODE_OPENBRACE) PORT_CHAR(10)
	ASR33_KEY(25, "RETURN", KEYCODE_ENTER) PORT_CHAR(13)
	ASR33_KEY(26, "A", KEYCODE_A) PORT_CHAR('A')
	ASR33_KEY(27, "S", KEYCODE_S) PORT_CHAR('S')
	ASR33_KEY(28, "D", KEYCODE_D) PORT_CHAR('D')
	ASR33_KEY(29, "F", KEYCODE_F) PORT_CHAR('F')
	ASR33_KEY(30, "G", KEYCODE_G) PORT_CHAR('G')
	ASR33_KEY(31, "H", KEYCODE_H) PORT_CHAR('H')

	PORT_START("KEYS1")
	ASR33_KEY( 0, "J", KEYCODE_J) PORT_CHAR('J')
	ASR33_KEY( 1, "K  [", KEYCODE_K) PORT_CHAR('K') PORT_CHAR('[')
	ASR33_KEY( 2, "L  \\", KEYCODE_L) PORT_CHAR('L') PORT_CHAR('\\')
	ASR33_KEY( 3, ";  +", KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR('+')
	ASR33_KEY( 4, "RUB OUT", KEYCODE_QUOTE) PORT_CHAR(127)
	ASR33_KEY( 5, "Z", KEYCODE_Z) PORT_CHAR('Z')
	ASR33_KEY( 6, "X", KEYCODE_X) PORT_CHAR('X')
	ASR33_KEY( 7, "C", KEYCODE_C) PORT_CHAR('C')
	ASR33_KEY( 8, "V", KEYCODE_V) PORT_CHAR('V')
	ASR33_KEY( 9, "B", KEYCODE_B) PORT_CHAR('B')
	ASR33_KEY(10, "N  \xe2\x86\x91", KEYCODE_N) PORT_CHAR('N') PORT_CHAR('^')
	ASR33_KEY(11, "M  ]", KEYCODE_M) PORT_CHAR('M') PORT_CHAR(']')
	ASR33_KEY(12, ",  <", KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<')
	ASR33_KEY(13, ".  >", KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>')
	ASR33_KEY(14, "/  ?", KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?')
	ASR33_KEY(15, "SPACE", KEYCODE_SPACE) PORT_CHAR(' ')

	PORT_START("MODIFIERS")
	ASR33_KEY(0, "SHIFT", KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
	ASR33_KEY(1, "CTRL", KEYCODE_LCONTROL) PORT_CODE(KEYCODE_RCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	ASR33_KEY(2, "REPT", KEYCODE_RALT)
	ASR33_KEY(3, "BREAK", KEYCODE_PAUSE)

	PORT_START("CONTROLS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Tape Reader START")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Tape Reader STOP")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Tape Punch ON")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Tape Punch OFF")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Tape Punch B.SP.")

	PORT_START("CONFIG")
	PORT_CONFNAME(0x03, 0x00, "Mode")
	PORT_CONFSETTING(0x00, "LINE")
	PORT_CONFSETTING(0x01, "LOCAL")
	PORT_CONFSETTING(0x02, "OFF")
	PORT_CONFNAME(0x04, 0x00, "Duplex")
	PORT_CONFSETTING(0x00, "Full")
	PORT_CONFSETTING(0x04, "Half")
	PORT_CONFNAME(0x18, 0x00, "Keyboard eighth bit")
	PORT_CONFSETTING(0x00, "Even parity")
	PORT_CONFSETTING(0x08, "Always marking")
	PORT_CONFSETTING(0x10, "Always spacing")
	PORT_CONFNAME(0x20, 0x00, "Tape reader")
	PORT_CONFSETTING(0x00, "Manual")
	PORT_CONFSETTING(0x20, "Automatic (DC1 and DC3)")
	PORT_CONFNAME(0x40, 0x00, "Tape punch")
	PORT_CONFSETTING(0x00, "Buttons only")
	PORT_CONFSETTING(0x40, "Also DC2 and DC4")
	PORT_CONFNAME(0x80, 0x00, "Answer-back on ENQ")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x80, DEF_STR(On))
INPUT_PORTS_END

#undef ASR33_KEY

ioport_constructor asr33_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(asr33);
}

void asr33_device::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen);
	m_screen->set_refresh_hz(50);
	m_screen->set_size(SCREEN_WIDTH, SCREEN_HEIGHT);
	m_screen->set_visarea(0, SCREEN_WIDTH - 1, 0, SCREEN_HEIGHT - 1);
	m_screen->set_screen_update(FUNC(asr33_device::screen_update));

	SPEAKER(config, "speaker").front_center();
	BEEP(config, m_bell, 1'600).add_route(ALL_OUTPUTS, "speaker", 0.25);

	ASR33_TAPE_READER(config, m_reader);
	ASR33_TAPE_PUNCH(config, m_punch);

	config.set_default_layout(layout_asr33);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SERIAL_TERMINAL_ASR33, device_rs232_port_interface, asr33_device, "teletype_asr33", "Teletype Model 33 ASR")
