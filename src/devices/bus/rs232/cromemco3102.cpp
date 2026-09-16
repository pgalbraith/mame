// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

    Cromemco 3102 terminal

    The terminal every Z-2H shipped with. It is a Beehive Micro Bee 2 with
    Cromemco's name on it, and the manual Cromemco issued as part 023-6001
    is Beehive's own with the covers changed.

    THIS IS NOT THE REAL TERMINAL, AND HERE IS WHAT THAT MEANS
    ---------------------------------------------------------
    The 3102 is a microprocessor terminal: an 8085A, six masked 2K ROMs at
    0000-2FFF, eight 2142 RAMs at 8000-97FF, two 8255s, an 8253, two 8251s,
    an 8257 and an 8275 CRT controller. Every one of those is already a
    device in MAME and the technical manual has the schematics, so the board
    could be built here exactly. What cannot be built is the part that makes
    it work: none of the six ROMs has been dumped, and neither has the
    character generator, so there is no program for the 8085 to run.

    What this is instead is a program that behaves the way the manual says
    the terminal behaves. It keeps a screen of characters and acts on the
    control codes and escape sequences, and it does that without an 8085 or
    any firmware. Against a host it looks right; inside it is nothing like
    the real thing. Replace it with a real driver the day the ROMs turn up.

    The glyphs are MAME's own terminal font, from generic_terminal_device.
    They are not the 3102's, because that character generator is undumped
    too.

    WHAT IT DOES
    ------------
    Control codes: BEL, BS, HT, LF, CR, and ESC. Tab stops are fixed every
    eight columns. The alarm also sounds when the cursor passes column 72,
    which is what warns the operator that the line is running out.

    Escape sequences, all from sections 3.3.5 to 3.3.7 of the technical
    manual:

    ESC A   cursor up, wrapping to the bottom line
    ESC B   cursor down, wrapping to the top line
    ESC C   cursor right, wrapping to the next line and then to home
    ESC D   cursor left, wrapping back and then to the last position
    ESC H   home
    ESC E   clear the screen and home the cursor
    ESC K   erase to the end of the line
    ESC J   erase to the end of the page
    ESC F   address the cursor, as ESC F <line> <column>
    ESC Y   the same, which is the sequence the manual gives as the
            alternative
    ESC L   insert a line, pushing the rest of the screen down
    ESC M   delete a line, pulling the rest of the screen up
    ESC R   delete the character under the cursor
    ESC Q   turn insert character mode on
    ESC @   turn insert character mode off
    ESC G   send the character under the cursor to the host
    ESC \   send the cursor position to the host, as ESC F <line> <column>
    ESC 3   sound the alarm, which the real terminal holds on until ESC 4
    ESC 4   stop it

    Line and column are one based and go on the wire as the value plus 1FH,
    so line 1 is a space. The manual's own example addresses line 15 and
    column 41 with ESC F . H, and answers a cursor sense from line 5 column
    34 with ESC F $ A. An address outside the screen throws the whole
    sequence away.

    WHAT IT DOES NOT DO
    -------------------
    The visual attributes (reverse, blink, underline and half intensity),
    the logical field attributes and Forms mode, the status line, memory
    lock, the auxiliary port, the block send modes, the 16 function keys,
    X-ON/X-OFF flow control and the self test. The alarm sounds briefly
    rather than continuously, because generic_terminal_device owns the
    beeper and only offers the brief one. Attributes and the status
    line need a screen of this device's own, since generic_terminal_device
    stores characters and nothing else; the rest need a second serial port
    or a host that drives them.

    The manual contradicts itself over ESC N and ESC n. Table 3-2 says ESC N
    goes on line and ESC n goes local; section 3.3.9 says the opposite.
    Neither changes what is on the screen, so both are accepted and ignored.

    References
    - Cromemco 3102 Terminal Technical Manual, part 023-6001, March 1980,
      which is Beehive's Micro Bee 2 manual. Key functions in table 3-2,
      edit functions in 3.3.5, cursor movement in 3.3.6, screen erasure in
      3.3.7, cursor addressing and cursor sense on printed page 3-24, and
      the cursor address codes in table 3-8.
    - Cromemco 3102 Terminal User Manual, for the same sequences written for
      the operator.

***************************************************************************/

#include "emu.h"
#include "cromemco3102.h"

#include "machine/terminal.h"

#include <algorithm>


namespace {

class cromemco_3102_device : public generic_terminal_device,
	public device_buffered_serial_interface<16U>,
	public device_rs232_port_interface
{
public:
	cromemco_3102_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_txd(int state) override;

	void update_serial(int state);

protected:
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void term_write(uint8_t data) override;
	virtual void tra_callback() override;
	virtual void send_key(uint8_t code) override;

private:
	virtual void received_byte(uint8_t byte) override;

	// the screen, as the manual counts it: lines and columns from one
	static constexpr uint8_t ADDRESS_BIAS = 0x1f;

	// the column the alarm sounds at, warning of the end of the line
	static constexpr uint8_t WARN_COLUMN = 72;

	uint8_t *cell(uint8_t y, uint8_t x) const { return m_buffer.get() + (y * m_width) + x; }

	void cursor_up();
	void cursor_down();
	void cursor_left();
	void cursor_right();
	void erase_to_end_of_line();
	void erase_to_end_of_page();
	void insert_line();
	void delete_line();
	void delete_character();
	void insert_character(uint8_t data);
	void put_character(uint8_t data);
	void escape(uint8_t data);
	void send_string(const uint8_t *data, unsigned length);

	required_ioport m_rs232_txbaud;
	required_ioport m_rs232_databits;
	required_ioport m_rs232_parity;
	required_ioport m_rs232_stopbits;

	// 0 idle, 1 seen ESC, 2 seen ESC F or ESC Y, 3 seen the line
	uint8_t m_esc_state;
	uint8_t m_esc_line;

	bool m_insert_mode;
};

cromemco_3102_device::cromemco_3102_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: generic_terminal_device(mconfig, SERIAL_TERMINAL_C3102, tag, owner, clock, TERMINAL_WIDTH, TERMINAL_HEIGHT)
	, device_buffered_serial_interface(mconfig, *this)
	, device_rs232_port_interface(mconfig, *this)
	, m_rs232_txbaud(*this, "RS232_TXBAUD")
	, m_rs232_databits(*this, "RS232_DATABITS")
	, m_rs232_parity(*this, "RS232_PARITY")
	, m_rs232_stopbits(*this, "RS232_STOPBITS")
	, m_esc_state(0)
	, m_esc_line(0)
	, m_insert_mode(false)
{
}


//**************************************************************************
//  CURSOR MOVEMENT
//
//  Every one of these wraps. The manual is explicit about it: up from the
//  top line reaches the bottom, right from the last position reaches home.
//**************************************************************************

void cromemco_3102_device::cursor_up()
{
	m_y_pos = m_y_pos ? (m_y_pos - 1) : (m_height - 1);
}

void cromemco_3102_device::cursor_down()
{
	m_y_pos = (m_y_pos + 1 < m_height) ? (m_y_pos + 1) : 0;
}

void cromemco_3102_device::cursor_left()
{
	if (m_x_pos)
	{
		m_x_pos--;
		return;
	}

	m_x_pos = m_width - 1;
	cursor_up();
}

void cromemco_3102_device::cursor_right()
{
	if (m_x_pos + 1 < m_width)
	{
		m_x_pos++;

		if (m_x_pos == WARN_COLUMN)
			generic_terminal_device::term_write(0x07);

		return;
	}

	m_x_pos = 0;
	cursor_down();
}


//**************************************************************************
//  ERASING AND EDITING
//**************************************************************************

void cromemco_3102_device::erase_to_end_of_line()
{
	std::fill(cell(m_y_pos, m_x_pos), cell(m_y_pos, 0) + m_width, ' ');
}

void cromemco_3102_device::erase_to_end_of_page()
{
	std::fill(cell(m_y_pos, m_x_pos), m_buffer.get() + (m_width * m_height), ' ');
}

void cromemco_3102_device::insert_line()
{
	// the cursor line and everything below it move down one, and the line
	// that falls off the bottom is lost
	for (unsigned y = m_height - 1; y > m_y_pos; y--)
		std::copy_n(cell(y - 1, 0), m_width, cell(y, 0));

	std::fill_n(cell(m_y_pos, 0), m_width, ' ');
	m_x_pos = 0;
}

void cromemco_3102_device::delete_line()
{
	for (unsigned y = m_y_pos; y + 1 < m_height; y++)
		std::copy_n(cell(y + 1, 0), m_width, cell(y, 0));

	std::fill_n(cell(m_height - 1, 0), m_width, ' ');
}

void cromemco_3102_device::delete_character()
{
	// the rest of the line closes up and a space arrives at the end
	std::copy(cell(m_y_pos, m_x_pos) + 1, cell(m_y_pos, 0) + m_width, cell(m_y_pos, m_x_pos));
	*(cell(m_y_pos, 0) + m_width - 1) = ' ';
}

void cromemco_3102_device::insert_character(uint8_t data)
{
	// the rest of the line moves right and whatever reaches the end is lost
	std::copy_backward(cell(m_y_pos, m_x_pos), cell(m_y_pos, 0) + m_width - 1, cell(m_y_pos, 0) + m_width);
	*cell(m_y_pos, m_x_pos) = data;
	cursor_right();
}

void cromemco_3102_device::put_character(uint8_t data)
{
	if (m_insert_mode)
	{
		insert_character(data);
		return;
	}

	*cell(m_y_pos, m_x_pos) = data;
	cursor_right();
}


//**************************************************************************
//  THE COMMAND DECODER
//**************************************************************************

void cromemco_3102_device::send_string(const uint8_t *data, unsigned length)
{
	for (unsigned i = 0; i < length; i++)
		transmit_byte(data[i]);
}

void cromemco_3102_device::escape(uint8_t data)
{
	switch (data)
	{
	case 'A':
		cursor_up();
		break;
	case 'B':
		cursor_down();
		break;
	case 'C':
		cursor_right();
		break;
	case 'D':
		cursor_left();
		break;
	case 'H':
		m_x_pos = 0;
		m_y_pos = 0;
		break;

	case 'E':
		std::fill_n(m_buffer.get(), m_width * m_height, ' ');
		m_x_pos = 0;
		m_y_pos = 0;
		break;
	case 'K':
		erase_to_end_of_line();
		break;
	case 'J':
		erase_to_end_of_page();
		break;

	case 'L':
		insert_line();
		break;
	case 'M':
		delete_line();
		break;
	case 'R':
		delete_character();
		break;

	case 'Q':
		m_insert_mode = true;
		break;
	case '@':
		m_insert_mode = false;
		break;

	case 'F':
	case 'Y':
		m_esc_state = 2;
		return;

	case 'G':
		// read cursor character
		transmit_byte(*cell(m_y_pos, m_x_pos));
		break;

	case '\\':
		{
			// cursor sense, answered in the same form the host would use to
			// put the cursor back
			uint8_t const reply[4] =
			{
				0x1b, 'F',
				uint8_t(ADDRESS_BIAS + 1 + m_y_pos),
				uint8_t(ADDRESS_BIAS + 1 + m_x_pos)
			};

			send_string(reply, 4);
		}
		break;

	case '3':
	case '4':
		// ESC 3 starts a continuous alarm and ESC 4 stops it. The base class
		// owns the beeper and only knows how to sound it briefly, so this
		// sounds it once and leaves the stop with nothing to do.
		if (data == '3')
			generic_terminal_device::term_write(0x07);
		break;

	default:
		// everything this does not implement is swallowed, which is what the
		// real terminal does with a sequence it does not recognise
		break;
	}

	m_esc_state = 0;
}

void cromemco_3102_device::term_write(uint8_t data)
{
	switch (m_esc_state)
	{
	case 1:
		escape(data);
		return;

	case 2:
		// the line of a cursor address. Out of range throws the whole
		// sequence away rather than clamping.
		if ((data <= ADDRESS_BIAS) || ((data - ADDRESS_BIAS) > m_height))
		{
			m_esc_state = 0;
			return;
		}

		m_esc_line = data - ADDRESS_BIAS - 1;
		m_esc_state = 3;
		return;

	case 3:
		if ((data > ADDRESS_BIAS) && ((data - ADDRESS_BIAS) <= m_width))
		{
			m_y_pos = m_esc_line;
			m_x_pos = data - ADDRESS_BIAS - 1;
		}

		m_esc_state = 0;
		return;

	default:
		break;
	}

	switch (data)
	{
	case 0x00:
		// a pad code, sent after a carriage return to give the screen time
		break;

	case 0x08:
		cursor_left();
		break;

	case 0x1b:
		m_esc_state = 1;
		break;

	default:
		if (data >= 0x20 && data != 0x7f)
			put_character(data);
		else
			generic_terminal_device::term_write(data);
		break;
	}
}


//**************************************************************************
//  SERIAL PORT
//**************************************************************************

void cromemco_3102_device::input_txd(int state)
{
	device_buffered_serial_interface::rx_w(state);
}

void cromemco_3102_device::received_byte(uint8_t byte)
{
	term_write(byte & 0x7f);
}

void cromemco_3102_device::send_key(uint8_t code)
{
	transmit_byte(code);
}

void cromemco_3102_device::tra_callback()
{
	output_rxd(transmit_register_get_data_bit());
}

void cromemco_3102_device::update_serial(int state)
{
	clear_fifo();

	int const startbits = 1;
	int const databits = convert_databits(m_rs232_databits->read());
	parity_t const parity = convert_parity(m_rs232_parity->read());
	stop_bits_t const stopbits = convert_stopbits(m_rs232_stopbits->read());

	set_data_frame(startbits, databits, parity, stopbits);

	int const baud = convert_baud(m_rs232_txbaud->read());
	set_tra_rate(baud);
	set_rcv_rate(baud);

	output_rxd(1);

	// nothing here drives the handshake lines, so hold them asserted the way
	// a three wire cable would
	output_dcd(0);
	output_dsr(0);
	output_cts(0);

	receive_register_reset();
	transmit_register_reset();
}


//**************************************************************************
//  DEVICE
//**************************************************************************

static INPUT_PORTS_START(cromemco_3102)
	PORT_INCLUDE(generic_terminal)

	// The rear panel switches pick the main port rate. 9600 is what the Z-2H
	// installation manual has the operator start at.
	PORT_RS232_BAUD("RS232_TXBAUD", RS232_BAUD_9600, "Main Baud Rate", cromemco_3102_device, update_serial)
	PORT_RS232_DATABITS("RS232_DATABITS", RS232_DATABITS_8, "Data Bits", cromemco_3102_device, update_serial)
	PORT_RS232_PARITY("RS232_PARITY", RS232_PARITY_NONE, "Parity", cromemco_3102_device, update_serial)
	PORT_RS232_STOPBITS("RS232_STOPBITS", RS232_STOPBITS_1, "Stop Bits", cromemco_3102_device, update_serial)
INPUT_PORTS_END

ioport_constructor cromemco_3102_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cromemco_3102);
}

void cromemco_3102_device::device_start()
{
	generic_terminal_device::device_start();

	save_item(NAME(m_esc_state));
	save_item(NAME(m_esc_line));
	save_item(NAME(m_insert_mode));
}

void cromemco_3102_device::device_reset()
{
	generic_terminal_device::device_reset();

	m_esc_state = 0;
	m_esc_line = 0;
	m_insert_mode = false;

	update_serial(0);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SERIAL_TERMINAL_C3102, device_rs232_port_interface, cromemco_3102_device, "cromemco_3102", "Cromemco 3102 Terminal")
