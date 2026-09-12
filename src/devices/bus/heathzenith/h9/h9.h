// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heathkit H9 Video Terminal

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H9_H9_H
#define MAME_BUS_HEATHZENITH_H9_H9_H

#pragma once

#include "machine/keyboard.h"
#include "sound/beep.h"

#include "diserial.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"


class heath_h9_device : public device_t
					  , public device_serial_interface
					  , protected device_matrix_keyboard_interface<5U>
{
public:
	heath_h9_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// Serial data out of the terminal.  There is nothing else to connect: the
	// H8 cable carries Data+, Data- and ground and no handshaking at all (H9
	// Operations, Pictorial 2-3).
	auto serial_data_callback() { return m_write_sd.bind(); }

	void serial_in_w(int state);

	// Stand in for the rate jumper on the I/O circuit board, for a machine
	// whose console runs at a rate the board was not shipped wired for.  The
	// BAUD RATE key and the rear panel switch still sit on top of this: the key
	// released is 110 baud whatever is set here, and the switch can still pick
	// 300.  Zero, the default, means use the jumper in the JUMPERS port.
	void set_preset_baud(u32 rate) { m_preset_baud = rate; }

	DECLARE_INPUT_CHANGED_MEMBER(break_key);
	DECLARE_INPUT_CHANGED_MEMBER(scroll_key_changed);
	DECLARE_INPUT_CHANGED_MEMBER(baud_changed);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

	// device_serial_interface
	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

	// device_matrix_keyboard_interface
	virtual void key_make(u8 row, u8 column) override;
	virtual void key_repeat(u8 row, u8 column) override;
	virtual void key_break(u8 row, u8 column) override;

private:
	// The serial output queue only ever has to hold one page, which is what
	// XMIT PAGE loads into it in one go.
	static constexpr u16 TX_FIFO_SIZE = 2048;

	// front panel switch states, all read live from the ioports
	bool short_form() const;
	bool auto_carry() const;
	bool scroll_key() const;
	bool plot_mode() const;
	bool full_duplex() const;
	bool off_line() const;
	u32  baud_rate() const;

	// RAM and the three cursor counters
	u16  ram_addr(u8 line, u8 block, u8 chr) const { return (line * 80) + (block * 20) + chr; }
	u16  cursor_addr() const { return ram_addr(m_c12, m_c4, m_c20); }
	void write_ram(u8 data) { m_ram[cursor_addr()] = data; }
	u8   read_ram() const { return m_ram[cursor_addr()]; }

	// the TPU's housekeeping
	void handle_char(u8 data, bool from_keyboard);
	void advance_cursor();
	void cursor_left();
	void cursor_down();
	void check_scroll();
	void do_scroll();
	void erase_line();
	void erase_to_end_of_line();
	void erase_page();
	void home_cursor();
	void ring_bell();
	void send_char(u8 data);
	void key_char(u8 data);
	void start_transmit_page();

	TIMER_CALLBACK_MEMBER(bell_off);

	// video
	u32  screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_dots(bitmap_ind16 &bitmap, const rectangle &cliprect, int x, int y, u8 dots);
	u8   glyph_row(u8 chr, u8 row) const;

	required_device<screen_device>   m_screen;
	required_device<palette_device>  m_palette;
	required_device<beep_device>     m_bell;
	required_memory_region           m_chargen;
	required_ioport                  m_modifiers;
	required_ioport                  m_modes;
	required_ioport                  m_jumpers;

	emu_timer *m_bell_timer = nullptr;

	u8  m_ram[1024];

	// cursor: character within a 20-character block, block, and line
	u8  m_c20 = 0;
	u8  m_c4 = 0;
	u8  m_c12 = 0;

	// start of page: which line is displayed at the top (long form) and which
	// block at the left (short form)
	u8  m_scroll12 = 0;
	u8  m_scroll4 = 0;

	bool m_hold_screen = false;

	u32 m_preset_baud = 0;

	u8  m_tx_fifo[TX_FIFO_SIZE];
	u16 m_tx_head = 0;
	u16 m_tx_tail = 0;
	bool m_tx_busy = false;

	devcb_write_line m_write_sd;
};

DECLARE_DEVICE_TYPE(HEATH_H9, heath_h9_device)

#endif // MAME_BUS_HEATHZENITH_H9_H9_H
