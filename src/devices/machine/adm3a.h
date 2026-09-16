// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Lear Siegler ADM-3A Dumb Terminal

****************************************************************************/

#ifndef MAME_MACHINE_ADM3A_H
#define MAME_MACHINE_ADM3A_H

#pragma once

#include "machine/keyboard.h"
#include "sound/beep.h"

#include "diserial.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"


class adm3a_device : public device_t
				   , public device_serial_interface
				   , protected device_matrix_keyboard_interface<5U>
{
public:
	adm3a_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// Serial data out of the terminal, pin 2 of the MODEM connector.
	auto serial_data_callback() { return m_write_sd.bind(); }

	void serial_in_w(int state);

	// Stand in for the S1/S2 baud rate switches, for a machine whose console
	// runs at a rate the terminal would have to be opened up to match.  Zero,
	// the default, means use the BAUD port.
	void set_preset_baud(u32 rate) { m_preset_baud = rate; }

	DECLARE_INPUT_CHANGED_MEMBER(break_key);
	DECLARE_INPUT_CHANGED_MEMBER(clear_key);
	DECLARE_INPUT_CHANGED_MEMBER(frame_changed);
	DECLARE_INPUT_CHANGED_MEMBER(refresh_changed);

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
	static constexpr int COLUMNS = 80;
	static constexpr int ROWS    = 24;

	// configuration switches, all read live from the ioports
	bool half_duplex() const;
	bool auto_new_line() const;
	bool lower_case_enabled() const;
	bool lower_case_displayed() const;
	bool clear_screen_enabled() const;
	bool keyboard_lock_enabled() const;
	bool destructive_space() const;
	bool cursor_control() const;
	bool twelve_line() const;
	bool fifty_hertz() const;
	bool column_beep() const;
	bool gated_extension() const;
	bool lower_case_fitted() const;
	u32  baud_rate() const;

	// RAM, the row counter, the offset counter and the column counter
	u8   entry_row() const { return (m_row + m_offset) % ROWS; }
	u8  &ram(u8 row, u8 col) { return m_ram[(row * COLUMNS) + col]; }

	// what the command decoder does with each character
	void handle_char(u8 data);
	void write_char(u8 data);
	void forespace();
	void backspace();
	void down_line();
	void up_line();
	void home_cursor();
	void carriage_return();
	void clear_screen();
	void erase_row(u8 row);
	void load_cursor_row(u8 data);
	void load_cursor_column(u8 data);
	void ring_bell();

	void send_char(u8 data);
	void key_char(u8 data);
	u8   shifted(u8 code, bool shift) const;

	void update_frame();
	void update_screen_params();

	TIMER_CALLBACK_MEMBER(bell_off);

	// video
	u32  screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	u8   glyph_row(u8 chr, u8 row) const;

	required_device<screen_device>   m_screen;
	required_device<palette_device>  m_palette;
	required_device<beep_device>     m_bell;
	required_memory_region           m_chargen;
	required_ioport                  m_modifiers;
	required_ioport                  m_baud;
	required_ioport                  m_switches;
	required_ioport                  m_options;

	emu_timer *m_bell_timer = nullptr;

	u8  m_ram[ROWS * COLUMNS];

	// the cursor's apparent row on the screen, its column, and how far the
	// display has scrolled away from the start of memory
	u8  m_row = 0;
	u8  m_col = 0;
	u8  m_offset = 0;

	// set by CR and cleared by LF, this is the window in which a space code
	// moves the cursor without erasing what it passes over
	bool m_no_write = false;

	// 0 idle, 1 seen ESC, 2 seen ESC =, 3 seen the row
	u8  m_esc_state = 0;

	bool m_keyboard_locked = false;

	u32 m_preset_baud = 0;

	// the UART's holding register, which is all the buffering there is
	u8   m_tx_char = 0;
	bool m_tx_full = false;
	bool m_tx_busy = false;
	bool m_break = false;

	devcb_write_line m_write_sd;
};

DECLARE_DEVICE_TYPE(ADM3A, adm3a_device)

#endif // MAME_MACHINE_ADM3A_H
