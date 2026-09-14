// license:BSD-3-Clause
// copyright-holders:AJR, Paul Galbraith
/***************************************************************************

    DEC VT50 DECscope

***************************************************************************/

#ifndef MAME_MACHINE_DEC_VT50_H
#define MAME_MACHINE_DEC_VT50_H

#pragma once

#include "cpu/vt50/vt50.h"
#include "machine/ay31015.h"


class dec_vt50_device : public device_t
{
public:
	// positions of the rotary switches S1 and S2
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

	dec_vt50_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// serial data out of the EIA connector
	auto serial_data_callback() { return m_write_sd.bind(); }

	void serial_in_w(int state);

	void break_w(int state);
	DECLARE_INPUT_CHANGED_MEMBER(serial_sw_changed);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

private:
	void update_serial_settings();
	void update_uart_clocks();
	void gated_serial_output();
	void update_serial_in();

	u8 key_r(offs_t offset);
	void vert_count_w(u8 data);
	void uart_xd_w(u8 data);
	void serial_out_w(int state);
	int xrdy_eoc_r();
	u8 chargen_r(offs_t offset);

	void rom_1k(address_map &map) ATTR_COLD;
	void ram_1k(address_map &map) ATTR_COLD;

	required_device<vt50_cpu_device> m_maincpu;
	required_device<ay31015_device> m_uart;
	required_ioport_array<7> m_keys;
	required_ioport m_break_key;
	required_ioport m_mode_sw;
	required_ioport m_speed_sw;
	required_ioport m_data_sw;
	required_region_ptr<u8> m_chargen;

	devcb_write_line m_write_sd;

	bool m_serial_out;
	bool m_rec_data;
	bool m_9600_clock;
	u8 m_baud_divider;
	u8 m_110_baud_counter;
};

DECLARE_DEVICE_TYPE(DEC_VT50, dec_vt50_device)

#endif // MAME_MACHINE_DEC_VT50_H
