// license:BSD-3-Clause
// copyright-holders:AJR, Paul Galbraith
/***************************************************************************

    DEC VT52 Video Display Terminal

***************************************************************************/

#ifndef MAME_MACHINE_DEC_VT52_H
#define MAME_MACHINE_DEC_VT52_H

#pragma once

#include "cpu/vt50/vt50.h"
#include "machine/ay31015.h"


class dec_vt52_device : public device_t
{
public:
	dec_vt52_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// serial data out of the EIA connector
	auto serial_data_callback() { return m_write_sd.bind(); }

	void serial_in_w(int state);

	void break_w(int state);
	DECLARE_INPUT_CHANGED_MEMBER(data_sw_changed);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

private:
	void update_serial_settings();
	void gated_serial_output(bool state);

	u8 key_r(offs_t offset);
	void baud_9600_w(int state);
	void vert_count_w(u8 data);
	void uart_xd_w(u8 data);
	void serial_out_w(int state);
	int xrdy_eoc_r();
	u8 chargen_r(offs_t offset);

	void rom_1k(address_map &map) ATTR_COLD;
	void ram_2k(address_map &map) ATTR_COLD;

	required_device<vt52_cpu_device> m_maincpu;
	required_device<ay31015_device> m_uart;
	required_ioport_array<8> m_keys;
	required_ioport m_break_key;
	required_ioport m_baud_sw;
	required_ioport m_data_sw;
	required_region_ptr<u8> m_chargen;

	devcb_write_line m_write_sd;

	bool m_serial_out;
	bool m_rec_data;
	u8 m_110_baud_counter;
};

DECLARE_DEVICE_TYPE(DEC_VT52, dec_vt52_device)

#endif // MAME_MACHINE_DEC_VT52_H
