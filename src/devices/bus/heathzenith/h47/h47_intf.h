// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath H-47 interface, the part that is the same on both buses

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H47_H47_INTF_H
#define MAME_BUS_HEATHZENITH_H47_H47_INTF_H

#pragma once

#include "h47.h"


class heath_h47_intf_device : public device_t
{
public:

	// Two ports, status then data, both readable and writable.  Only A0 is
	// decoded, so a four port block answers as 170, 171, 170, 171.
	u8   read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:

	heath_h47_intf_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_reset_after_children() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// the bus attachment decides where the interrupt goes
	virtual void set_interrupt(int state) = 0;

	// status register bits, H47DEF's S.* flags
	static constexpr u8 S_ERR = 0x01;
	static constexpr u8 S_SW0 = 0x02;
	static constexpr u8 S_SW1 = 0x04;
	static constexpr u8 S_SW2 = 0x08;
	static constexpr u8 S_SW3 = 0x10;
	static constexpr u8 S_DON = 0x20;
	static constexpr u8 S_IEN = 0x40;
	static constexpr u8 S_DTR = 0x80;

	// what a write to the status port does
	static constexpr u8 W_U137B = 0x01;
	static constexpr u8 W_RES   = 0x02;
	static constexpr u8 W_IEN   = 0x40;

	void done_w(int state);
	void dtr_w(int state);
	void error_w(int state);
	void update_interrupt();

	required_device<heath_h47_device> m_h47;
	required_ioport                   m_sw101;

	// the three cable lines, as the status buffer reports them
	bool m_done;
	bool m_dtr;
	bool m_error;

	bool m_int_enabled;   // status port D6
	bool m_int_latched;   // U141A
	bool m_u137b;         // status port D0
};

#endif // MAME_BUS_HEATHZENITH_H47_H47_INTF_H
