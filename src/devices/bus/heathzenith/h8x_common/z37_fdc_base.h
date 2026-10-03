// license:BSD-3-Clause
// copyright-holders:Mark Garlanger, Paul Galbraith
/***************************************************************************

  Heathkit Z-37 Soft-sectored Floppy Disk Controller

  Heath built the same controller for both of its busses:

    Z-89-37 - H89 right-hand slot
    WH-8-37 - H8 Benton Harbor bus, on one board with a Z-67 interface

  Both present the same four registers, and both take over the CPU board's
  interrupt encoding to run their data transfers.  Only the way the card
  hangs off the host differs, so everything except the bus attachment lives
  here.

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H8X_COMMON_Z37_FDC_BASE_H
#define MAME_BUS_HEATHZENITH_H8X_COMMON_Z37_FDC_BASE_H

#pragma once

#include "imagedev/floppy.h"
#include "machine/wd_fdc.h"


class heath_z37_fdc_base_device : public device_t
{
public:
	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:
	heath_z37_fdc_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// The controller's three lines into the interrupt logic, which on both
	// machines replaces the CPU board's priority encoder.  The bus attachment
	// decides how they get there.
	virtual void set_irq_out(int state) = 0;
	virtual void set_drq_out(int state) = 0;
	virtual void block_interrupts_out(int state) = 0;

	void ctrl_w(u8 val);

	void intf_w(u8 val);

	void cmd_w(u8 val);
	u8 stat_r();

	void data_w(u8 val);
	u8 data_r();

	void set_irq(int state);
	void set_drq(int state);

	required_device<fd1797_device> m_fdc;
	required_device_array<floppy_connector, 4> m_floppies;

	bool m_irq_allowed;
	bool m_drq_allowed;
	bool m_access_track_sector;

	/// Bits set in cmd_ControlPort_c - DK.CON
	static constexpr u8 ctrl_EnableIntReq_c = 0;
	static constexpr u8 ctrl_EnableDrqInt_c = 1;
	static constexpr u8 ctrl_SetMFMRecording_c = 2;
	static constexpr u8 ctrl_MotorsOn_c = 3;
	static constexpr u8 ctrl_Drive_0_c = 4;
	static constexpr u8 ctrl_Drive_1_c = 5;
	static constexpr u8 ctrl_Drive_2_c = 6;
	static constexpr u8 ctrl_Drive_3_c = 7;

	/// Bits to set alternate registers on InterfaceControl_c - DK.INT
	static constexpr u8 if_SelectSectorTrack_c = 0;
};

#endif // MAME_BUS_HEATHZENITH_H8X_COMMON_Z37_FDC_BASE_H
