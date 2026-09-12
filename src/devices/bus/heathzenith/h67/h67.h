// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath/Zenith Z-67 interface, the part that is the same on both buses

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H67_H67_H
#define MAME_BUS_HEATHZENITH_H67_H67_H

#pragma once

#include "bus/nscsi/sa1403d.h"
#include "machine/nscsi_bus.h"


// ======================> heath_z67_controller_device
//
//  The cabinet's controller, which is the stock SA1400-family board with the
//  drives a Z-67 carried rather than the ones its DIP switch defaults to.

class heath_z67_controller_device : public nscsi_sa1403d_device
{
public:

	heath_z67_controller_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);
};

DECLARE_DEVICE_TYPE(HEATH_Z67_CONTROLLER, heath_z67_controller_device)



// ======================> heath_h67_host_device
//
//  The line-level bridge between the card's two ports and the SASI bus.

class heath_h67_host_device : public device_t, public nscsi_device_interface
{
public:

	heath_h67_host_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// REQ, for whatever the card does with its interrupt enable bit
	auto req_cb() { return m_req_cb.bind(); }

	// Control register, written to the second port - MTR-90's BC_* names
	static constexpr u8 BC_SEL = 0x40;  // select, and data bit 0 with it
	static constexpr u8 BC_IE  = 0x20;  // interrupt enable
	static constexpr u8 BC_RST = 0x10;  // reset
	static constexpr u8 BC_EDT = 0x02;  // enable data

	// Bus status register, read from the second port - MTR-90's BS_* names
	static constexpr u8 BS_REQ = 0x80;  // data transfer request
	static constexpr u8 BS_DTD = 0x40;  // direction, set means to the controller
	static constexpr u8 BS_LMB = 0x20;  // last byte of a command or data string
	static constexpr u8 BS_MTY = 0x10;  // message type, set means command
	static constexpr u8 BS_BSY = 0x08;  // busy

	u8   data_r();
	void data_w(u8 data);
	u8   status_r();
	void control_w(u8 data);

protected:

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void scsi_ctrl_changed() override;

private:

	TIMER_CALLBACK_MEMBER(ack_off);

	// The ACK the card pulses for each byte the host reads or writes.  The
	// card has no documented timing, so this is the same half microsecond
	// every other host adapter of the period is modelled with.
	static constexpr attotime ACK_PULSE = attotime::from_nsec(500);

	devcb_write_line m_req_cb;
	emu_timer       *m_ack_timer;

	u8   m_control;
	bool m_req;
};

DECLARE_DEVICE_TYPE(HEATH_H67_HOST, heath_h67_host_device)


// ======================> heath_h67_intf_device

class heath_h67_intf_device : public device_t
{
public:

	// Two ports, data then control and bus status.  Only A0 is decoded, so a
	// four port block answers as 170, 171, 170, 171.
	u8   read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:

	heath_h67_intf_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// the bus attachment decides where the interrupt goes
	virtual void set_interrupt(int state) = 0;

	void req_w(int state);
	void update_interrupt();

	required_device<nscsi_bus_device>      m_sasi;
	required_device<heath_h67_host_device> m_host;

	bool m_int_enabled;
	bool m_req;
};

#endif // MAME_BUS_HEATHZENITH_H67_H67_H
