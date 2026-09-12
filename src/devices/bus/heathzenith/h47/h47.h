// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath H-47 Dual 8-inch Floppy Disk System

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H47_H47_H
#define MAME_BUS_HEATHZENITH_H47_H47_H

#pragma once

#include "imagedev/floppy.h"
#include "machine/wd_fdc.h"


class heath_h47_device : public device_t
{
public:

	heath_h47_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// The three cable lines the interface card puts in its status register.
	// All three are active low on the 40-wire cable; they are reported here
	// the way the card's own status bits read, so 1 means asserted.
	auto done_cb()  { return m_done_cb.bind(); }   // /BUSY,  idle, will accept a command
	auto dtr_cb()   { return m_dtr_cb.bind(); }    // /DTR,   a byte is wanted or waiting
	auto error_cb() { return m_error_cb.bind(); }  // /ERROR, the last command failed

	// /MRST - unconditionally abort whatever is in progress and return to the
	// system ready state
	void mrst_w(int state);

	// The data port.  Each access carries its own DTAK, so the card has no
	// separate line to drive for it.
	u8   data_r();
	void data_w(u8 data);

protected:

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:

	// What the controller is doing between the host's accesses.  Plain u8
	// rather than an enum class so that save_item takes it.
	static constexpr u8 PH_IDLE     = 0;  // waiting for a command byte
	static constexpr u8 PH_PARAMS   = 1;  // collecting the command's parameters
	static constexpr u8 PH_EXEC     = 2;  // the drive is working
	static constexpr u8 PH_DATA_OUT = 3;  // bytes are waiting for the host
	static constexpr u8 PH_DATA_IN  = 4;  // bytes are wanted from the host

	// Where the floppy sequencer is within one command.
	static constexpr u8 ST_NONE     = 0;
	static constexpr u8 ST_SEEK     = 1;
	static constexpr u8 ST_READ_ID  = 2;
	static constexpr u8 ST_READ     = 3;
	static constexpr u8 ST_WRITE    = 4;
	static constexpr u8 ST_FORMAT   = 5;
	// only ever an m_after_id: write back the sector already in the buffer
	// rather than asking the host for one
	static constexpr u8 ST_COPY     = 6;

	// Controller commands.  Names and numbers are Heath's own, from the
	// H47DEF deck of the HDOS 3.02 sources.
	static constexpr u8 CMD_BOOT    = 0x00;  // boot
	static constexpr u8 CMD_RST     = 0x01;  // read status
	static constexpr u8 CMD_RAS     = 0x02;  // read auxiliary status
	static constexpr u8 CMD_LSC     = 0x03;  // load sector count
	static constexpr u8 CMD_RAD     = 0x04;  // read address of last sector accessed
	static constexpr u8 CMD_REA     = 0x05;  // read sectors
	static constexpr u8 CMD_WRI     = 0x06;  // write sectors
	static constexpr u8 CMD_REAB    = 0x07;  // read sectors buffered
	static constexpr u8 CMD_WRIB    = 0x08;  // write sectors buffered
	static constexpr u8 CMD_WRD     = 0x09;  // write sectors, deleted data
	static constexpr u8 CMD_WRBD    = 0x0a;  // write sectors buffered, deleted data
	static constexpr u8 CMD_CPY     = 0x0b;  // copy
	static constexpr u8 CMD_FRM0    = 0x0c;  // format IBM single density
	static constexpr u8 CMD_FRM1    = 0x0d;  // format single density
	static constexpr u8 CMD_FRM2    = 0x0e;  // format IBM double density
	static constexpr u8 CMD_FRM3    = 0x0f;  // format double density
	static constexpr u8 CMD_RRDY    = 0x10;  // read ready status

	// Status byte, what CMD_RST returns - H47DEF's SB.* flags
	static constexpr u8 SB_UNR      = 0x80;  // unit not ready
	static constexpr u8 SB_WPD      = 0x40;  // write protected drive
	static constexpr u8 SB_DLD      = 0x20;  // deleted data
	static constexpr u8 SB_NRF      = 0x10;  // no record found
	static constexpr u8 SB_CRC      = 0x08;  // CRC error
	static constexpr u8 SB_LTD      = 0x04;  // late data
	static constexpr u8 SB_ILC      = 0x02;  // illegal command
	static constexpr u8 SB_BTO      = 0x01;  // bad track overflow

	// Auxiliary status byte, what CMD_RAS returns - H47DEF's AS.* flags
	static constexpr u8 AS_0DD      = 0x40;  // track 0 double density
	static constexpr u8 AS_1DD      = 0x20;  // tracks 1-76 double density
	static constexpr u8 AS_S1A      = 0x10;  // side 1 available
	static constexpr u8 AS_SLM      = 0x03;  // sector length mask

	// FD1793 commands as this firmware issues them.  There is no restore
	// among them: a 179x seeks to track 0 by itself when it comes out of
	// reset, and the drives start there too, so the head position is known
	// from the beginning and m_phys_track tracks it from there.
	static constexpr u8 FDC_SEEK    = 0x18;  // load head, 3 ms step, no verify
	static constexpr u8 FDC_READ    = 0x84;  // single record, 15 ms settle
	static constexpr u8 FDC_WRITE   = 0xa4;
	static constexpr u8 FDC_WRITE_D = 0xa5;  // ... laying a deleted data mark
	static constexpr u8 FDC_READ_ID = 0xc4;
	static constexpr u8 FDC_WR_TRK  = 0xf4;
	static constexpr u8 FDC_FORCE   = 0xd0;

	// FD1793 status bits for the type II and III commands
	static constexpr u8 FS_NOT_RDY  = 0x80;
	static constexpr u8 FS_PROTECT  = 0x40;
	static constexpr u8 FS_DELETED  = 0x20;
	static constexpr u8 FS_NOT_FND  = 0x10;
	static constexpr u8 FS_CRC_ERR  = 0x08;
	static constexpr u8 FS_LOST     = 0x04;

	static constexpr u8  TRACKS     = 77;    // 0 to 76
	// one 8" MFM track at 500 kbit/s and 360 rpm
	static constexpr u16 MAX_TRACK  = 10416;

	// 8" drives clock the 179x at 2 MHz
	static constexpr XTAL FDC_CLOCK = 8_MHz_XTAL / 4;

	void set_done(int state);
	void set_dtr(int state);
	void set_error(int state);

	void start_command();
	void want_params(u8 count);
	void send_bytes(u32 count);
	void take_bytes(u32 count);
	void finish(u8 status);
	void abort();

	floppy_image_device *unit();
	bool   select(u8 sus, bool for_write);
	u16    sector_size() const { return 128 << (m_size_code & 3); }
	u8     sectors_per_track() const;

	void   begin_transfer(bool write, bool deleted);
	void   begin_copy();
	void   copy_read();
	void   begin_format(bool double_density, u8 sectors);
	void   build_track(bool double_density, u8 sectors);
	static u8 next_sector(u8 sus, u8 per_track);

	void   seek_then(u8 next);
	void   start_read();
	void   start_write();
	void   fdc_intrq_w(int state);
	void   fdc_drq_w(int state);
	u8     translate_status(u8 fdc_status) const;

	required_device<fd1793_device>              m_fdc;
	required_device_array<floppy_connector, 4>  m_floppies;

	devcb_write_line  m_done_cb;
	devcb_write_line  m_dtr_cb;
	devcb_write_line  m_error_cb;

	// host side
	u8     m_phase;
	bool   m_done;
	bool   m_dtr;
	bool   m_error;

	u8     m_cmd;
	u8     m_params[4];
	u8     m_param_count;
	u8     m_param_wanted;

	u8     m_buffer[MAX_TRACK];
	u32    m_buf_pos;
	u32    m_buf_len;
	u8     m_gap_fill;          // what to keep feeding a write track that runs dry

	u8     m_status;            // what CMD_RST will report
	u16    m_sector_count;      // what CMD_LSC last loaded

	// drive and media
	u8     m_unit;
	u8     m_side;
	u8     m_track;
	u8     m_sector;
	u8     m_phys_track[4];
	u8     m_last_id[4];        // what CMD_RAD will report

	bool   m_double_density;
	u8     m_size_code;

	// floppy sequencer
	u8     m_step;
	u8     m_after_seek;
	u8     m_after_id;
	bool   m_id_retried;
	u16    m_sectors_left;
	bool   m_deleted;
	bool   m_deleted_seen;
	u8     m_format_sectors;

	// the copy command, which has a source and a destination rather than a
	// host on one end
	bool   m_copying;
	u8     m_src_track;
	u8     m_src_sus;
	u8     m_dst_track;
	u8     m_dst_sus;
};

DECLARE_DEVICE_TYPE(HEATH_H47, heath_h47_device)

#endif // MAME_BUS_HEATHZENITH_H47_H47_H
