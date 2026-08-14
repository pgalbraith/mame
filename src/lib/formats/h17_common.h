// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/*********************************************************************

    formats/h17_common.h

    Definitions shared by the Heath hard-sectored disk image formats,
    h17disk and h8d_dsk.

    Both formats describe the same physical media, so the fixed disk
    parameters, the checksum, and the routines for reading the HDOS label
    and decoding FM bytes have to agree between them.  They previously had
    separate copies which had drifted apart.

    See https://heathkit.garlanger.com/diskformats/ for the H17Disk
    specification, which also documents the physical format.

*********************************************************************/
#ifndef MAME_FORMATS_H17_COMMON_H
#define MAME_FORMATS_H17_COMMON_H

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>


namespace heath_h17 {

// Fixed parameters for all Heath hard-sectored disks: 10 sectors per track,
// 256 data bytes per sector, 300 RPM, FM encoding, 250 kbps.
constexpr int SECTORS_PER_TRACK = 10;
constexpr int SECTOR_DATA_SIZE  = 256;

// Cell timings.  fm_w() emits two half-cells of BITCELL_SIZE ns per data bit,
// so TRACK_SIZE half-cells make up the 200ms revolution of a 300 RPM drive.
// 4000ns per half-cell is the 250 kbps the format specification gives.
//
// Note this does not match heath_h17_fdc_device::fm_cell_time(), which is
// derived from the controller's 128 kHz USRT clock.  That is expected: on real
// hardware the USRT receive clock is recovered from the data coming off the
// disk, not driven by the transmit clock.  See the TODO in h17_fdc.cpp.
constexpr int TRACK_SIZE   = 50'000;
constexpr int BITCELL_SIZE = 4000;

// Sync byte preceding both the header and the data of every sector.
constexpr uint8_t H17_SYNC_BYTE = 0xfd;


// H17 checksum: D = RLCA(byte XOR D) per byte, matching the ROM's RDB/WNB
// routines.  The sync byte resets D to 0 before the header or data bytes are
// accumulated, so it is not itself part of the sum.
inline uint8_t h17_checksum(uint8_t const *data, size_t length)
{
	uint8_t d = 0;

	for (size_t i = 0; i < length; i++)
	{
		uint8_t const x = d ^ data[i];
		d = uint8_t((x << 1) | (x >> 7));  // RLCA
	}

	return d;
}

// Header checksum, taken over the volume, track and sector bytes.
inline uint8_t h17_checksum(uint8_t volume, uint8_t track, uint8_t sector)
{
	uint8_t const data[3] = { volume, track, sector };

	return h17_checksum(data, sizeof(data));
}


// Identify an HDOS volume from its label sector, returning the volume number
// in lab_ser.  The volume number is written into the header of every sector on
// tracks above zero, and HDOS rejects sectors whose volume does not match, so
// a false negative here silently produces an unreadable disk.
inline bool is_hdos(std::vector<uint8_t> const &img, uint8_t &lab_ser)
{
	if (img.size() < 0x0a00)
	{
		return false;
	}

	uint8_t const *const label = &img[0x0900]; // track 0, sector 9
	uint8_t const lab_vlt = label[0x08];
	uint8_t const lab_ver = label[0x09];

	// LAB.VLT: 0 = data, 1 = system, 2 = no directory.
	if (lab_vlt > 2)
	{
		return false;
	}

	if (!((lab_ver == 0x16) || (lab_ver == 0x17) || (lab_ver == 0x20) || ((lab_ver >= 0x30) && (lab_ver <= 0x39))))
	{
		return false;
	}

	lab_ser = label[0x00];

	return true;
}


inline uint8_t reverse_byte(uint8_t val)
{
	constexpr unsigned char lookup[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };

	return lookup[val & 0x0f] << 4 | lookup[val >> 4];
}


// Decode one FM-encoded byte out of a bitstream, at a half-cell position.  The
// data half-cells are the odd entries; bytes are recorded least significant bit
// first, hence the reversal.
inline bool fm_byte_from_bitstream(std::vector<bool> const &bitstream, size_t pos, uint8_t &val)
{
	if ((pos + 16) > bitstream.size())
	{
		return false;
	}

	uint8_t result = 0;

	for (int i = 0; i < 8; i++)
	{
		result = (result << 1) | (bitstream[pos + (i * 2) + 1] ? 1 : 0);
	}

	val = reverse_byte(result);

	return true;
}

} // namespace heath_h17

#endif // MAME_FORMATS_H17_COMMON_H
