// license:BSD-3-Clause
// copyright-holders:Mark Garlanger, Paul Galbraith
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

#include "flopimg.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>


namespace heath_h17 {

// Fixed parameters for all Heath hard-sectored disks: 10 sectors per track,
// 256 data bytes per sector, 300 RPM, FM encoding.
constexpr int SECTORS_PER_TRACK = 10;
constexpr int SECTOR_DATA_SIZE  = 256;

// Cell timings.  fm_w() emits two half-cells of BITCELL_SIZE ns per data bit,
// so TRACK_SIZE half-cells make up the 200ms revolution of a 300 RPM drive.
//
// These follow the controller rather than the nominal 250 kbps of single
// density.  The card's USRT is clocked at 2.048MHz / 16 = 128kHz - the divider
// is U810 on the H-88-1 schematic, a 4-bit counter with its control pins
// strapped to +5V so that it free-runs - and one bit leaves per clock, so a
// bit cell is 7812.5ns and a revolution holds 25600 of them: 3200 bytes a
// track, 320 a sector.  Everything about the format agrees with 320 and not
// with the 312.5 that 250 kbps would give: the layout below spends 290 bytes
// on a sector's preamble, header, data and checksums, and the ~30 bytes of
// trailing zeroes the generators pad with is exactly the remainder.
//
// Only TRACK_SIZE sets the rate the media is generated at: generate_track_from
// _levels() normalizes the buffer it is handed onto a 200ms revolution, so the
// half-cell comes out at 200ms / 51200 = 3906.25ns exactly however the buffer
// was measured.  BITCELL_SIZE is the unit that buffer is built in, and is used
// for real on the way back out, as the cell generate_bitstream_from_track()
// samples the flux at.  3906 is 0.006% under the 3906.25 it wants there, which
// is far inside what the drive itself varies by.
constexpr int TRACK_SIZE   = 51'200;
constexpr int BITCELL_SIZE = 3906;

// Sync byte preceding both the header and the data of every sector.
constexpr uint8_t H17_SYNC_BYTE = 0xfd;


// The physical layouts a disk can be realized as.  Neither container records
// which one it holds - both store logical sectors in order, and a given volume
// reads back the same whichever surface its tracks were spread over - so the
// layout is taken from the size, or from what the file states, and the drive
// settles any tie.  SSQD ties with DSSD at 200K and follows it, leaving the
// documented double-sided case to win a match made on size alone.
struct format {
	int      head_count;
	int      track_count;
	uint32_t variant;
	uint32_t drive_variant;
};

inline constexpr format formats[] = {
	{ 1, 40, floppy_image::SSSD10, floppy_image::SSSD }, // H-17-1
	{ 2, 40, floppy_image::DSSD10, floppy_image::DSSD },
	{ 1, 80, floppy_image::SSQD10, floppy_image::SSQD },
	{ 2, 80, floppy_image::DSQD10, floppy_image::DSQD }, // H-17-4
	{}
};

inline uint64_t format_size(format const &fmt)
{
	return uint64_t(fmt.head_count) * fmt.track_count * SECTORS_PER_TRACK * SECTOR_DATA_SIZE;
}

// A layout is only offered to a drive reporting one of its variants.  An empty
// list, which is what floptool passes, takes anything.
inline bool is_compatible(format const &fmt, std::vector<uint32_t> const &variants)
{
	if (variants.empty())
	{
		return true;
	}

	for (uint32_t variant : variants)
	{
		if ((variant == fmt.variant) || (variant == fmt.drive_variant))
		{
			return true;
		}
	}

	return false;
}


// The layout an image is written out as: the smallest one that holds every
// formatted track on it.
//
// A blank has no track to measure.  Its layout comes from the variant the
// unformatted filesystem stamped on it when it was created.  This controller
// writes single density whatever the media is rated for, so a double density
// variant means the same layout as the single density one, with or without
// its hard sectors.  A blank with no variant at all is written as the
// smallest layout.
//
// This has to answer for a blank rather than refuse it.  The drive truncates
// the file before it calls save() and does not look at the result, so a save
// that returns false leaves a 0 byte file, which no format will identify the
// next time it is mounted.  Only an image with more heads or tracks than any
// layout holds goes unanswered.
inline format find_format(floppy_image const &image)
{
	int tracks, heads;
	image.get_actual_geometry(tracks, heads);

	if (!tracks)
	{
		switch (image.get_variant())
		{
		case floppy_image::SSSD:
		case floppy_image::SSDD:
		case floppy_image::SSSD10:
		case floppy_image::SSDD10:
			heads = 1;
			tracks = 40;
			break;
		case floppy_image::DSSD:
		case floppy_image::DSDD:
		case floppy_image::DSSD10:
		case floppy_image::DSDD10:
			heads = 2;
			tracks = 40;
			break;
		case floppy_image::SSQD:
		case floppy_image::SSQD10:
			heads = 1;
			tracks = 80;
			break;
		case floppy_image::DSQD:
		case floppy_image::DSQD10:
			heads = 2;
			tracks = 80;
			break;
		default:
			heads = 0;
			tracks = 0;
			break;
		}
	}

	for (int i = 0; formats[i].head_count; i++)
	{
		if ((formats[i].head_count >= heads) && (formats[i].track_count >= tracks))
		{
			return formats[i];
		}
	}

	return {};
}


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

	// LAB.VLT: 0 = data, 1 = system, 2 = no directory.
	if (label[0x08] > 2)
	{
		return false;
	}

	// Two things identify an HDOS disk, and either will do.  Failing to
	// recognise one is worse than the alternative: the volume number would
	// default to zero, and HDOS rejects sectors whose volume does not match the
	// one in its label, so the disk simply will not read.
	//
	// The first is a copyright notice INIT writes past the end of the
	// user-supplied label text, at a fixed offset.  Every disk seen carries it,
	// including data volumes and third party software, though the wording
	// changed for 3.x - "SYSTEM COPYRIGHT HEATH CO., 10/1977" became "System
	// Copyright (c) Heath Co., 19xx" - so only the common part is compared, and
	// case is ignored.
	static char const marker[] = "SYSTEM COPYRIGHT";

	bool found = (label[0x54] == '\r') && (label[0x55] == '\n');

	for (int i = 0; found && (i < int(sizeof(marker) - 1)); i++)
	{
		uint8_t const c = label[0x56 + i];

		found = (((c >= 'a') && (c <= 'z')) ? uint8_t(c - 0x20) : c) == marker[i];
	}

	// The second is the version byte.  It cannot stand alone, since a 1.0 disk
	// records zero there and that cannot be told from a sector which was never
	// written, but it covers any disk whose copyright notice is missing.
	uint8_t const lab_ver = label[0x09];

	if (!found && (lab_ver != 0x15) && (lab_ver != 0x16) && (lab_ver != 0x20) && ((lab_ver < 0x30) || (lab_ver > 0x39)))
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


// How far after a header to look for its data field. The machine rewrites the
// data on its own, starting wherever the head is once it has read the header
// and set the write gate, so the gap it leaves is its own rather than the one
// the image was built with. A sector is 320 bytes of a 3200 byte track and the
// data field takes 258 of them, so a gap beyond this has nowhere to go.
constexpr int MAX_HEADER_TO_DATA_BYTES = 54;


// One sector as found on a track.
struct sector_read
{
	bool    found           = false;  // a header, and a data field to go with it
	bool    data_valid      = false;  // the data checksum matched
	uint8_t volume          = 0;
	uint8_t track           = 0;
	uint8_t sector          = 0;
	uint8_t header_checksum = 0;
	uint8_t data_checksum   = 0;

	std::array<uint8_t, SECTOR_DATA_SIZE> data{};
};


// Read every sector a track holds, locating each by its header rather than by
// where it sits. A sector the machine has rewritten lands wherever the head
// happened to be - measurably displacing the sectors after it - and it can
// straddle the index, so the search covers the whole track and wraps around it.
//
// The data field is searched for a bit at a time rather than a byte, because
// the machine lays it down as a record in its own right and it need not share
// the bit phase of the header the image was built with.
//
// Both checksums are verified, so a sync byte occurring within data cannot be
// taken for the start of a sector. A sector whose data checksum is bad is still
// returned, since a disk may genuinely hold one and dropping it would quietly
// replace the data with zeroes; a good copy found later wins.
inline void decode_track(std::vector<bool> const &bitstream, std::array<sector_read, SECTORS_PER_TRACK> &sectors)
{
	sectors.fill(sector_read());

	size_t const size = bitstream.size();

	if (size < 16)
	{
		return;
	}

	// a wrapped copy, so a sector crossing the index reads contiguously
	std::vector<bool> track(bitstream);
	track.insert(track.end(), bitstream.begin(), bitstream.end());

	for (size_t pos = 0; pos < size; pos++)
	{
		uint8_t val;

		if (!fm_byte_from_bitstream(track, pos, val) || (val != H17_SYNC_BYTE))
		{
			continue;
		}

		// header: volume, track, sector, checksum
		uint8_t header[4];
		size_t next = pos + 16;
		bool complete = true;

		for (uint8_t &b : header)
		{
			if (!fm_byte_from_bitstream(track, next, b))
			{
				complete = false;
				break;
			}

			next += 16;
		}

		int const sector = header[2];

		if (!complete || (sector >= SECTORS_PER_TRACK) || (h17_checksum(header, 3) != header[3]) || sectors[sector].data_valid)
		{
			continue;
		}

		size_t const search_end = next + (size_t(MAX_HEADER_TO_DATA_BYTES) * 16);

		for (size_t sync_pos = next; sync_pos < search_end; sync_pos++)
		{
			if (!fm_byte_from_bitstream(track, sync_pos, val))
			{
				break;
			}

			if (val != H17_SYNC_BYTE)
			{
				continue;
			}

			size_t data_pos = sync_pos + 16;

			std::array<uint8_t, SECTOR_DATA_SIZE> data;
			complete = true;

			for (uint8_t &b : data)
			{
				if (!fm_byte_from_bitstream(track, data_pos, b))
				{
					complete = false;
					break;
				}

				data_pos += 16;
			}

			uint8_t checksum;

			if (!complete || !fm_byte_from_bitstream(track, data_pos, checksum))
			{
				break;
			}

			bool const valid = h17_checksum(data.data(), data.size()) == checksum;

			if (!sectors[sector].found || valid)
			{
				sector_read &out = sectors[sector];

				out.found           = true;
				out.data_valid      = valid;
				out.volume          = header[0];
				out.track           = header[1];
				out.sector          = header[2];
				out.header_checksum = header[3];
				out.data_checksum   = checksum;
				out.data            = data;
			}

			break;
		}
	}
}

} // namespace heath_h17

#endif // MAME_FORMATS_H17_COMMON_H
