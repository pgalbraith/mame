// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/*********************************************************************

    formats/mits_dsk.cpp

    MITS Altair 8-inch floppy disk image format

    The raw sector images simh and the Altair clones use: 77 tracks of 32
    sectors of 137 bytes, in order, 337,568 bytes. The images on deramp.com
    are padded out to a 256-byte boundary, 337,664 bytes, and load the same.

    The disk is hard sectored, 32 sector holes and an index hole, turning
    at 360 RPM, and recorded in FM at 250,000 bits a second, most significant
    bit first. The 88-DCDD controller has no sector header to look for. It
    starts writing a fixed delay after the sector hole, and when reading it
    waits a shorter delay and then takes the first data 1 as the sync bit,
    which is also the top bit of the sector's first byte. So the 137 bytes
    are everything a sector holds, and MITS software always starts a sector
    with a byte that has that bit set - 0x80 plus the track number.

    Loading puts each sector's data 389us after its hole, where a controller
    with the 1977 "NWD" write timing writes it; data there reads with either
    of the controller's read timings. Saving finds each sector the way the
    controller does, by the first data 1 after the read delay.

    References
    - Altair 88-DCDD manual, controller I/O information and read/write
      timing [https://deramp.com/downloads/altair/hardware/8_inch_floppy/Altair%20Floppy%20(88-DCDD)%20Manual.pdf]
    - MITS memo, System Timing Modification for Altair 88-DCDD Floppy Disk,
      2 September 1977, in the same manual

*********************************************************************/

#include "mits_dsk.h"

#include "ioprocs.h"

#include <vector>


namespace {

constexpr int TRACKS      = 77;
constexpr int SECTORS     = 32;
constexpr int SECTOR_SIZE = 137;
constexpr int IMAGE_SIZE  = TRACKS * SECTORS * SECTOR_SIZE;    // 337,568
constexpr int PADDED_SIZE = (IMAGE_SIZE + 255) & ~255;         // 337,664

// Floppy image positions run 0 to 200,000,000 a revolution. At 360 RPM a
// revolution is 166.67ms, so the 2us half of a 4us FM cell is exactly 2400.
constexpr int HALF_CELL = 2400;

// 1302 cells make a 5.2ms sector. 32 of them come to 41,664 cells, 0.007%
// short of a revolution; generate_track_from_levels() stretches the track to
// fit, which is far inside what a real drive's speed varies by.
constexpr int SECTOR_CELLS = 1302;

// where the controller writes (389us, the NWD write delay) and where it
// starts looking for the sync bit (140us, the earlier and shorter read delay)
constexpr int WRITE_DELAY_CELLS = 97;
constexpr int READ_DELAY_CELLS  = 35;

} // anonymous namespace


mits_dsk_format::mits_dsk_format() : floppy_image_format_t()
{
}

int mits_dsk_format::identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const
{
	uint64_t size;
	if (io.length(size) || ((size != IMAGE_SIZE) && (size != PADDED_SIZE)))
		return 0;

	if ((form_factor != floppy_image::FF_UNKNOWN) && (form_factor != floppy_image::FF_8))
		return 0;

	if (!variants.empty() && !has_variant(variants, floppy_image::SSSD32))
		return 0;

	// a disk MITS software has formatted starts every sector with the sync bit
	for (int sector = 0; sector < TRACKS * SECTORS; sector++)
	{
		uint8_t first;
		auto const [err, actual] = read_at(io, sector * SECTOR_SIZE, &first, 1);
		if (err || (actual != 1))
			return 0;
		if (!(first & 0x80))
			return FIFID_SIZE;
	}

	return FIFID_SIZE | FIFID_STRUCT;
}

bool mits_dsk_format::load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const
{
	std::vector<uint8_t> img(IMAGE_SIZE);
	auto const [err, actual] = read_at(io, 0, img.data(), IMAGE_SIZE);
	if (err || (actual != IMAGE_SIZE))
		return false;

	image.set_form_variant(floppy_image::FF_8, floppy_image::SSSD32);

	std::vector<uint32_t> buf;
	for (int track = 0; track < TRACKS; track++)
	{
		buf.clear();
		for (int sector = 0; sector < SECTORS; sector++)
		{
			uint8_t const *const data = &img[(track * SECTORS + sector) * SECTOR_SIZE];

			// clock cells only, from the hole to where the controller writes
			for (int cell = 0; cell < WRITE_DELAY_CELLS; cell++)
				fm_w(buf, 1, 0, HALF_CELL);

			for (int i = 0; i < SECTOR_SIZE; i++)
				fm_w(buf, 8, data[i], HALF_CELL);

			// and the 000 the controller goes on writing to the next hole
			for (int cell = WRITE_DELAY_CELLS + (SECTOR_SIZE * 8); cell < SECTOR_CELLS; cell++)
				fm_w(buf, 1, 0, HALF_CELL);
		}

		generate_track_from_levels(track, 0, buf, 0, image);
	}

	return true;
}

bool mits_dsk_format::save(util::random_read_write &io, const std::vector<uint32_t> &variants, const floppy_image &image) const
{
	std::vector<uint8_t> img(IMAGE_SIZE, 0);

	for (int track = 0; track < TRACKS; track++)
	{
		std::vector<bool> const bits = generate_bitstream_from_track(track, 0, HALF_CELL, image);

		for (int sector = 0; sector < SECTORS; sector++)
		{
			size_t const start = size_t(uint64_t(bits.size()) * sector / SECTORS);
			size_t const end = size_t(uint64_t(bits.size()) * (sector + 1) / SECTORS);

			// The first data 1 after the read delay is the sync bit. In the run of
			// clock-only cells before it, transitions are a whole cell apart, so the
			// first two a half cell apart are a clock and that data 1.
			size_t sync = start + (READ_DELAY_CELLS * 2);
			while ((sync + 1 < end) && !(bits[sync] && bits[sync + 1]))
				sync++;
			if (sync + 1 >= end)
				continue;   // nothing written here; the image keeps zeroes

			uint8_t *const data = &img[(track * SECTORS + sector) * SECTOR_SIZE];
			for (int i = 0; i < SECTOR_SIZE * 8; i++)
			{
				// data half-cells follow the sync bit's, two at a time
				size_t const pos = sync + 1 + (i * 2);
				if (pos >= end)
					break;
				if (bits[pos])
					data[i / 8] |= 0x80 >> (i % 8);
			}
		}
	}

	auto const [err, actual] = write_at(io, 0, img.data(), IMAGE_SIZE);
	return !err && (actual == IMAGE_SIZE);
}

const mits_dsk_format FLOPPY_MITS_DSK_FORMAT;
