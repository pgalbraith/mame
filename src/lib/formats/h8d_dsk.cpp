// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/*********************************************************************

Heath H8D disk image format

Raw-sector disk images for Heath hard-sectored floppy controllers.

See https://sebhc.github.io/sebhc/h8d.html for the raw sector layout.

*********************************************************************/

#include "h8d_dsk.h"

#include "imageutl.h"

#include "ioprocs.h"

#include <array>
#include <cstring>


namespace {

// Cell timings.  fm_w() emits two half-cells of BITCELL_SIZE ns per data bit,
// so TRACK_SIZE half-cells make up the 200ms revolution of a 300 RPM drive.
//
// UNRESOLVED: this works out to a 125 kbps data rate and 3125 bytes/track,
// whereas the H17 controller's USRT clock (12.288MHz / 6 / 16 = 128 kHz)
// implies 128 kbps, a 3906.25ns half-cell and ~3200 bytes/track -- which is
// also the figure the drive's published capacity suggests.  Both give the
// correct 200ms revolution, and the controller's read PLL absorbs the 2.4%
// difference (sectors decode byte-exact), so nothing is visibly broken.  But
// one of the two figures is wrong, and h17disk.cpp carries the same constants.
// Settling it would remove the standing mismatch between the media written
// here and the rate heath_h17_fdc_device::fm_cell_time() reads at.
constexpr int TRACK_SIZE        = 50'000;
constexpr int BITCELL_SIZE      = 4000;

constexpr int SECTOR_DATA_SIZE  = 256;
constexpr int SECTORS_PER_TRACK = 10;
constexpr uint8_t H17_SYNC_BYTE = 0xfd;

struct format {
	int      head_count;
	int      track_count;
	uint32_t variant;
	uint32_t drive_variant;
};

const format formats[] = {
	{ 1, 40, floppy_image::SSSD10, floppy_image::SSSD }, // H-17-1
	// 200K raw images are ambiguous; H17 only officially supported SSSD so this is moot.
	{ 1, 80, floppy_image::SSQD10, floppy_image::SSQD },
	{ 2, 40, floppy_image::DSSD10, floppy_image::DSSD },
	{ 2, 80, floppy_image::DSQD10, floppy_image::DSQD }, // H-17-4
	{}
};

uint64_t format_size(format const &fmt)
{
	return uint64_t(fmt.head_count) * fmt.track_count * SECTORS_PER_TRACK * SECTOR_DATA_SIZE;
}

bool is_compatible(format const &fmt, std::vector<uint32_t> const &variants)
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

format find_format(uint64_t size, std::vector<uint32_t> const &variants)
{
	format first_size_match = {};

	for (int i = 0; formats[i].head_count; i++)
	{
		if (size == format_size(formats[i]))
		{
			if (!first_size_match.head_count)
			{
				first_size_match = formats[i];
			}

			if (is_compatible(formats[i], variants))
			{
				return formats[i];
			}
		}
	}

	return variants.empty() ? first_size_match : format{};
}

// H17 checksum: D = RLCA(byte XOR D) per byte, matching the ROM's RDB/WNB routines.
// The sync byte resets D to 0 before header or data bytes are accumulated.
uint8_t h17_checksum(uint8_t const *data, size_t length)
{
	uint8_t d = 0;

	for (size_t i = 0; i < length; i++)
	{
		uint8_t const x = d ^ data[i];
		d = uint8_t((x << 1) | (x >> 7));  // RLCA
	}

	return d;
}

bool is_hdos(std::vector<uint8_t> const &img, uint8_t &lab_ser)
{
	if (img.size() < 0x0a00)
	{
		return false;
	}

	uint8_t const *const label = &img[0x0900]; // track 0, sector 9
	uint8_t const lab_vlt = label[0x08];
	uint8_t const lab_ver = label[0x09];

	// LAB.VLT: 0=data, 1=system, 2=no directory.
	if (lab_vlt > 2)
	{
		return false;
	}

	if (!((lab_ver == 0x16) || (lab_ver == 0x17) || (lab_ver == 0x20) || ((lab_ver >= 0x30) && (lab_ver <= 0x39))))
	{
		return false;
	}

	int printable = 0;
	for (int i = 0; i < 60; i++)
	{
		uint8_t const c = label[0x11 + i];

		if (c == 0)
		{
			break;
		}

		if ((c == '\r') || (c == '\n') || (c == '\t') || ((c >= 0x20) && (c <= 0x7e)))
		{
			printable++;
		}
		else
		{
			return false;
		}
	}

	if (printable < 3)
	{
		return false;
	}

	lab_ser = label[0x00];
	return true;
}

uint8_t reverse_byte(uint8_t val)
{
	constexpr unsigned char lookup[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };

	return lookup[val & 0x0f] << 4 | lookup[val >> 4];
}

bool fm_byte_from_bitstream(std::vector<bool> const &bitstream, size_t pos, uint8_t &val)
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

bool decode_sector_data(std::vector<bool> const &bitstream, int sector, std::array<uint8_t, SECTOR_DATA_SIZE> &sector_data)
{
	size_t const sector_start = size_t(sector) * TRACK_SIZE / SECTORS_PER_TRACK;
	size_t const sector_end = size_t(sector + 1) * TRACK_SIZE / SECTORS_PER_TRACK;

	if (sector_end > bitstream.size())
	{
		return false;
	}

	for (size_t bit_offset = sector_start; bit_offset < (sector_start + 16); bit_offset++)
	{
		std::vector<uint8_t> bytes;

		for (size_t pos = bit_offset; (pos + 16) <= sector_end; pos += 16)
		{
			uint8_t val;
			if (!fm_byte_from_bitstream(bitstream, pos, val))
			{
				break;
			}

			bytes.push_back(val);
		}

		// find header sync
		size_t header_sync = 0;
		while ((header_sync < bytes.size()) && (bytes[header_sync] != H17_SYNC_BYTE))
		{
			header_sync++;
		}

		if ((header_sync + 5) > bytes.size())
		{
			continue;
		}

		// find data sync after the 4-byte header (sync + volume + track + sector + checksum)
		size_t data_sync = header_sync + 5;
		while ((data_sync < bytes.size()) && (bytes[data_sync] != H17_SYNC_BYTE))
		{
			data_sync++;
		}

		if ((data_sync + 1 + SECTOR_DATA_SIZE) > bytes.size())
		{
			continue;
		}

		std::copy_n(&bytes[data_sync + 1], SECTOR_DATA_SIZE, sector_data.begin());
		return true;
	}

	return false;
}

} // anonymous namespace


heath_h8d_format::heath_h8d_format() : floppy_image_format_t()
{
}

int heath_h8d_format::identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const
{
	uint64_t size;
	if (io.length(size))
	{
		return 0;
	}

	return find_format(size, variants).head_count ? FIFID_SIZE : 0;
}

bool heath_h8d_format::load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const
{
	uint64_t size;
	if (io.length(size))
	{
		return false;
	}

	const format fmt = find_format(size, variants);

	if (!fmt.head_count)
	{
		LOG_FORMATS("invalid H8D format\n");

		return false;
	}

	std::vector<uint8_t> img(size);

	auto const [err, actual] = read_at(io, 0, img.data(), img.size());
	if (err || (actual != img.size()))
	{
		LOG_FORMATS("unable to read H8D image\n");

		return false;
	}

	uint8_t lab_ser = 0;
	bool const hdos = is_hdos(img, lab_ser);

	image.set_variant(fmt.variant);

	std::vector<uint32_t> buf;
	uint8_t sector_data[SECTOR_DATA_SIZE];

	for (int head = 0; head < fmt.head_count; head++)
	{
		for (int track = 0; track < fmt.track_count; track++)
		{
			for (int sector = 0; sector < SECTORS_PER_TRACK; sector++)
			{
				int const data_offset = ((track * fmt.head_count + head) * SECTORS_PER_TRACK + sector) * SECTOR_DATA_SIZE;
				std::memcpy(sector_data, &img[data_offset], SECTOR_DATA_SIZE);

				// See https://sebhc.github.io/sebhc/project8080/design_h17.html
				// for the H-17 logical track and track-zero volume rules.
				int const logical_track = (track * fmt.head_count) + head;
				uint8_t const volume = (hdos && (logical_track > 0)) ? lab_ser : 0;
				uint8_t const hdr_bytes[] = { volume, uint8_t(logical_track), uint8_t(sector) };
				uint8_t const header_checksum = h17_checksum(hdr_bytes, 3);
				uint8_t const data_checksum = h17_checksum(sector_data, SECTOR_DATA_SIZE);

				// Initial 15 zero bytes
				for (int i = 0; i < 15; i++)
				{
					fm_reverse_byte_w(buf, 0);
				}

				// header (sync byte, volume, track, sector, checksum)
				fm_reverse_byte_w(buf, H17_SYNC_BYTE);
				fm_reverse_byte_w(buf, volume);
				fm_reverse_byte_w(buf, logical_track);
				fm_reverse_byte_w(buf, sector);
				fm_reverse_byte_w(buf, header_checksum);

				// 12 zero bytes
				for (int i = 0; i < 12; i++)
				{
					fm_reverse_byte_w(buf, 0);
				}

				// data sync byte
				fm_reverse_byte_w(buf, H17_SYNC_BYTE);

				// sector data
				for (int i = 0; i < SECTOR_DATA_SIZE; i++)
				{
					fm_reverse_byte_w(buf, sector_data[i]);
				}

				// sector data checksum
				fm_reverse_byte_w(buf, data_checksum);

				// trailing zero's until the next sector hole usually ~ 30 characters.
				while (buf.size() < TRACK_SIZE / SECTORS_PER_TRACK * (sector + 1))
				{
					fm_reverse_byte_w(buf, 0);
				}
			}

			generate_track_from_levels(track, head, buf, 0, image);
			buf.clear();
		}
	}

	return true;
}

bool heath_h8d_format::save(util::random_read_write &io, const std::vector<uint32_t> &variants, const floppy_image &image) const
{
	int track_count, head_count;
	image.get_actual_geometry(track_count, head_count);

	if (!head_count || !track_count)
	{
		LOG_FORMATS("H8D save: empty image\n");
		return false;
	}

	// find the matching format
	format fmt = {};
	for (int i = 0; formats[i].head_count; i++)
	{
		if ((formats[i].head_count == head_count) && (formats[i].track_count == track_count))
		{
			fmt = formats[i];
			break;
		}
	}

	if (!fmt.head_count)
	{
		LOG_FORMATS("H8D save: no matching format for %d heads %d tracks\n", head_count, track_count);
		return false;
	}

	std::vector<uint8_t> img(format_size(fmt), 0);

	std::array<uint8_t, SECTOR_DATA_SIZE> sector_data;

	for (int head = 0; head < fmt.head_count; head++)
	{
		for (int track = 0; track < fmt.track_count; track++)
		{
			std::vector<bool> bitstream = generate_bitstream_from_track(track, head, BITCELL_SIZE, image);

			for (int sector = 0; sector < SECTORS_PER_TRACK; sector++)
			{
				int const data_offset = ((track * fmt.head_count + head) * SECTORS_PER_TRACK + sector) * SECTOR_DATA_SIZE;

				if (decode_sector_data(bitstream, sector, sector_data))
				{
					std::copy(sector_data.begin(), sector_data.end(), &img[data_offset]);
				}
				else
				{
					LOG_FORMATS("H8D save: failed to decode track %d head %d sector %d\n", track, head, sector);
				}
			}
		}
	}

	auto const [err, actual] = write_at(io, 0, img.data(), img.size());
	return !err && (actual == img.size());
}

void heath_h8d_format::fm_reverse_byte_w(std::vector<uint32_t> &buffer, uint8_t val) const
{
	constexpr unsigned char lookup[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };

	fm_w(buffer, 8, lookup[val & 0x0f] << 4 | lookup[val >> 4], BITCELL_SIZE);
}

const heath_h8d_format FLOPPY_H8D_FORMAT;
