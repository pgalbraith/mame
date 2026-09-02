// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/*********************************************************************

Heath H8D disk image format

Raw-sector disk images for Heath hard-sectored floppy controllers.

See https://sebhc.github.io/sebhc/h8d.html for the raw sector layout.

*********************************************************************/

#include "h8d_dsk.h"

#include "h17_common.h"
#include "imageutl.h"

#include "ioprocs.h"

#include <array>
#include <cstring>


namespace {

using heath_h17::BITCELL_SIZE;
using heath_h17::H17_SYNC_BYTE;
using heath_h17::SECTORS_PER_TRACK;
using heath_h17::SECTOR_DATA_SIZE;
using heath_h17::TRACK_SIZE;
using heath_h17::fm_byte_from_bitstream;
using heath_h17::format;
using heath_h17::format_size;
using heath_h17::formats;
using heath_h17::h17_checksum;
using heath_h17::is_compatible;
using heath_h17::is_hdos;

// The 200K and 400K sizes are the double-sided images the page above describes,
// sides interleaved a track at a time; 400K only works out at 80 tracks a side.
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
	format const fmt = heath_h17::find_format(image);

	if (!fmt.head_count)
	{
		LOG_FORMATS("H8D save: no layout holds this image\n");
		return false;
	}

	std::vector<uint8_t> img(format_size(fmt), 0);

	std::array<heath_h17::sector_read, SECTORS_PER_TRACK> sectors;

	for (int head = 0; head < fmt.head_count; head++)
	{
		for (int track = 0; track < fmt.track_count; track++)
		{
			std::vector<bool> bitstream = generate_bitstream_from_track(track, head, BITCELL_SIZE, image);

			heath_h17::decode_track(bitstream, sectors);

			for (int sector = 0; sector < SECTORS_PER_TRACK; sector++)
			{
				int const data_offset = ((track * fmt.head_count + head) * SECTORS_PER_TRACK + sector) * SECTOR_DATA_SIZE;

				if (sectors[sector].found)
				{
					std::copy(sectors[sector].data.begin(), sectors[sector].data.end(), &img[data_offset]);

					if (!sectors[sector].data_valid)
					{
						LOG_FORMATS("H8D save: bad data checksum on track %d head %d sector %d\n", track, head, sector);
					}
				}
				else
				{
					// nothing to write but the zeroes already there
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
	fm_w(buffer, 8, heath_h17::reverse_byte(val), BITCELL_SIZE);
}

const heath_h8d_format FLOPPY_H8D_FORMAT;
