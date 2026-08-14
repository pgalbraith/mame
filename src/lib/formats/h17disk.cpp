// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/*********************************************************************

Heath H17D disk image format (version 2.0.0)

   Format for Heath hard-sectored 5.25" disk images.

   See https://heathkit.garlanger.com/diskformats/ for more information

*********************************************************************/

#include "h17disk.h"

#include "imageutl.h"

#include "ioprocs.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>


namespace {

// See the note on these constants in h8d_dsk.cpp: the resulting 125 kbps data
// rate does not agree with the 128 kbps implied by the H17 controller's USRT
// clock.  Kept identical in both formats so they stay in step.
constexpr int TRACK_SIZE           = 50'000;
constexpr int BITCELL_SIZE         = 4000;

constexpr int SECTOR_METADATA_SIZE = 16;

constexpr int SECTOR_DATA_SIZE     = 256;
constexpr int SECTORS_PER_TRACK    = 10;
constexpr uint8_t H17_SYNC_BYTE    = 0xfd;

struct format {
	int      head_count;
	int      track_count;
	uint32_t variant;
};

const format formats[] = {
	{ 1, 40, floppy_image::SSSD10 }, // H-17-1
	{ 2, 40, floppy_image::DSSD10 },
	{ 1, 80, floppy_image::SSQD10 },
	{ 2, 80, floppy_image::DSQD10 }, // H-17-4
	{}
};

struct block_info {
	std::uint64_t pos = 0;
	std::uint32_t length = 0;
};

struct h17disk_info {
	block_info dskf;
	block_info h8db;
	block_info secm;
};

enum {
	DskF  = 0x466b7344, //!< "DskF", Disk Format
	Parm  = 0x6b726150, //!< "Parm", Parameters
	Date  = 0x65746144, //!< "Date", Date
	Imgr  = 0x72676d49, //!< "Imgr", Imager

	Prog  = 0x676f7250, //!< "Prog", Program (creation)
	Padd  = 0x64646150, //!< "Padd", Padding
	H8DB  = 0x42443848, //!< "H8DB", H8D data block
	SecM  = 0x4d636553, //!< "SecM", Sector Metadata
	Labl  = 0x6c62614c, //!< "Labl", Label
	Comm  = 0x6d6d6f43, //!< "Comm", Comment
};

uint32_t get_be32(uint8_t const *data)
{
	return (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | data[3];
}

uint32_t get_block_name(uint8_t const *data)
{
	return (uint32_t(data[3]) << 24) | (uint32_t(data[2]) << 16) | (uint32_t(data[1]) << 8) | data[0];
}

void put_be32(uint8_t *data, uint32_t val)
{
	data[0] = val >> 24;
	data[1] = val >> 16;
	data[2] = val >> 8;
	data[3] = val;
}

void put_block_name(uint8_t *data, uint32_t val)
{
	data[0] = val;
	data[1] = val >> 8;
	data[2] = val >> 16;
	data[3] = val >> 24;
}

uint8_t reverse_byte(uint8_t val)
{
	constexpr unsigned char lookup[16] = { 0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };

	return lookup[val & 0x0f] << 4 | lookup[val >> 4];
}

uint8_t checksum(uint8_t const *data, size_t length)
{
	uint8_t sum = 0;

	for (size_t i = 0; i < length; i++)
	{
		sum += data[i];
	}

	return -sum;
}

uint8_t checksum(uint8_t val0, uint8_t val1, uint8_t val2, uint8_t val3)
{
	return -(val0 + val1 + val2 + val3);
}

bool validate_header(util::random_read &io)
{
	uint8_t h[8];
	auto const [err, actual] = read_at(io, 0, h, sizeof(h));

	if (err || (actual != sizeof(h)))
	{
		return false;
	}

	return (h[0] == 'H') && (h[1] == '1') && (h[2] == '7') && (h[3] == 'D') &&
		(h[4] == '2') && (h[5] >= '0') && (h[5] <= '9') && (h[6] >= '0') && (h[6] <= '9') &&
		(h[7] == 0xff);
}

bool parse_blocks(util::random_read &io, h17disk_info &info)
{
	uint64_t file_size;
	if (io.length(file_size) || (file_size < 8))
	{
		return false;
	}

	uint64_t pos = 8;
	bool first = true;

	while ((pos + 8) <= file_size)
	{
		uint8_t header[8];
		auto const [err, actual] = read_at(io, pos, header, sizeof(header));

		if (err || (actual != sizeof(header)))
		{
			return false;
		}

		uint32_t const block_name = get_block_name(header);
		uint32_t const length = get_be32(&header[4]);
		uint64_t const data_pos = pos + 8;
		uint64_t const next_pos = data_pos + length;

		if (next_pos > file_size)
		{
			LOG_FORMATS("H17D block 0x%08x overruns file\n", block_name);
			return false;
		}

		if (first && (block_name != DskF))
		{
			LOG_FORMATS("H17D DskF block is not first\n");
			return false;
		}

		first = false;

		switch (block_name)
		{
			case DskF:
				info.dskf = { data_pos, length };
				break;
			case H8DB:
				info.h8db = { data_pos, length };
				break;
			case SecM:
				info.secm = { data_pos, length };
				break;
		}

		pos = next_pos;
	}

	return (pos == file_size) && (info.dskf.pos != 0) && (info.h8db.pos != 0);
}

format find_format(util::random_read &io, h17disk_info const &info)
{
	if ((info.dskf.length < 2) || (info.dskf.length > 3))
	{
		LOG_FORMATS("Can't find valid DskF block %d/%d\n", info.dskf.pos, info.dskf.length);

		return {};
	}

	uint8_t buf[3] = {};

	auto const [err, actual] = read_at(io, info.dskf.pos, buf, info.dskf.length);
	if (err || (actual != info.dskf.length))
	{
		LOG_FORMATS("read error\n");

		return {};
	}

	int const head_count = buf[0];
	int const track_count = buf[1];

	if ((info.dskf.length == 3) && (buf[2] > 1))
	{
		LOG_FORMATS("invalid DskF read-only flag %d\n", buf[2]);

		return {};
	}

	for (int i = 0; formats[i].head_count; i++)
	{
		if ((formats[i].head_count == head_count) && (formats[i].track_count == track_count))
		{
			LOG_FORMATS("find_format format found: %d - variant: 0x%x\n", i, formats[i].variant);

			return formats[i];
		}
	}

	LOG_FORMATS("Invalid disk format - heads: %d, tracks: %d\n", head_count, track_count);
	return {};
}

uint64_t format_size(format const &fmt)
{
	return uint64_t(fmt.head_count) * fmt.track_count * SECTORS_PER_TRACK * SECTOR_DATA_SIZE;
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

void generate_sector_metadata(uint8_t *metadata, format const &fmt, std::vector<uint8_t> const &img, int head, int track, int sector)
{
	uint8_t lab_ser = 0;
	bool const hdos = is_hdos(img, lab_ser);
	int const logical_track = (track * fmt.head_count) + head;
	uint8_t const volume = (hdos && (logical_track > 0)) ? lab_ser : 0;

	put_be32(metadata, uint32_t(((track * fmt.head_count + head) * SECTORS_PER_TRACK + sector) * SECTOR_DATA_SIZE));
	metadata[4] = 0;
	metadata[5] = H17_SYNC_BYTE;
	metadata[6] = volume;
	metadata[7] = uint8_t(logical_track);
	metadata[8] = uint8_t(sector);
	metadata[9] = checksum(H17_SYNC_BYTE, volume, uint8_t(logical_track), uint8_t(sector));
	metadata[10] = H17_SYNC_BYTE;
	metadata[11] = checksum(&img[get_be32(metadata)], SECTOR_DATA_SIZE);
	metadata[12] = 1;
	metadata[13] = 0;
	metadata[14] = 0;
	metadata[15] = 0;
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

bool decode_sector(std::vector<bool> const &bitstream, int sector, std::array<uint8_t, SECTOR_METADATA_SIZE> &metadata, std::array<uint8_t, SECTOR_DATA_SIZE> &sector_data)
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

		size_t header_sync = 0;
		while ((header_sync < bytes.size()) && (bytes[header_sync] != H17_SYNC_BYTE))
		{
			header_sync++;
		}

		if ((header_sync + 5) > bytes.size())
		{
			continue;
		}

		size_t data_sync = header_sync + 5;
		while ((data_sync < bytes.size()) && (bytes[data_sync] != H17_SYNC_BYTE))
		{
			data_sync++;
		}

		if ((data_sync + 1 + SECTOR_DATA_SIZE + 1) > bytes.size())
		{
			continue;
		}

		std::fill(metadata.begin(), metadata.end(), 0);
		metadata[4] = 0;
		metadata[5] = bytes[header_sync + 0];
		metadata[6] = bytes[header_sync + 1];
		metadata[7] = bytes[header_sync + 2];
		metadata[8] = bytes[header_sync + 3];
		metadata[9] = bytes[header_sync + 4];
		metadata[10] = bytes[data_sync];
		metadata[11] = bytes[data_sync + 1 + SECTOR_DATA_SIZE];
		metadata[12] = 1;
		metadata[13] = 0;
		std::copy_n(&bytes[data_sync + 1], SECTOR_DATA_SIZE, sector_data.begin());
		return true;
	}

	return false;
}

bool write_exact(util::random_read_write &io, uint64_t offset, void const *data, size_t length)
{
	auto const [err, actual] = write_at(io, offset, data, length);

	return !err && (actual == length);
}

bool write_block(util::random_read_write &io, uint64_t &offset, uint32_t block_name, void const *data, uint32_t length)
{
	uint8_t header[8];
	put_block_name(header, block_name);
	put_be32(&header[4], length);

	if (!write_exact(io, offset, header, sizeof(header)))
	{
		return false;
	}

	offset += sizeof(header);

	if (length && !write_exact(io, offset, data, length))
	{
		return false;
	}

	offset += length;
	return true;
}

} // anonymous namespace


heath_h17d_format::heath_h17d_format() : floppy_image_format_t()
{
}

int heath_h17d_format::identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const
{
	return validate_header(io) ? FIFID_SIGN : 0;
}

bool heath_h17d_format::load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const
{
	h17disk_info info;

	if (!validate_header(io) || !parse_blocks(io, info))
	{
		LOG_FORMATS("invalid H17D header or block structure\n");

		return false;
	}

	const format fmt = find_format(io, info);

	if (!fmt.head_count)
	{
		LOG_FORMATS("invalid format\n");

		return false;
	}

	if (info.h8db.length != format_size(fmt))
	{
		LOG_FORMATS("invalid H8DB length %d\n", info.h8db.length);

		return false;
	}

	if (info.h8db.pos != 256)
	{
		LOG_FORMATS("H8DB data does not start at offset 256\n");

		return false;
	}

	if (info.secm.pos && (info.secm.length < (fmt.head_count * fmt.track_count * SECTORS_PER_TRACK * SECTOR_METADATA_SIZE)))
	{
		LOG_FORMATS("SecM block too small %d\n", info.secm.length);

		return false;
	}

	image.set_variant(fmt.variant);

	std::vector<uint32_t> buf;

	std::vector<uint8_t> img(info.h8db.length);
	auto const [img_err, img_actual] = read_at(io, info.h8db.pos, img.data(), img.size());

	if (img_err || (img_actual != img.size()))
	{
		LOG_FORMATS("unable to read H8DB data\n");

		return false;
	}

	uint8_t sector_meta_data[SECTOR_METADATA_SIZE];
	uint8_t sector_data[SECTOR_DATA_SIZE];

	for (int head = 0; head < fmt.head_count; head++)
	{
		for (int track = 0; track < fmt.track_count; track++)
		{
			for (int sector = 0; sector < SECTORS_PER_TRACK; sector++)
			{
				int const sector_index = sector + (track * fmt.head_count + head) * SECTORS_PER_TRACK;
				int data_offset;

				if (info.secm.pos)
				{
					uint64_t const sect_meta_pos = uint64_t(sector_index) * SECTOR_METADATA_SIZE + info.secm.pos;

					auto const [err, actual] = read_at(io, sect_meta_pos, sector_meta_data, SECTOR_METADATA_SIZE);

					if (err || (actual != SECTOR_METADATA_SIZE))
					{
						LOG_FORMATS("unable to read sect meta data %d/%d/%d\n", head, track, sector);

						return false;
					}

					uint32_t const sector_data_pos = get_be32(sector_meta_data);

					if (sector_data_pos < info.h8db.pos)
					{
						LOG_FORMATS("sect data offset points before H8DB %d/%d/%d: %d\n", head, track, sector, sector_data_pos);

						return false;
					}

					data_offset = sector_data_pos - info.h8db.pos;
				}
				else
				{
					generate_sector_metadata(sector_meta_data, fmt, img, head, track, sector);
					data_offset = get_be32(sector_meta_data);
				}

				if ((data_offset < 0) || ((data_offset + SECTOR_DATA_SIZE) > img.size()))
				{
					LOG_FORMATS("invalid sect data offset %d/%d/%d: %d\n", head, track, sector, data_offset);

					return false;
				}

				std::memcpy(sector_data, &img[data_offset], SECTOR_DATA_SIZE);

				// Initial 15 zero bytes
				for (int i = 0; i < 15; i++)
				{
					fm_reverse_byte_w(buf, 0);
				}

				// header (sync byte, volume, track, sector, checksum)
				for (int i = 0; i < 5; i++)
				{
					fm_reverse_byte_w(buf, sector_meta_data[5 + i]);
				}

				// 12 zero bytes
				for (int i = 0; i < 12; i++)
				{
					fm_reverse_byte_w(buf, 0);
				}

				// data sync byte
				fm_reverse_byte_w(buf, sector_meta_data[10]);

				// sector data
				for (int i = 0; i < 256; i++)
				{
					fm_reverse_byte_w(buf, sector_data[i]);
				}

				// sector data checksum
				fm_reverse_byte_w(buf, sector_meta_data[11]);

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

bool heath_h17d_format::save(util::random_read_write &io, const std::vector<uint32_t> &variants, const floppy_image &image) const
{
	int tracks, heads;
	image.get_actual_geometry(tracks, heads);

	format fmt = {};
	for (int i = 0; formats[i].head_count; i++)
	{
		if ((formats[i].head_count == heads) && (formats[i].track_count == tracks))
		{
			fmt = formats[i];
			break;
		}
	}

	if (!fmt.head_count)
	{
		LOG_FORMATS("unsupported H17D geometry %d/%d\n", heads, tracks);

		return false;
	}

	size_t const sector_count = size_t(fmt.head_count) * fmt.track_count * SECTORS_PER_TRACK;
	std::vector<uint8_t> h8db(sector_count * SECTOR_DATA_SIZE);
	std::vector<uint8_t> secm(sector_count * SECTOR_METADATA_SIZE);

	for (int head = 0; head < fmt.head_count; head++)
	{
		for (int track = 0; track < fmt.track_count; track++)
		{
			std::vector<bool> const bitstream = generate_bitstream_from_track(track, head, BITCELL_SIZE, image);

			for (int sector = 0; sector < SECTORS_PER_TRACK; sector++)
			{
				size_t const sector_index = sector + (track * fmt.head_count + head) * SECTORS_PER_TRACK;
				std::array<uint8_t, SECTOR_METADATA_SIZE> metadata;
				std::array<uint8_t, SECTOR_DATA_SIZE> sector_data;

				if (!decode_sector(bitstream, sector, metadata, sector_data))
				{
					LOG_FORMATS("unable to decode H17D sector %d/%d/%d\n", head, track, sector);

					return false;
				}

				uint32_t const data_offset = 256 + uint32_t(sector_index * SECTOR_DATA_SIZE);
				put_be32(metadata.data(), data_offset);

				std::copy(sector_data.begin(), sector_data.end(), h8db.begin() + (sector_index * SECTOR_DATA_SIZE));
				std::copy(metadata.begin(), metadata.end(), secm.begin() + (sector_index * SECTOR_METADATA_SIZE));
			}
		}
	}

	uint8_t const header[8] = { 'H', '1', '7', 'D', '2', '0', '0', 0xff };

	if (!write_exact(io, 0, header, sizeof(header)))
	{
		return false;
	}

	uint64_t offset = sizeof(header);
	uint8_t dskf[3] = { uint8_t(fmt.head_count), uint8_t(fmt.track_count), 0 };

	if (!write_block(io, offset, DskF, dskf, sizeof(dskf)))
	{
		return false;
	}

	std::vector<uint8_t> padding(248 - offset - 8, 0);

	if (!write_block(io, offset, Padd, padding.data(), uint32_t(padding.size())))
	{
		return false;
	}

	if (!write_block(io, offset, H8DB, h8db.data(), uint32_t(h8db.size())))
	{
		return false;
	}

	return write_block(io, offset, SecM, secm.data(), uint32_t(secm.size()));
}

void heath_h17d_format::fm_reverse_byte_w(std::vector<uint32_t> &buffer, uint8_t val) const
{
	fm_w(buffer, 8, reverse_byte(val), BITCELL_SIZE);
}

const heath_h17d_format FLOPPY_H17D_FORMAT;
