// license:BSD-3-Clause
// copyright-holders:Mark Garlanger, Paul Galbraith
/*********************************************************************

Heath H17D disk image format (version 2.0.0)

   Format for Heath hard-sectored 5.25" disk images.

   See https://heathkit.garlanger.com/diskformats/ for more information

*********************************************************************/

#include "h17disk.h"

#include "h17_common.h"
#include "imageutl.h"

#include "ioprocs.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>


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
using heath_h17::reverse_byte;

constexpr int SECTOR_METADATA_SIZE = 16;

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

// Version 1 uses a single byte block identifier rather than a four character
// name.  These follow the identifiers HeathImager writes.
enum {
	V1_DISK_FORMAT = 0x00,
	V1_PARAMETERS  = 0x01,
	V1_LABEL       = 0x02,
	V1_COMMENT     = 0x03,
	V1_DATE        = 0x04,
	V1_IMAGER      = 0x05,
	V1_PROGRAM     = 0x06,
	V1_DATA        = 0x10,
	V1_RAW_DATA    = 0x30,
};

constexpr uint8_t V1_TRACK_SUB_BLOCK  = 0x11;
constexpr uint8_t V1_SECTOR_SUB_BLOCK = 0x12;
constexpr uint8_t V1_MANDATORY        = 0x80;

// Header sizes: block identifier, flags and a four byte length for version 1
// blocks; identifier, head, track and a two byte length for its track
// sub-blocks; identifier, sector, error status and a two byte length for its
// sector sub-blocks.
constexpr int V1_BLOCK_HEADER_SIZE  = 6;
constexpr int V1_TRACK_HEADER_SIZE  = 5;
constexpr int V1_SECTOR_HEADER_SIZE = 5;

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


enum class h17_version { v1, v2 };

// Both versions start with the "H17D" tag followed by a three byte version
// number, but they differ from there.  Version 1 records the version as binary
// values and begins its blocks at offset 7, while version 2 records it as
// ASCII digits, adds a 0xff byte to check the file has survived an 8-bit clean
// path, and begins its blocks at offset 8.
bool validate_header(util::random_read &io, h17_version &version)
{
	uint8_t h[8];
	auto const [err, actual] = read_at(io, 0, h, sizeof(h));

	if (err || (actual != sizeof(h)))
	{
		return false;
	}

	if ((h[0] != 'H') || (h[1] != '1') || (h[2] != '7') || (h[3] != 'D'))
	{
		return false;
	}

	if ((h[4] == '2') && (h[5] >= '0') && (h[5] <= '9') && (h[6] >= '0') && (h[6] <= '9') && (h[7] == 0xff))
	{
		version = h17_version::v2;

		return true;
	}

	if (h[4] == 1)
	{
		version = h17_version::v1;

		return true;
	}

	return false;
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

format find_format(util::random_read &io, h17disk_info const &info, std::vector<uint32_t> const &variants)
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
			// unlike an h8d the geometry is stated rather than guessed, so
			// there is nothing to disambiguate here - the variants only say
			// whether the drive can take the disk at all
			if (!is_compatible(formats[i], variants))
			{
				LOG_FORMATS("drive does not accept a %d head %d track disk\n", head_count, track_count);

				return {};
			}

			LOG_FORMATS("find_format format found: %d - variant: 0x%x\n", i, formats[i].variant);

			return formats[i];
		}
	}

	LOG_FORMATS("Invalid disk format - heads: %d, tracks: %d\n", head_count, track_count);
	return {};
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
	metadata[9] = h17_checksum(volume, uint8_t(logical_track), uint8_t(sector));
	metadata[10] = H17_SYNC_BYTE;
	metadata[11] = h17_checksum(&img[get_be32(metadata)], SECTOR_DATA_SIZE);
	metadata[12] = 1;
	metadata[13] = 0;
	metadata[14] = 0;
	metadata[15] = 0;
}

// Walk a version 1 data block.  Unlike version 2, which stores a bare image of
// the user data, version 1 stores a dump of each physical sector, so the sector
// headers can be taken from the disk rather than being manufactured.  Unpack
// those dumps into the raw sector image and a metadata array laid out like a
// version 2 SecM block, which is what the loader works from.  Offsets recorded
// in the metadata are relative to img.
//
// The dumps are longer than a sector's share of a track - the imager captures
// some slack either side - so they cannot be written to the track verbatim.
bool unpack_v1_sectors(std::vector<uint8_t> const &block, format const &fmt,
		std::vector<uint8_t> &img, std::vector<uint8_t> &secm)
{
	int const sector_count = fmt.head_count * fmt.track_count * SECTORS_PER_TRACK;

	img.assign(size_t(sector_count) * SECTOR_DATA_SIZE, 0);
	secm.assign(size_t(sector_count) * SECTOR_METADATA_SIZE, 0);

	size_t pos = 0;

	while ((pos + V1_TRACK_HEADER_SIZE) <= block.size())
	{
		if (block[pos] != V1_TRACK_SUB_BLOCK)
		{
			LOG_FORMATS("H17D v1: expected track sub-block at %d\n", int(pos));

			return false;
		}

		int const head = block[pos + 1];
		int const track = block[pos + 2];
		size_t const track_length = (size_t(block[pos + 3]) << 8) | block[pos + 4];
		size_t sect_pos = pos + V1_TRACK_HEADER_SIZE;
		size_t const track_end = sect_pos + track_length;

		if ((head >= fmt.head_count) || (track >= fmt.track_count) || (track_end > block.size()))
		{
			LOG_FORMATS("H17D v1: bad track sub-block %d/%d at %d\n", head, track, int(pos));

			return false;
		}

		while ((sect_pos + V1_SECTOR_HEADER_SIZE) <= track_end)
		{
			if (block[sect_pos] != V1_SECTOR_SUB_BLOCK)
			{
				LOG_FORMATS("H17D v1: expected sector sub-block at %d\n", int(sect_pos));

				return false;
			}

			uint8_t const status = block[sect_pos + 2];
			size_t const sect_length = (size_t(block[sect_pos + 3]) << 8) | block[sect_pos + 4];
			size_t const data_pos = sect_pos + V1_SECTOR_HEADER_SIZE;

			if ((data_pos + sect_length) > track_end)
			{
				LOG_FORMATS("H17D v1: sector sub-block overruns track at %d\n", int(sect_pos));

				return false;
			}

			// The dump holds the sector as it appears on the disk: a run of
			// zeros, the header sync byte, the volume, track and sector
			// numbers and their checksum, more zeros, the data sync byte, the
			// data and its checksum.
			size_t hdr = data_pos;
			while ((hdr < (data_pos + sect_length)) && (block[hdr] != H17_SYNC_BYTE))
			{
				hdr++;
			}

			if ((hdr + 5) > (data_pos + sect_length))
			{
				LOG_FORMATS("H17D v1: no header sync in sector at %d\n", int(sect_pos));

				return false;
			}

			int const sector = block[hdr + 3];

			if (sector >= SECTORS_PER_TRACK)
			{
				LOG_FORMATS("H17D v1: header sector %d out of range at %d\n", sector, int(sect_pos));

				return false;
			}

			size_t data = hdr + 5;
			while ((data < (data_pos + sect_length)) && (block[data] != H17_SYNC_BYTE))
			{
				data++;
			}

			if ((data + 1 + SECTOR_DATA_SIZE) > (data_pos + sect_length))
			{
				LOG_FORMATS("H17D v1: no data sync in sector at %d\n", int(sect_pos));

				return false;
			}

			int const sector_index = ((track * fmt.head_count) + head) * SECTORS_PER_TRACK + sector;
			uint32_t const img_offset = uint32_t(sector_index) * SECTOR_DATA_SIZE;
			uint8_t *const metadata = &secm[size_t(sector_index) * SECTOR_METADATA_SIZE];

			std::memcpy(&img[img_offset], &block[data + 1], SECTOR_DATA_SIZE);

			put_be32(metadata, img_offset);
			metadata[4]  = status;
			metadata[5]  = H17_SYNC_BYTE;
			metadata[6]  = block[hdr + 1];  // volume
			metadata[7]  = block[hdr + 2];  // track
			metadata[8]  = block[hdr + 3];  // sector
			metadata[9]  = block[hdr + 4];  // header checksum
			metadata[10] = H17_SYNC_BYTE;
			metadata[11] = block[data + 1 + SECTOR_DATA_SIZE];  // data checksum
			metadata[12] = 1;               // 0x0100 valid bytes
			metadata[13] = 0;
			metadata[14] = 0;
			metadata[15] = 0;

			sect_pos = data_pos + sect_length;
		}

		pos = track_end;
	}

	return pos == block.size();
}

// Version 1 blocks have a one byte identifier, a flags byte whose top bit marks
// the block as mandatory to process, and a four byte length.  Only the data
// block has to be present; the disk format block may be omitted, in which case
// its defaults apply.
bool load_v1(util::random_read &io, format &fmt, std::vector<uint8_t> &img, std::vector<uint8_t> &secm, std::vector<uint32_t> const &variants)
{
	uint64_t file_size;

	if (io.length(file_size) || (file_size < 7))
	{
		return false;
	}

	int head_count = 1;
	int track_count = 40;
	std::vector<uint8_t> data_block;
	bool have_data = false;
	uint64_t pos = 7;

	while ((pos + V1_BLOCK_HEADER_SIZE) <= file_size)
	{
		uint8_t header[V1_BLOCK_HEADER_SIZE];
		auto const [err, actual] = read_at(io, pos, header, sizeof(header));

		if (err || (actual != sizeof(header)))
		{
			return false;
		}

		uint8_t const id = header[0];
		uint8_t const flags = header[1];
		uint32_t const length = get_be32(&header[2]);
		uint64_t const data_pos = pos + V1_BLOCK_HEADER_SIZE;
		uint64_t const next_pos = data_pos + length;

		if (next_pos > file_size)
		{
			LOG_FORMATS("H17D v1 block 0x%02x overruns file\n", id);

			return false;
		}

		switch (id)
		{
		case V1_DISK_FORMAT:
			{
				if (length < 2)
				{
					LOG_FORMATS("H17D v1 disk format block too short %d\n", length);

					return false;
				}

				uint8_t buf[2];
				auto const [derr, dactual] = read_at(io, data_pos, buf, sizeof(buf));

				if (derr || (dactual != sizeof(buf)))
				{
					return false;
				}

				head_count = buf[0];
				track_count = buf[1];
			}
			break;

		case V1_DATA:
			{
				data_block.resize(length);
				auto const [derr, dactual] = read_at(io, data_pos, data_block.data(), data_block.size());

				if (derr || (dactual != data_block.size()))
				{
					LOG_FORMATS("unable to read H17D v1 data block\n");

					return false;
				}

				have_data = true;
			}
			break;

		case V1_PARAMETERS:
		case V1_LABEL:
		case V1_COMMENT:
		case V1_DATE:
		case V1_IMAGER:
		case V1_PROGRAM:
		case V1_RAW_DATA:
			// nothing here needs them to rebuild the disk
			break;

		default:
			// an unknown block only matters if the writer marked it as
			// something a reader has to understand
			if (flags & V1_MANDATORY)
			{
				LOG_FORMATS("unknown mandatory H17D v1 block 0x%02x\n", id);

				return false;
			}
			break;
		}

		pos = next_pos;
	}

	if ((pos != file_size) || !have_data)
	{
		LOG_FORMATS("invalid H17D v1 block structure\n");

		return false;
	}

	for (int i = 0; formats[i].head_count; i++)
	{
		if ((formats[i].head_count == head_count) && (formats[i].track_count == track_count))
		{
			if (!is_compatible(formats[i], variants))
			{
				LOG_FORMATS("drive does not accept a %d head %d track disk\n", head_count, track_count);

				return false;
			}

			fmt = formats[i];

			return unpack_v1_sectors(data_block, fmt, img, secm);
		}
	}

	LOG_FORMATS("invalid H17D v1 geometry - heads: %d, tracks: %d\n", head_count, track_count);

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
	h17_version version;

	// Version 1 is handled by heath_h17d_v1_format, which does not save. Only
	// version 2 is written, so claiming a version 1 image here would quietly
	// rewrite it in the other version the first time the machine touched it.
	return (validate_header(io, version) && (version == h17_version::v2)) ? FIFID_SIGN : 0;
}

int heath_h17d_v1_format::identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const
{
	h17_version version;

	return (validate_header(io, version) && (version == h17_version::v1)) ? FIFID_SIGN : 0;
}

bool heath_h17d_format::load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const
{
	h17_version version;

	if (!validate_header(io, version))
	{
		LOG_FORMATS("invalid H17D header\n");

		return false;
	}

	format fmt{};
	std::vector<uint8_t> img;

	// per-sector metadata laid out as a version 2 SecM block, left empty when
	// the file does not carry any and it has to be manufactured instead
	std::vector<uint8_t> secm;

	if (version == h17_version::v1)
	{
		if (!load_v1(io, fmt, img, secm, variants))
		{
			return false;
		}
	}
	else
	{
		h17disk_info info;

		if (!parse_blocks(io, info))
		{
			LOG_FORMATS("invalid H17D block structure\n");

			return false;
		}

		fmt = find_format(io, info, variants);

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

		int const sector_count = fmt.head_count * fmt.track_count * SECTORS_PER_TRACK;

		if (info.secm.pos && (info.secm.length < (sector_count * SECTOR_METADATA_SIZE)))
		{
			LOG_FORMATS("SecM block too small %d\n", info.secm.length);

			return false;
		}

		img.resize(info.h8db.length);
		auto const [img_err, img_actual] = read_at(io, info.h8db.pos, img.data(), img.size());

		if (img_err || (img_actual != img.size()))
		{
			LOG_FORMATS("unable to read H8DB data\n");

			return false;
		}

		if (info.secm.pos)
		{
			secm.resize(size_t(sector_count) * SECTOR_METADATA_SIZE);
			auto const [err, actual] = read_at(io, info.secm.pos, secm.data(), secm.size());

			if (err || (actual != secm.size()))
			{
				LOG_FORMATS("unable to read sector metadata\n");

				return false;
			}

			// rebase the recorded file offsets onto img so both versions look
			// the same to the loop below
			for (int i = 0; i < sector_count; i++)
			{
				uint8_t *const metadata = &secm[size_t(i) * SECTOR_METADATA_SIZE];
				uint32_t const sector_data_pos = get_be32(metadata);

				if (sector_data_pos < info.h8db.pos)
				{
					LOG_FORMATS("sect data offset points before H8DB %d: %d\n", i, sector_data_pos);

					return false;
				}

				put_be32(metadata, uint32_t(sector_data_pos - info.h8db.pos));
			}
		}
	}

	// The drive fixes how big the image is, and get_buffer() only asserts its
	// bounds in a debug build, so a disk carrying more heads or tracks than the
	// drive holds would otherwise be generated straight past the end of the
	// track array.
	int max_tracks, max_heads;
	image.get_maximal_geometry(max_tracks, max_heads);

	if ((fmt.head_count > max_heads) || (fmt.track_count > max_tracks))
	{
		LOG_FORMATS("%d head %d track image does not fit a %d head %d track drive\n", fmt.head_count, fmt.track_count, max_heads, max_tracks);

		return false;
	}

	image.set_variant(fmt.variant);

	std::vector<uint32_t> buf;

	uint8_t sector_meta_data[SECTOR_METADATA_SIZE];
	uint8_t sector_data[SECTOR_DATA_SIZE];

	for (int head = 0; head < fmt.head_count; head++)
	{
		for (int track = 0; track < fmt.track_count; track++)
		{
			for (int sector = 0; sector < SECTORS_PER_TRACK; sector++)
			{
				int const sector_index = sector + (track * fmt.head_count + head) * SECTORS_PER_TRACK;

				if (!secm.empty())
				{
					std::memcpy(sector_meta_data, &secm[size_t(sector_index) * SECTOR_METADATA_SIZE], SECTOR_METADATA_SIZE);
				}
				else
				{
					generate_sector_metadata(sector_meta_data, fmt, img, head, track, sector);
				}

				uint64_t const data_offset = get_be32(sector_meta_data);

				if ((data_offset + SECTOR_DATA_SIZE) > img.size())
				{
					LOG_FORMATS("invalid sect data offset %d/%d/%d: %d\n", head, track, sector, int(data_offset));

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
	format const fmt = heath_h17::find_format(image);

	if (!fmt.head_count)
	{
		LOG_FORMATS("no H17D layout holds this image\n");

		return false;
	}

	size_t const sector_count = size_t(fmt.head_count) * fmt.track_count * SECTORS_PER_TRACK;
	std::vector<uint8_t> h8db(sector_count * SECTOR_DATA_SIZE);
	std::vector<uint8_t> secm(sector_count * SECTOR_METADATA_SIZE);

	std::array<heath_h17::sector_read, SECTORS_PER_TRACK> sectors;

	for (int head = 0; head < fmt.head_count; head++)
	{
		for (int track = 0; track < fmt.track_count; track++)
		{
			std::vector<bool> const bitstream = generate_bitstream_from_track(track, head, BITCELL_SIZE, image);

			heath_h17::decode_track(bitstream, sectors);

			for (int sector = 0; sector < SECTORS_PER_TRACK; sector++)
			{
				size_t const sector_index = sector + (track * fmt.head_count + head) * SECTORS_PER_TRACK;
				heath_h17::sector_read const &found = sectors[sector];
				std::array<uint8_t, SECTOR_METADATA_SIZE> metadata;

				metadata.fill(0);

				uint32_t const data_offset = 256 + uint32_t(sector_index * SECTOR_DATA_SIZE);
				put_be32(metadata.data(), data_offset);

				// A sector that will not decode must not cost the whole image:
				// the file this is replacing has already been truncated by the
				// time save() is called, so returning here would leave nothing
				// at all. Record it as empty and carry on.
				if (!found.found)
				{
					LOG_FORMATS("unable to decode H17D sector %d/%d/%d\n", head, track, sector);
				}
				else
				{
					if (!found.data_valid)
					{
						LOG_FORMATS("bad data checksum on H17D sector %d/%d/%d\n", head, track, sector);
					}

					metadata[5]  = H17_SYNC_BYTE;
					metadata[6]  = found.volume;
					metadata[7]  = found.track;
					metadata[8]  = found.sector;
					metadata[9]  = found.header_checksum;
					metadata[10] = H17_SYNC_BYTE;
					metadata[11] = found.data_checksum;
					metadata[12] = 1;

					std::copy(found.data.begin(), found.data.end(), h8db.begin() + (sector_index * SECTOR_DATA_SIZE));
				}

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
const heath_h17d_v1_format FLOPPY_H17D_V1_FORMAT;
