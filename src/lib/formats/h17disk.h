// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/*********************************************************************

Heath h17disk  disk image format

The Heath hard-sectored disk format for the H8 and H89 systems with the
H17 controller on the H8 and the H-88-1 controller on the H89.

*********************************************************************/
#ifndef MAME_FORMATS_H17DISK_H
#define MAME_FORMATS_H17DISK_H

#pragma once

#include "flopimg.h"

class heath_h17d_format : public floppy_image_format_t
{
public:
	heath_h17d_format();

	int identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const override;
	bool load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const override;
	bool save(util::random_read_write &io, const std::vector<uint32_t> &variants, const floppy_image &image) const override;

	const char *name() const noexcept override { return "h17disk"; }
	const char *description() const noexcept override { return "Heath H17D disk image"; }
	const char *extensions() const noexcept override { return "h17,h17d,h17disk"; }
	bool supports_save() const noexcept override { return true; }

protected:

	void fm_reverse_byte_w(std::vector<uint32_t> &buffer, uint8_t val) const;
};


// Version 1 images load but are never written back. Only version 2 is written,
// so saving one would silently change the version of the file, and version 1
// keeps things that cannot be reconstructed from what is loaded: a dump of each
// physical sector including the slack the imager caught either side of it, the
// error status the imager recorded, and any blocks the loader passed over.
class heath_h17d_v1_format : public heath_h17d_format
{
public:
	int identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const override;

	const char *name() const noexcept override { return "h17disk1"; }
	const char *description() const noexcept override { return "Heath H17D disk image (version 1, read only)"; }
	bool supports_save() const noexcept override { return false; }
};

extern const heath_h17d_format FLOPPY_H17D_FORMAT;
extern const heath_h17d_v1_format FLOPPY_H17D_V1_FORMAT;

#endif // MAME_FORMATS_H17DISK_H
