// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/*********************************************************************

    formats/mits_dsk.h

    MITS Altair 8-inch floppy disk image format

*********************************************************************/
#ifndef MAME_FORMATS_MITS_DSK_H
#define MAME_FORMATS_MITS_DSK_H

#pragma once

#include "flopimg.h"

class mits_dsk_format : public floppy_image_format_t
{
public:
	mits_dsk_format();

	int identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const override;
	bool load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const override;
	bool save(util::random_read_write &io, const std::vector<uint32_t> &variants, const floppy_image &image) const override;

	const char *name() const noexcept override { return "mits_dsk"; }
	const char *description() const noexcept override { return "MITS Altair 8-inch disk image"; }
	const char *extensions() const noexcept override { return "dsk"; }
	bool supports_save() const noexcept override { return true; }
};

extern const mits_dsk_format FLOPPY_MITS_DSK_FORMAT;

#endif // MAME_FORMATS_MITS_DSK_H
