// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heath WH-8-37 Double-Density Disk Controller and Z-67 Interface

  The floppy controller is shared with the H89's Z-89-37, and lives in
  bus/heathzenith/h8x_common/z37_fdc_base.{h,cpp}.  The Z-67 interface is
  shared with the Z-89-67, in bus/heathzenith/h8x_common/h67.{h,cpp}.

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H8_WH_8_37_H
#define MAME_BUS_HEATHZENITH_H8_WH_8_37_H

#pragma once

#include "h8bus.h"

DECLARE_DEVICE_TYPE(H8BUS_WH_8_37, device_h8bus_card_interface)

#endif // MAME_BUS_HEATHZENITH_H8_WH_8_37_H
