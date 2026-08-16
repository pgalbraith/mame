// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

  Heathkit H-17 Floppy Disk Controller


  Model number: H-88-1

  The controller itself is shared with the H8's H-8-17 card, and lives in
  bus/heathzenith/h17/h17_fdc_base.{h,cpp}.  Only the bus attachment is here.

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H89_H17_FDC_H
#define MAME_BUS_HEATHZENITH_H89_H17_FDC_H

#pragma once

#include "h89bus.h"

DECLARE_DEVICE_TYPE(H89BUS_H_17_FDC, device_h89bus_right_card_interface)

#endif // MAME_BUS_HEATHZENITH_H89_H17_FDC_H
