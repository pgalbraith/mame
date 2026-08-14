// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

  Heathkit H-17 Floppy Disk Controller


  Model number: H-88-1

  TODO
   - Use the floppy clock bits to clock the USRT receive clock.  The receive
     path currently synthesises that clock from a fixed-rate timer; see the
     TODO block in h17_fdc.cpp for what that costs.

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H89_H17_FDC_H
#define MAME_BUS_HEATHZENITH_H89_H17_FDC_H

#pragma once

#include "h89bus.h"

DECLARE_DEVICE_TYPE(H89BUS_H_17_FDC, device_h89bus_right_card_interface)

#endif // MAME_BUS_HEATHZENITH_H89_H17_FDC_H
