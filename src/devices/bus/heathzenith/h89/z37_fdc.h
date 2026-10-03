// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

  Heathkit Z-37 Floppy Disk Controller

****************************************************************************/

#ifndef MAME_BUS_HEATHZENITH_H89_Z37_FDC_H
#define MAME_BUS_HEATHZENITH_H89_Z37_FDC_H

#pragma once

#include "h89bus.h"

#include "bus/heathzenith/h8x_common/z37_fdc_base.h"
#include "bus/heathzenith/intr_cntrl/intr_cntrl.h"

class h89bus_z37_device : public heath_z37_fdc_base_device, public device_h89bus_right_card_interface
{
public:
	h89bus_z37_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	template <typename T> void set_intr_cntrl(T &&tag) { m_intr_cntrl.set_tag(std::forward<T>(tag)); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual void set_irq_out(int state) override;
	virtual void set_drq_out(int state) override;
	virtual void block_interrupts_out(int state) override;

private:
	required_device<heath_intr_socket> m_intr_cntrl;

	bool m_installed;
};

DECLARE_DEVICE_TYPE(H89BUS_Z37, device_h89bus_right_card_interface)

#endif // MAME_BUS_HEATHZENITH_H89_Z37_FDC_H
