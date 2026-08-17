// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

  Heathkit H-17 Floppy controller

    The H89 version of the card, model number H-88-1. It plugs into the
    right-hand P506 slot, where the /FLPY select line decodes its four ports
    and the FMWE line write-enables the 1k of floppy RAM on the CPU board.

    The controller logic itself is shared with the H8's H-8-17 card, see
    bus/heathzenith/h17/h17_fdc_base.cpp.

****************************************************************************/

#include "emu.h"
#include "h17_fdc.h"

#include "bus/heathzenith/h17/h17_fdc_base.h"

#include "softlist_dev.h"

namespace {

class h_88_1_device : public heath_h17_fdc_base_device, public device_h89bus_right_card_interface
{
public:
	h_88_1_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual void set_ram_write_enable(int state) override;

	bool m_installed;
};


h_88_1_device::h_88_1_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_h17_fdc_base_device(mconfig, H89BUS_H_17_FDC, tag, owner, 0)
	, device_h89bus_right_card_interface(mconfig, *this)
{
}

void h_88_1_device::set_ram_write_enable(int state)
{
	// FMWE write-enables the floppy RAM in the ROM address space
	// ($1400-$17FF).  It is a P506 bus signal, so it has to be driven through
	// the slot interface - the h89bus routes it to the driver's memory view
	// selection.
	set_slot_fmwe(state);
}

void h_88_1_device::device_start()
{
	heath_h17_fdc_base_device::device_start();

	m_installed = false;

	save_item(NAME(m_installed));
}

void h_88_1_device::device_reset()
{
	if (!m_installed)
	{
		// /FLPY only reaches P506, so the decoder has to be asked with this
		// card's P506 signalling.  Left at the default of false it strips the
		// very bit being asked for, and a request for no select lines at all
		// matches every address in the PROM - the card then installs itself
		// over the whole I/O space and takes any other card's ports with it.
		h89bus::addr_ranges  addr_ranges = h89bus().get_address_ranges(h89bus::IO_FLPY, m_p506_signals);

		if (addr_ranges.size() == 1)
		{
			h89bus::addr_range range = addr_ranges.front();

			h89bus().install_io_device(range.first, range.second,
				read8sm_delegate(*this, FUNC(h_88_1_device::read)),
				write8sm_delegate(*this, FUNC(h_88_1_device::write)));
		}

		m_installed = true;
	}

	heath_h17_fdc_base_device::device_reset();
}

void h_88_1_device::device_add_mconfig(machine_config &config)
{
	heath_h17_fdc_base_device::device_add_mconfig(config);

	// the list belongs to the controller rather than to any one machine: the
	// H-88-1 came standard on the H-89 and was an option for the H-88
	SOFTWARE_LIST(config, "flop_list").set_original("h17_flop");
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(H89BUS_H_17_FDC, device_h89bus_right_card_interface, h_88_1_device, "h89_h17_fdc", "Heath H-17 Hard-sectored Controller (H-88-1)");
