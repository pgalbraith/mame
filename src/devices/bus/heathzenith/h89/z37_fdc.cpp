// license:BSD-3-Clause
// copyright-holders:Mark Garlanger
/***************************************************************************

  Heathkit Z-37 Floppy controller

    The H89 version of the card, model number Z-89-37.  The controller logic
    itself is shared with the H8's WH-8-37, see
    bus/heathzenith/h8x_common/z37_fdc_base.cpp.

****************************************************************************/

#include "emu.h"

#include "z37_fdc.h"

#include "softlist_dev.h"

#define LOG_ERR (1U << 1)    // log errors
#define LOG_SETUP (1U << 2)  // setup

#define VERBOSE (0)

#include "logmacro.h"

#define LOGERR(...)        LOGMASKED(LOG_ERR, __VA_ARGS__)
#define LOGSETUP(...)      LOGMASKED(LOG_SETUP, __VA_ARGS__)

#ifdef _MSC_VER
#define FUNCNAME __func__
#else
#define FUNCNAME __PRETTY_FUNCTION__
#endif


h89bus_z37_device::h89bus_z37_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: heath_z37_fdc_base_device(mconfig, H89BUS_Z37, tag, owner, 0)
	, device_h89bus_right_card_interface(mconfig, *this)
	, m_intr_cntrl(*this, finder_base::DUMMY_TAG)
{
}

void h89bus_z37_device::device_start()
{
	heath_z37_fdc_base_device::device_start();

	m_installed = false;

	save_item(NAME(m_installed));
}

void h89bus_z37_device::device_reset()
{
	if (!m_installed)
	{
		h89bus::addr_ranges  addr_ranges = h89bus().get_address_ranges(h89bus::IO_CASS);

		if (addr_ranges.size() == 1)
		{
			h89bus::addr_range range = addr_ranges.front();

			LOGSETUP("%s: addr: 0x%04x-0x%04x\n", FUNCNAME, range.first, range.second);

			h89bus().install_io_device(range.first, range.second,
				read8sm_delegate(*this, FUNC(h89bus_z37_device::read)),
				write8sm_delegate(*this, FUNC(h89bus_z37_device::write)));
		}
		else
		{
			LOGERR("%s: no address provided for device\n", FUNCNAME);
		}

		m_installed = true;
	}

	heath_z37_fdc_base_device::device_reset();
}

void h89bus_z37_device::device_add_mconfig(machine_config &config)
{
	heath_z37_fdc_base_device::device_add_mconfig(config);

	// the list belongs to the controller rather than to any one machine: the
	// Z-37 came standard on the Z-90 and was an option for the H-89
	SOFTWARE_LIST(config, "flop_list").set_original("h37_flop");
}

void h89bus_z37_device::set_irq_out(int state)
{
	m_intr_cntrl->set_irq(state);
}

void h89bus_z37_device::set_drq_out(int state)
{
	m_intr_cntrl->set_drq(state);
}

void h89bus_z37_device::block_interrupts_out(int state)
{
	m_intr_cntrl->block_interrupts(state);
}

DEFINE_DEVICE_TYPE_PRIVATE(H89BUS_Z37, device_h89bus_right_card_interface, h89bus_z37_device, "h89_z37", "Heathkit Z-37 Floppy Disk Controller");
