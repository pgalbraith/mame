// license:BSD-3-Clause
// copyright-holders:AJR
/****************************************************************************

    DEC VT52 Video Display Terminal

    The VT50 "DECscope" was DEC's first video terminal to contain a CPU of
    sorts, with TTL logic spanning two boards executing custom microcode.
    It displayed 12 lines of 80-column text, using a standard character
    generator that only contained uppercase letters and symbols.

    The VT52 used the same case and most of the same circuitry as the VT50,
    but quickly displaced it by supporting 24 lines of text and a full ASCII
    character generator (on a board of its own). VT50 and VT52 each had
    minor variants differing in keyboard function and printer availability.

    The VT55 DECgraphic Scope was a graphical terminal based on the same
    main boards as the VT50 and VT52.

    The whole terminal is the DEC_VT52 device, so that it can also sit on
    any RS-232 port; this driver only gives it an EIA port.

****************************************************************************/

#include "emu.h"
#include "bus/rs232/rs232.h"
#include "machine/dec_vt52.h"


namespace {

class vt52_state : public driver_device
{
public:
	vt52_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_vt52(*this, "vt52")
		, m_eia(*this, "eia")
	{
	}

	void vt52(machine_config &config);

protected:
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<dec_vt52_device> m_vt52;
	required_device<rs232_port_device> m_eia;
};

void vt52_state::machine_reset()
{
	m_eia->write_dtr(0);
	m_eia->write_rts(0);
}

void vt52_state::vt52(machine_config &config)
{
	DEC_VT52(config, m_vt52);
	m_vt52->serial_data_callback().set(m_eia, FUNC(rs232_port_device::write_txd));

	RS232_PORT(config, m_eia, default_rs232_devices, nullptr);
	m_eia->rxd_handler().set(m_vt52, FUNC(dec_vt52_device::serial_in_w));
}

ROM_START(vt52)
ROM_END

} // anonymous namespace


COMP(1975, vt52, 0, 0, vt52, 0, vt52_state, empty_init, "Digital Equipment Corporation", "VT52 Video Display Terminal (M4)", MACHINE_SUPPORTS_SAVE | MACHINE_IMPERFECT_SOUND | MACHINE_NODEVICE_PRINTER)
