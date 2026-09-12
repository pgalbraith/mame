// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Heathkit H9 Video Terminal

    Heath's first terminal, sold from 1977 and two years older than the H19.
    It has no processor: twelve lines of eighty upper-case characters are put
    on the screen by TTL, and a timing and processing unit sequences the
    writing, scrolling and erasing during vertical retrace.

    Everything that makes the terminal behave the way it does lives in the
    HEATH_H9 device; this driver only gives it a serial port to talk to.

****************************************************************************/

#include "emu.h"

#include "bus/heathzenith/h9/h9.h"
#include "bus/rs232/rs232.h"


namespace {

class h9_state : public driver_device
{
public:
	h9_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_h9(*this, "h9")
	{
	}

	void h9(machine_config &config);

private:
	required_device<heath_h9_device> m_h9;
};

void h9_state::h9(machine_config &config)
{
	HEATH_H9(config, m_h9);
	m_h9->serial_data_callback().set("dte", FUNC(rs232_port_device::write_txd));

	rs232_port_device &dte(RS232_PORT(config, "dte", default_rs232_devices, "loopback"));
	dte.rxd_handler().set(m_h9, FUNC(heath_h9_device::serial_in_w));
}

ROM_START( h9 )
ROM_END

} // anonymous namespace

//    year  name  parent  compat  machine input  class      init         company           fullname         flags
COMP( 1977, h9,   0,      0,      h9,     0,     h9_state,  empty_init,  "Heath Company",  "H-9 Terminal",  MACHINE_SUPPORTS_SAVE )
