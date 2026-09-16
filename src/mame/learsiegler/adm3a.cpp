// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/***************************************************************************

  Lear Siegler ADM-3A Dumb Terminal

    Lear Siegler's cheap 24 by 80 terminal, sold from 1976 and named for what
    it does not do.  It has no processor: TTL counters put the characters on
    the screen and a handful of decoders act on the fourteen control codes it
    understands.

    Everything that makes the terminal behave the way it does lives in the
    ADM3A device; this driver only gives it a serial port to talk to.

****************************************************************************/

#include "emu.h"

#include "bus/rs232/rs232.h"
#include "machine/adm3a.h"


namespace {

class adm3a_state : public driver_device
{
public:
	adm3a_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_adm3a(*this, "adm3a")
	{
	}

	void adm3a(machine_config &config);

private:
	required_device<adm3a_device> m_adm3a;
};

void adm3a_state::adm3a(machine_config &config)
{
	ADM3A(config, m_adm3a);
	m_adm3a->serial_data_callback().set("modem", FUNC(rs232_port_device::write_txd));

	// The MODEM connector, J1.  Loopback by default, which is the only way to
	// see anything typed on a terminal that comes up in full duplex.
	rs232_port_device &modem(RS232_PORT(config, "modem", default_rs232_devices, "loopback"));
	modem.rxd_handler().set(m_adm3a, FUNC(adm3a_device::serial_in_w));
}

ROM_START( adm3a )
ROM_END

} // anonymous namespace

//    year  name   parent  compat  machine  input  class        init         company          fullname                    flags
COMP( 1976, adm3a, 0,      0,      adm3a,   0,     adm3a_state, empty_init,  "Lear Siegler",  "ADM-3A Dumb Terminal",     MACHINE_SUPPORTS_SAVE )
