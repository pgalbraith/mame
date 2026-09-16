// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/******************************************************************************

    Lear Siegler ADM-3A Dumb Terminal (serial interface)

    The terminal itself is the ADM3A device; this only puts it on a port.

    Set the rate with the Machine Configuration menu, or wire it from the
    machine with set_preset_baud().  9600 is the terminal's factory setting and
    is what this comes up at, so a host running slower will print rubbish until
    one of the two is changed.

******************************************************************************/

#include "emu.h"
#include "adm3a.h"

#include "machine/adm3a.h"


namespace {

class serial_adm3a_device : public device_t, public device_rs232_port_interface
{
public:
	serial_adm3a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: device_t(mconfig, SERIAL_TERMINAL_ADM3A, tag, owner, clock)
		, device_rs232_port_interface(mconfig, *this)
		, m_adm3a(*this, "adm3a")
	{
	}

	virtual void input_txd(int state) override { m_adm3a->serial_in_w(state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	required_device<adm3a_device> m_adm3a;
};

void serial_adm3a_device::device_start()
{
	// The ADM-3A drives Request to Send and reads Clear to Send, but only the
	// modem turnaround modes make anything of either and those are not
	// modelled.  Hold the handshake lines asserted so a host that does gate on
	// them - the H-8-5's 8251 will not raise TxRDY without CTS, for one - sees
	// what a three wire cable would give it.
	output_rxd(1);
	output_dcd(0);
	output_dsr(0);
	output_cts(0);
}

void serial_adm3a_device::device_add_mconfig(machine_config &config)
{
	ADM3A(config, m_adm3a);
	m_adm3a->serial_data_callback().set(FUNC(serial_adm3a_device::output_rxd));
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SERIAL_TERMINAL_ADM3A, device_rs232_port_interface, serial_adm3a_device, "serial_adm3a", "Lear Siegler ADM-3A Terminal (Serial Port)")
