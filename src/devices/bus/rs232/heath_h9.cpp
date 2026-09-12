// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/******************************************************************************

    Heath H9 Terminal (serial interface)

    Only three wires reach the terminal - Data+, Data- and ground - so none of
    the handshake lines are connected here.  That is not a simplification: the
    H9 has no handshaking on its serial port at all, and Pictorial 2-3 of the
    H9 Operations manual shows the H8 cable carrying exactly those three.

******************************************************************************/

#include "emu.h"
#include "heath_h9.h"

#include "bus/heathzenith/h9/h9.h"


namespace {

class serial_heath_h9_device : public device_t, public device_rs232_port_interface
{
public:
	serial_heath_h9_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: device_t(mconfig, SERIAL_TERMINAL_H9, tag, owner, clock)
		, device_rs232_port_interface(mconfig, *this)
		, m_h9(*this, "h9")
	{
	}

	virtual void input_txd(int state) override { m_h9->serial_in_w(state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	required_device<heath_h9_device> m_h9;
};

void serial_heath_h9_device::device_start()
{
	// Nothing drives the handshake lines, so hold them all asserted.  That is
	// what the machine at the other end sees with only three wires in the
	// cable, and it matters: the H-8-5's console 8251 takes CTS from this port
	// and will not raise TxRDY while it reads as deasserted, so leaving these
	// alone gives an H8 that never gets past its first character.
	output_rxd(1);
	output_dcd(0);
	output_dsr(0);
	output_cts(0);
}

void serial_heath_h9_device::device_add_mconfig(machine_config &config)
{
	HEATH_H9(config, m_h9);
	m_h9->serial_data_callback().set(FUNC(serial_heath_h9_device::output_rxd));
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SERIAL_TERMINAL_H9, device_rs232_port_interface, serial_heath_h9_device, "serial_heath_h9", "Heath H9 Terminal (Serial Port)")
