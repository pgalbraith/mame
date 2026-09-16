// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/******************************************************************************

    DEC VT52 Video Display Terminal (serial interface)

    The terminal itself is the DEC_VT52 device; this only puts it on a port.

******************************************************************************/

#include "emu.h"
#include "dec_vt52.h"

#include "machine/dec_vt52.h"


namespace {

class serial_dec_vt52_device : public device_t, public device_rs232_port_interface
{
public:
	serial_dec_vt52_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: device_t(mconfig, SERIAL_TERMINAL_VT52, tag, owner, clock)
		, device_rs232_port_interface(mconfig, *this)
		, m_vt52(*this, "vt52")
	{
	}

	virtual void input_txd(int state) override { m_vt52->serial_in_w(state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	required_device<dec_vt52_device> m_vt52;
};

void serial_dec_vt52_device::device_start()
{
	// The terminal asserts DATA TERMINAL READY and REQUEST TO SEND and never
	// changes either, so hold the host's handshake inputs asserted; a host
	// that waits for CTS would otherwise never send.
	output_rxd(1);
	output_dcd(0);
	output_dsr(0);
	output_cts(0);
}

void serial_dec_vt52_device::device_add_mconfig(machine_config &config)
{
	DEC_VT52(config, m_vt52);
	m_vt52->serial_data_callback().set(FUNC(serial_dec_vt52_device::output_rxd));

	// The VT52 clocks its UART from the CPU's timing chain, so it sends and samples every bit at wherever its CPU has
	// got to. With another CPU-clocked UART on the other end and nothing timer-driven in between, the scheduler can let
	// one CPU run a whole frame ahead and every character comes through garbled. A quarter of a bit at 9600 baud keeps
	// the two close enough to sample mid-bit.
	config.set_maximum_quantum(attotime::from_hz(4 * 9600));
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(SERIAL_TERMINAL_VT52, device_rs232_port_interface, serial_dec_vt52_device, "serial_dec_vt52", "DEC VT52 Video Display Terminal (Serial Port)")
