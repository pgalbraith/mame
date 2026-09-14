// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS 88-4PIO four port parallel interface, and 88-HDSK hard disk

    One to four 6820 PIAs, ICs J, K, L and M. Each is a port of two
    sections, A and B, with eight data lines and two control lines apiece,
    and everything about them but the board's address is set by software
    through the PIA registers.

    The board takes 16 I/O addresses. Jumpers compare A7-A4, A3-A2 pick the
    port, and A1-A0 the register: base+0 A control, base+1 A data or data
    direction, base+2 B control, base+3 B data or data direction. So each
    control register sits below its data register, the other way round from
    the 6820's own register select order. DBL expects the board at 040
    octal: it sends its error code to port 0's B data register at 043.

    The parts list is for a one port board, which is the default here; the
    PIAs for ports K, L and M, with their connector cables, were options. A
    port with no PIA reads as FF.

    Not emulated
    - anything on the ports' connectors. The inputs read high, as
      unconnected TTL inputs do, so no port can interrupt, and the jumpers
      that take each section's interrupt request to PINT are left out
    - the wait state the board inserts on every IN, about 500 ns

    88-HDSK
    The Altair hard disk: a Datakeeper controller, with an 8X300 processor,
    its firmware and 1K of buffer RAM, cabled to two ports of an 88-4PIO at
    240 octal, and a Pertec D3422 drive with a fixed platter and a 5440
    removable cartridge. Each platter has two surfaces of 406 cylinders of
    24 sectors of 256 bytes, about 5 megabytes.

    The computer writes a command's low byte to 247 and its high byte to
    243; the controller strobes CB1 of the first port to acknowledge it,
    CA1 of the first port when it is ready for the next command, with error
    flags on 241, and uses CA1 and CB1 of the second port to say it has a
    byte for 245 or wants one on 247. The commands, in the top four bits of
    the high byte, as the manual and the version 4.3 firmware listing give
    them, are:
    - 0000: seek to a cylinder
    - 0010, 0011: write or read a sector through one of four 256 byte
      buffers, naming the head (0-1 the removable platter, 2-3 the fixed
      platter) and sector
    - 0100, 0101: write or read up to 256 bytes of a buffer
    - 0110: read an IV byte (a status or control latch) of a drive
    - 1000: write an IV byte
    - 1010: read a sector unformatted
    - 1100: format the track under a head
    - 1110: initialize the controller
    The controller finishes a command before it takes the next; a command
    written while it is busy waits for it. Drive status IV bytes 21 and 22
    give ready, file protected, illegal address, seeking and the sector
    count, with the bit order the errata give for the computer.

    Each platter is an image of 4,988,928 bytes, the sectors in order of
    cylinder, surface and sector: hard1 is the removable cartridge and hard2
    the fixed platter. MITS's boot loader PROMs go in the 88-PMC at 176000.

    Not emulated on the 88-HDSK
    - the 8X300 and its firmware; the controller is modelled from the
      manual's description of its commands
    - sector headers and CRCs: the header and CRC error flags never set,
      except that an unformatted read reports the CRC error the errata say
      it always gets, and FORMAT only clears the track's data
    - drives 1-3, the extended platters of larger drives, and spin-up time
    - IV bytes other than the drive status ones hold only what was written

    References
    - MITS 88-4PIO documentation, third printing, March 1977
      [https://deramp.com/downloads/altair/hardware/MITS%2088-4PIO.pdf]
    - MITS 88-HDSK documentation, October 1977, with its errata
      [https://deramp.com/downloads/altair/hardware/hard_disk/88-HDSK.pdf]
    - 88-HDSK controller firmware, version 4.3 listing
      [https://deramp.com/downloads/altair/hardware/hard_disk/8X300%20version%204-3.pdf]
    - Martin Eberhard's disassembly of the HD-TBL boot loader, and Mike
      Douglas's hard disk CP/M boot and BIOS, which show how software drives
      the controller [https://deramp.com/downloads/altair/software/hard_disk/]

**********************************************************************/

#include "emu.h"
#include "mits4pio.h"

#include "machine/6821pia.h"

#include <cmath>
#include <cstdlib>


//**************************************************************************
//  Pertec D3422 platter image
//**************************************************************************

DECLARE_DEVICE_TYPE(MITS_HDSK_PLATTER, mits_hdsk_platter_device)

class mits_hdsk_platter_device : public device_t, public device_image_interface
{
public:
	static constexpr unsigned CYLINDERS = 406;
	static constexpr unsigned SECTORS = 24;
	static constexpr unsigned SECTOR_BYTES = 256;
	static constexpr u32 IMAGE_BYTES = CYLINDERS * 2 * SECTORS * SECTOR_BYTES;

	mits_hdsk_platter_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0)
		: device_t(mconfig, MITS_HDSK_PLATTER, tag, owner, clock)
		, device_image_interface(mconfig, *this)
	{
	}

	bool read_sector(unsigned cylinder, unsigned side, unsigned sector, u8 *data)
	{
		if (!is_loaded() || !seek(cylinder, side, sector))
			return false;
		return fread(data, SECTOR_BYTES) == SECTOR_BYTES;
	}

	bool write_sector(unsigned cylinder, unsigned side, unsigned sector, const u8 *data)
	{
		if (!is_loaded() || is_readonly() || !seek(cylinder, side, sector))
			return false;
		return fwrite(data, SECTOR_BYTES) == SECTOR_BYTES;
	}

	// device_image_interface implementation
	virtual bool is_readable() const noexcept override { return true; }
	virtual bool is_writeable() const noexcept override { return true; }
	virtual bool is_creatable() const noexcept override { return true; }
	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual bool support_command_line_image_creation() const noexcept override { return true; }
	virtual const char *file_extensions() const noexcept override { return "dsk"; }
	virtual const char *image_type_name() const noexcept override { return "harddisk"; }
	virtual const char *image_brief_type_name() const noexcept override { return "hard"; }

	virtual std::pair<std::error_condition, std::string> call_load() override
	{
		if (length() != IMAGE_BYTES)
			return std::make_pair(image_error::INVALIDLENGTH, std::string());
		return std::make_pair(std::error_condition(), std::string());
	}

	virtual std::pair<std::error_condition, std::string> call_create(int format_type, util::option_resolution *format_options) override
	{
		std::vector<u8> const blank(IMAGE_BYTES, 0);
		if (fwrite(blank.data(), IMAGE_BYTES) != IMAGE_BYTES)
			return std::make_pair(std::errc::io_error, std::string());
		return std::make_pair(std::error_condition(), std::string());
	}

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD { }

private:
	bool seek(unsigned cylinder, unsigned side, unsigned sector)
	{
		if ((cylinder >= CYLINDERS) || (side > 1) || (sector >= SECTORS))
			return false;
		return !fseek(((cylinder * 2 + side) * SECTORS + sector) * SECTOR_BYTES, SEEK_SET);
	}
};

DEFINE_DEVICE_TYPE(MITS_HDSK_PLATTER, mits_hdsk_platter_device, "mits_hdsk_platter", "Pertec D3422 Platter")


namespace {

//**************************************************************************
//  88-4PIO
//**************************************************************************

class s100_mits_4pio_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_4pio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	s100_mits_4pio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_sinp_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;

	// A7-A4 select the board and A3-A2 a port, which answers only if its PIA is fitted
	bool selected(offs_t offset) { return ((offset & 0xf0) == m_address->read()) && (BIT(offset, 2, 2) < m_ports->read()); }

	required_device_array<pia6821_device, 4> m_pia;
	required_ioport m_address;
	required_ioport m_ports;

private:
	// A1-A0 of 00 is the A control register, which the PIA has at register 01
	static offs_t pia_register(offs_t offset) { return (offset & 0x03) ^ 0x01; }
};


s100_mits_4pio_device::s100_mits_4pio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_pia(*this, "pia%u", 0U)
	, m_address(*this, "ADDRESS")
	, m_ports(*this, "PORTS")
{
}

s100_mits_4pio_device::s100_mits_4pio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: s100_mits_4pio_device(mconfig, S100_MITS_4PIO, tag, owner, clock)
{
}


void s100_mits_4pio_device::device_start()
{
}


u8 s100_mits_4pio_device::s100_sinp_r(offs_t offset)
{
	if (!selected(offset))
		return 0xff;

	return m_pia[BIT(offset, 2, 2)]->read(pia_register(offset));
}

void s100_mits_4pio_device::s100_sout_w(offs_t offset, u8 data)
{
	if (selected(offset))
		m_pia[BIT(offset, 2, 2)]->write(pia_register(offset), data);
}


void s100_mits_4pio_device::device_add_mconfig(machine_config &config)
{
	for (unsigned port = 0; port < 4; port++)
	{
		PIA6821(config, m_pia[port]);

		// nothing is connected: the inputs float high and the outputs go nowhere
		m_pia[port]->readpa_handler().set_constant(0xff);
		m_pia[port]->readpb_handler().set_constant(0xff);
		m_pia[port]->readca1_handler().set_constant(1);
		m_pia[port]->readca2_handler().set_constant(1);
		m_pia[port]->readcb1_handler().set_constant(1);
		m_pia[port]->writepa_handler().set_nop();
		m_pia[port]->writepb_handler().set_nop();
		m_pia[port]->ca2_handler().set_nop();
		m_pia[port]->cb2_handler().set_nop();
	}
}


#define ADDRESS_JUMPER(bit, dflt) \
	PORT_CONFNAME(1 << bit, (dflt) << bit, "Address jumper A" #bit) \
	PORT_CONFSETTING(0, "0") \
	PORT_CONFSETTING(1 << bit, "1")

#define PORT_SETTINGS \
	PORT_CONFSETTING(0x01, "1 (IC J)") \
	PORT_CONFSETTING(0x02, "2 (ICs J and K)") \
	PORT_CONFSETTING(0x03, "3 (ICs J, K and L)") \
	PORT_CONFSETTING(0x04, "4 (ICs J, K, L and M)")

static INPUT_PORTS_START( mits_4pio )
	// A7-A4 are compared with the jumpers; the default is 040 octal
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(7, 0)
	ADDRESS_JUMPER(6, 0)
	ADDRESS_JUMPER(5, 1)
	ADDRESS_JUMPER(4, 0)

	PORT_START("PORTS")
	PORT_CONFNAME(0x07, 0x01, "Ports fitted")
	PORT_SETTINGS
INPUT_PORTS_END

ioport_constructor s100_mits_4pio_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_4pio);
}


//**************************************************************************
//  88-HDSK
//**************************************************************************

class s100_mits_hdsk_device : public s100_mits_4pio_device
{
public:
	s100_mits_hdsk_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_sinp_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;

private:
	enum : u8
	{
		STATE_IDLE,             // ready for a command
		STATE_BUSY,             // seeking, waiting for a sector, or restoring
		STATE_WRITE_BUFFER,     // taking bytes from the computer
		STATE_READ_BUFFER,      // giving bytes to the computer
		STATE_SET_BYTE,         // waiting for an IV byte's data
		STATE_STATUS_WAIT       // a status command for a drive that is not ready
	};

	// error flags returned on 241
	static constexpr u8 ERROR_NOT_READY = 0x01;
	static constexpr u8 ERROR_ILLEGAL_SECTOR = 0x02;
	static constexpr u8 ERROR_DATA_CRC = 0x04;
	static constexpr u8 ERROR_WRITE_PROTECT = 0x80;

	static constexpr double ROTATION = 0.025;   // 2400 RPM

	void command();
	void data_written(u8 data);
	void data_read();
	void finish(u8 errors);
	u8 iv_byte(u8 address);
	bool drive_ready(unsigned unit);
	unsigned current_sector();

	// Each strobe is a low pulse on a line that rests high. The PIA only learns
	// that level from a transition, so each is driven high first.
	void strobe_ready(u8 errors) { m_errors = errors; m_pia[0]->ca1_w(1); m_pia[0]->ca1_w(0); m_pia[0]->ca1_w(1); }
	void strobe_acknowledge() { m_pia[0]->cb1_w(1); m_pia[0]->cb1_w(0); m_pia[0]->cb1_w(1); }
	void strobe_data_available() { m_pia[1]->ca1_w(1); m_pia[1]->ca1_w(0); m_pia[1]->ca1_w(1); }
	void strobe_port_available() { m_pia[1]->cb1_w(1); m_pia[1]->cb1_w(0); m_pia[1]->cb1_w(1); }

	TIMER_CALLBACK_MEMBER(operation_done);

	required_device_array<mits_hdsk_platter_device, 2> m_platter;

	emu_timer *m_timer;

	u8 m_control[2][2];     // the PIAs' control registers, as last written
	u8 m_errors;            // error flags on the first port's A lines
	u8 m_data;              // a byte for the computer on the second port's A lines
	u8 m_state;
	bool m_pending;         // a command written while busy
	u16 m_command;
	unsigned m_count;       // bytes to transfer
	unsigned m_position;
	u8 m_buffer[4][256];
	u16 m_cylinder;         // where drive 0's heads are
	bool m_illegal_address;
	u8 m_iv[256];
};


s100_mits_hdsk_device::s100_mits_hdsk_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: s100_mits_4pio_device(mconfig, S100_MITS_HDSK, tag, owner, clock)
	, m_platter(*this, "platter%u", 0U)
	, m_timer(nullptr)
	, m_control{ { 0, 0 }, { 0, 0 } }
	, m_errors(0xff)
	, m_data(0xff)
	, m_state(STATE_BUSY)
	, m_pending(false)
	, m_command(0)
	, m_count(0)
	, m_position(0)
	, m_buffer{ }
	, m_cylinder(0)
	, m_illegal_address(false)
	, m_iv{ }
{
}


void s100_mits_hdsk_device::device_start()
{
	s100_mits_4pio_device::device_start();

	m_timer = timer_alloc(FUNC(s100_mits_hdsk_device::operation_done), this);

	save_item(NAME(m_control));
	save_item(NAME(m_errors));
	save_item(NAME(m_data));
	save_item(NAME(m_state));
	save_item(NAME(m_pending));
	save_item(NAME(m_command));
	save_item(NAME(m_count));
	save_item(NAME(m_position));
	save_item(NAME(m_buffer));
	save_item(NAME(m_cylinder));
	save_item(NAME(m_illegal_address));
	save_item(NAME(m_iv));
}

void s100_mits_hdsk_device::device_reset()
{
	// the firmware restores the drives to cylinder 0, then says it is ready,
	// with all the error flags set until the first command
	m_state = STATE_BUSY;
	m_pending = false;
	m_cylinder = 0;
	m_illegal_address = false;
	m_errors = 0xff;
	m_timer->adjust(attotime::from_msec(100), 0xff);
}


u8 s100_mits_hdsk_device::s100_sinp_r(offs_t offset)
{
	u8 const data = s100_mits_4pio_device::s100_sinp_r(offset);

	// a read of the second port's A data register strobes CA2: the byte is taken
	if (selected(offset) && ((offset & 0x0f) == 0x05) && BIT(m_control[1][0], 2) && !machine().side_effects_disabled())
		data_read();

	return data;
}

void s100_mits_hdsk_device::s100_sout_w(offs_t offset, u8 data)
{
	s100_mits_4pio_device::s100_sout_w(offset, data);

	if (!selected(offset))
		return;

	unsigned const port = BIT(offset, 2, 2);
	switch (offset & 0x03)
	{
	case 0: // A control
	case 2: // B control
		if (port < 2)
			m_control[port][BIT(offset, 1)] = data;
		break;

	case 3: // B data
		if ((port == 0) && BIT(m_control[0][1], 2))
		{
			// the high byte starts a command, which waits if the controller is busy
			if (m_state == STATE_IDLE)
				command();
			else
				m_pending = true;
		}
		else if ((port == 1) && BIT(m_control[1][1], 2))
		{
			data_written(data);
		}
		break;
	}
}


void s100_mits_hdsk_device::command()
{
	m_command = (m_pia[0]->b_output() << 8) | m_pia[1]->b_output();
	strobe_acknowledge();

	unsigned const unit = BIT(m_command, 10, 2);
	switch (m_command >> 12)
	{
	case 0x0:
		{
			// seek, taking about 10 ms for a track and 65 ms for all of them
			unsigned const cylinder = m_command & 0x1ff;
			if (!drive_ready(unit))
			{
				m_state = STATE_BUSY;
				m_timer->adjust(attotime::from_usec(50), ERROR_NOT_READY);
				break;
			}
			m_illegal_address = (cylinder >= mits_hdsk_platter_device::CYLINDERS);
			unsigned const distance = m_illegal_address ? 0 : std::abs(int(cylinder) - int(m_cylinder));
			if (!m_illegal_address)
				m_cylinder = cylinder;
			m_state = STATE_BUSY;
			m_timer->adjust(distance ? attotime::from_double((10.0 + 55.0 * (distance - 1) / 404.0) / 1000.0) : attotime::from_usec(50), 0);
		}
		break;

	case 0x2:
	case 0x3:
	case 0xa:
		{
			bool const read = (m_command >> 12) != 0x2;
			unsigned const sector = m_command & 0x1f;
			unsigned const head = BIT(m_command, 5, 3);
			unsigned const buffer = BIT(m_command, 8, 2);
			u8 errors = 0;

			m_state = STATE_BUSY;
			if (!drive_ready(unit) || (head > 3) || !m_platter[head >> 1]->exists())
			{
				errors = ERROR_NOT_READY;
			}
			else if (sector >= mits_hdsk_platter_device::SECTORS)
			{
				errors = ERROR_ILLEGAL_SECTOR;
			}
			else if (read)
			{
				m_platter[head >> 1]->read_sector(m_cylinder, head & 1, sector, m_buffer[buffer]);

				// the header and CRC are read as data, which never checks
				if ((m_command >> 12) == 0xa)
					errors = ERROR_DATA_CRC;
			}
			else if (!m_platter[head >> 1]->write_sector(m_cylinder, head & 1, sector, m_buffer[buffer]))
			{
				errors = ERROR_WRITE_PROTECT;
			}

			if (errors & (ERROR_NOT_READY | ERROR_ILLEGAL_SECTOR))
			{
				m_timer->adjust(attotime::from_usec(50), errors);
			}
			else
			{
				// wait for the sector to come round, then for it to pass the head
				double const sector_time = ROTATION / mits_hdsk_platter_device::SECTORS;
				double const turns = machine().time().as_double() / ROTATION;
				double wait = sector - (turns - std::floor(turns)) * mits_hdsk_platter_device::SECTORS;
				if (wait < 0.0)
					wait += mits_hdsk_platter_device::SECTORS;
				m_timer->adjust(attotime::from_double((wait + 1.0) * sector_time), errors);
			}
		}
		break;

	case 0x4:
	case 0x5:
		m_count = (m_command & 0xff) ? (m_command & 0xff) : 256;
		m_position = 0;
		if (BIT(m_command, 12))
		{
			m_state = STATE_READ_BUFFER;
			m_data = m_buffer[BIT(m_command, 8, 2)][0];
			strobe_data_available();
		}
		else
		{
			m_state = STATE_WRITE_BUFFER;
			strobe_port_available();
		}
		break;

	case 0x6:
		// a status command waits for the drive to be ready
		if (drive_ready(unit))
		{
			m_data = iv_byte(m_command & 0xff);
			strobe_data_available();
			finish(0);
		}
		else
		{
			m_state = STATE_STATUS_WAIT;
		}
		break;

	case 0x8:
		m_state = STATE_SET_BYTE;
		strobe_port_available();
		break;

	case 0xc:
		{
			// format the track under a head, which takes a revolution to find
			// the index and one to write
			unsigned const head = BIT(m_command, 5, 3);
			u8 errors = 0;
			if (!drive_ready(unit) || (head > 3) || !m_platter[head >> 1]->exists())
			{
				errors = ERROR_NOT_READY;
			}
			else if (m_platter[head >> 1]->is_readonly())
			{
				errors = ERROR_WRITE_PROTECT;
			}
			else
			{
				static constexpr u8 BLANK[mits_hdsk_platter_device::SECTOR_BYTES] = { };
				for (unsigned sector = 0; sector < mits_hdsk_platter_device::SECTORS; sector++)
					m_platter[head >> 1]->write_sector(m_cylinder, head & 1, sector, BLANK);
			}
			m_state = STATE_BUSY;
			m_timer->adjust(attotime::from_double(ROTATION * 2.0), errors);
		}
		break;

	case 0xe:
		// initialize, as at power on: restore the drives, then report ready
		m_state = STATE_BUSY;
		m_cylinder = 0;
		m_illegal_address = false;
		m_timer->adjust(attotime::from_msec(100), 0);
		break;

	default:
		// the firmware's jump table has nothing here
		finish(0);
		break;
	}
}

void s100_mits_hdsk_device::data_written(u8 data)
{
	if (m_state == STATE_WRITE_BUFFER)
	{
		m_buffer[BIT(m_command, 8, 2)][m_position] = data;
		if (++m_position == m_count)
			finish(0);
	}
	else if (m_state == STATE_SET_BYTE)
	{
		m_iv[m_command & 0xff] = data;
		finish(0);
	}
}

void s100_mits_hdsk_device::data_read()
{
	if (m_state != STATE_READ_BUFFER)
		return;

	if (++m_position < m_count)
		m_data = m_buffer[BIT(m_command, 8, 2)][m_position];
	else
		finish(0);
}

void s100_mits_hdsk_device::finish(u8 errors)
{
	m_state = STATE_IDLE;
	strobe_ready(errors);
	if (m_pending)
	{
		m_pending = false;
		command();
	}
}

TIMER_CALLBACK_MEMBER(s100_mits_hdsk_device::operation_done)
{
	finish(u8(param));
}


u8 s100_mits_hdsk_device::iv_byte(u8 address)
{
	switch (address)
	{
	case 21:
		// ready, index, file protected, illegal address, busy seeking 4-1
		return
				(drive_ready(0) ? 0x80 : 0x00) |
				((m_platter[0]->exists() && m_platter[0]->is_readonly()) ? 0x20 : 0x00) |
				(m_illegal_address ? 0x10 : 0x00) |
				((m_state == STATE_BUSY) ? 0x01 : 0x00);

	case 22:
		// sector pulse, and the sector count
		return current_sector();

	default:
		return m_iv[address];
	}
}

bool s100_mits_hdsk_device::drive_ready(unsigned unit)
{
	return (unit == 0) && (m_platter[0]->exists() || m_platter[1]->exists());
}

unsigned s100_mits_hdsk_device::current_sector()
{
	double const turns = machine().time().as_double() / ROTATION;
	return unsigned((turns - std::floor(turns)) * mits_hdsk_platter_device::SECTORS) % mits_hdsk_platter_device::SECTORS;
}


void s100_mits_hdsk_device::device_add_mconfig(machine_config &config)
{
	s100_mits_4pio_device::device_add_mconfig(config);

	// the controller drives the A lines of both ports
	m_pia[0]->readpa_handler().set([this] () { return m_errors; });
	m_pia[1]->readpa_handler().set([this] () { return m_data; });

	// hard1 is the removable cartridge, hard2 the fixed platter
	MITS_HDSK_PLATTER(config, m_platter[0]);
	MITS_HDSK_PLATTER(config, m_platter[1]);
}


static INPUT_PORTS_START( mits_hdsk )
	PORT_INCLUDE(mits_4pio)

	// the controller is cabled to two ports at 240 octal
	PORT_MODIFY("ADDRESS")
	ADDRESS_JUMPER(7, 1)
	ADDRESS_JUMPER(6, 0)
	ADDRESS_JUMPER(5, 1)
	ADDRESS_JUMPER(4, 0)

	PORT_MODIFY("PORTS")
	PORT_CONFNAME(0x07, 0x02, "Ports fitted")
	PORT_SETTINGS
INPUT_PORTS_END

ioport_constructor s100_mits_hdsk_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_hdsk);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_4PIO, device_s100_card_interface, s100_mits_4pio_device, "s100_mits_4pio", "MITS 88-4PIO Parallel Interface")
DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_HDSK, device_s100_card_interface, s100_mits_hdsk_device, "s100_mits_hdsk", "MITS 88-HDSK Hard Disk")
