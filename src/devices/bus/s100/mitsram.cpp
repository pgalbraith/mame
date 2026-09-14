// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    MITS Altair static RAM boards

    88-1MCS (1975)
    Intel 8101 RAMs fitted 256 bytes at a time, up to 1K; MITS sold the
    extra 256 byte steps as the 88-MM. Six jumpers put the board in any
    1K block. A protect latch is set and cleared from the front panel.
    Schematic 880-107 shows nothing clearing it at power-on, so it could
    come up either way; here it starts unprotected. The board also asks
    for two wait states on every read to suit the slow 8101, which is not
    emulated.

    88-4MCS (1976)
    2102 RAMs, 4K, or 2K when sold as the 88-2MCS. A four position DIP
    switch sets A12-A15. Its protect latch is cleared at power-on.

    88-16MCS (1976)
    4200 RAMs, 16K. Four switches, one per 16K block; closing exactly one
    is intended, and closing two makes the board answer in both. No
    memory protect.

    A protected board ignores writes and pulls PS* low when it is read,
    which lights the PROT LED on the front panel.

    References
    - MITS price list, 1 January 1976, for the part numbers
      [https://ubuntourist.codeberg.page/Altair-8800/price-list.html]
    - Altair 8800 Theory of Operation, pages 7-8 cover the 88-1MCS as the
      "1K Static Memory Board"
      [https://altairclone.com/downloads/manuals/Altair%208800%20Theory%20of%20Operation.pdf]
    - 88-1MCS schematic 880-107 [https://deramp.com/downloads/altair/hardware/altair_8800_computer/Altair%20Schematics.pdf]
    - 88-4MCS manual [https://deramp.com/downloads/altair/hardware/MITS%2088-4MCS%204K%20Static%20RAM.pdf]
    - 88-16MCS documentation, April 1977 [http://www.bitsavers.org/pdf/mits/8800/Altair_88-16K_SRAM_Documentation_197704.pdf]

**********************************************************************/

#include "emu.h"
#include "mitsram.h"


namespace {

//**************************************************************************
//  88-1MCS
//**************************************************************************

class s100_mits_1mcs_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_1mcs_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_smemr_r(offs_t offset) override;
	virtual void s100_mwrt_w(offs_t offset, u8 data) override;
	virtual void s100_prot_w(offs_t offset) override;
	virtual void s100_unprot_w(offs_t offset) override;
	virtual int s100_ps_r(offs_t offset) override;

private:
	// A10-A15 select the board; A8 and A9 pick one of the 256 byte pairs
	bool board_selected(offs_t offset) { return (offset & 0xfc00) == (m_address->read() << 10); }
	bool ram_selected(offs_t offset) { return board_selected(offset) && ((offset >> 8) & 3) <= m_size->read(); }

	required_ioport m_address;
	required_ioport m_size;
	u8 m_ram[0x400];
	bool m_protected;
};

s100_mits_1mcs_device::s100_mits_1mcs_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_1MCS, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_address(*this, "ADDRESS")
	, m_size(*this, "SIZE")
	, m_ram{ }
	, m_protected(false)
{
}

void s100_mits_1mcs_device::device_start()
{
	save_item(NAME(m_ram));
	save_item(NAME(m_protected));
}

u8 s100_mits_1mcs_device::s100_smemr_r(offs_t offset)
{
	return ram_selected(offset) ? m_ram[offset & 0x3ff] : 0xff;
}

void s100_mits_1mcs_device::s100_mwrt_w(offs_t offset, u8 data)
{
	if (ram_selected(offset) && !m_protected)
		m_ram[offset & 0x3ff] = data;
}

void s100_mits_1mcs_device::s100_prot_w(offs_t offset)
{
	if (board_selected(offset))
		m_protected = true;
}

void s100_mits_1mcs_device::s100_unprot_w(offs_t offset)
{
	if (board_selected(offset))
		m_protected = false;
}

int s100_mits_1mcs_device::s100_ps_r(offs_t offset)
{
	return (board_selected(offset) && m_protected) ? 0 : 1;
}

#define ADDRESS_JUMPER(bit, name) \
	PORT_CONFNAME(1 << (bit - 10), 0, "Address jumper " name) \
	PORT_CONFSETTING(0, "0") \
	PORT_CONFSETTING(1 << (bit - 10), "1")

static INPUT_PORTS_START( mits_1mcs )
	PORT_START("ADDRESS")
	ADDRESS_JUMPER(15, "A15")
	ADDRESS_JUMPER(14, "A14")
	ADDRESS_JUMPER(13, "A13")
	ADDRESS_JUMPER(12, "A12")
	ADDRESS_JUMPER(11, "A11")
	ADDRESS_JUMPER(10, "A10")

	PORT_START("SIZE")
	PORT_CONFNAME(0x03, 0x03, "RAM fitted")
	PORT_CONFSETTING(0x00, "256 bytes")
	PORT_CONFSETTING(0x01, "512 bytes")
	PORT_CONFSETTING(0x02, "768 bytes")
	PORT_CONFSETTING(0x03, "1K")
INPUT_PORTS_END

ioport_constructor s100_mits_1mcs_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_1mcs);
}


//**************************************************************************
//  88-4MCS
//**************************************************************************

class s100_mits_4mcs_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_4mcs_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_smemr_r(offs_t offset) override;
	virtual void s100_mwrt_w(offs_t offset, u8 data) override;
	virtual void s100_prot_w(offs_t offset) override;
	virtual void s100_unprot_w(offs_t offset) override;
	virtual int s100_ps_r(offs_t offset) override;

private:
	// A12-A15 select the board; A10 and A11 pick one of four rows of RAM, two on a 2K board
	bool board_selected(offs_t offset) { return (offset >> 12) == m_switches->read(); }
	bool ram_selected(offs_t offset) { return board_selected(offset) && ((offset >> 10) & 3) <= m_size->read(); }

	required_ioport m_switches;
	required_ioport m_size;
	u8 m_ram[0x1000];
	bool m_protected;
};

s100_mits_4mcs_device::s100_mits_4mcs_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_4MCS, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_switches(*this, "S1")
	, m_size(*this, "SIZE")
	, m_ram{ }
	, m_protected(false)
{
}

void s100_mits_4mcs_device::device_start()
{
	save_item(NAME(m_ram));
	save_item(NAME(m_protected));
}

void s100_mits_4mcs_device::device_reset()
{
	// POC clears the protect latch
	m_protected = false;
}

u8 s100_mits_4mcs_device::s100_smemr_r(offs_t offset)
{
	return ram_selected(offset) ? m_ram[offset & 0xfff] : 0xff;
}

void s100_mits_4mcs_device::s100_mwrt_w(offs_t offset, u8 data)
{
	if (ram_selected(offset) && !m_protected)
		m_ram[offset & 0xfff] = data;
}

void s100_mits_4mcs_device::s100_prot_w(offs_t offset)
{
	if (board_selected(offset))
		m_protected = true;
}

void s100_mits_4mcs_device::s100_unprot_w(offs_t offset)
{
	if (board_selected(offset))
		m_protected = false;
}

int s100_mits_4mcs_device::s100_ps_r(offs_t offset)
{
	// On schematic 8800-131 the PS* driver appears to be enabled by the read
	// enable, so it only reports where RAM answers. That is a reading of the
	// drawing, not something the manual says.
	return (ram_selected(offset) && m_protected) ? 0 : 1;
}

static INPUT_PORTS_START( mits_4mcs )
	// a switch that is on selects a 1 in that address bit
	PORT_START("S1")
	PORT_DIPNAME(0x08, 0x00, "Address A15")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x08, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "Address A14")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x04, DEF_STR(On))
	PORT_DIPNAME(0x02, 0x00, "Address A13")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x02, DEF_STR(On))
	PORT_DIPNAME(0x01, 0x00, "Address A12")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x01, DEF_STR(On))

	PORT_START("SIZE")
	PORT_CONFNAME(0x03, 0x03, "RAM fitted")
	PORT_CONFSETTING(0x01, "2K (88-2MCS)")
	PORT_CONFSETTING(0x03, "4K")
INPUT_PORTS_END

ioport_constructor s100_mits_4mcs_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_4mcs);
}


//**************************************************************************
//  88-16MCS
//**************************************************************************

class s100_mits_16mcs_device : public device_t, public device_s100_card_interface
{
public:
	s100_mits_16mcs_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_smemr_r(offs_t offset) override;
	virtual void s100_mwrt_w(offs_t offset, u8 data) override;

private:
	// A14 and A15 are decoded to four outputs, each through its own switch
	bool board_selected(offs_t offset) { return BIT(m_switches->read(), offset >> 14); }

	required_ioport m_switches;
	u8 m_ram[0x4000];
};

s100_mits_16mcs_device::s100_mits_16mcs_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_MITS_16MCS, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_switches(*this, "SW")
	, m_ram{ }
{
}

void s100_mits_16mcs_device::device_start()
{
	save_item(NAME(m_ram));
}

u8 s100_mits_16mcs_device::s100_smemr_r(offs_t offset)
{
	return board_selected(offset) ? m_ram[offset & 0x3fff] : 0xff;
}

void s100_mits_16mcs_device::s100_mwrt_w(offs_t offset, u8 data)
{
	if (board_selected(offset))
		m_ram[offset & 0x3fff] = data;
}

static INPUT_PORTS_START( mits_16mcs )
	PORT_START("SW")
	PORT_DIPNAME(0x01, 0x01, "Address 000000-037777 (0-16K)")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x01, DEF_STR(On))
	PORT_DIPNAME(0x02, 0x00, "Address 040000-077777 (16K-32K)")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x02, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "Address 100000-137777 (32K-48K)")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x04, DEF_STR(On))
	PORT_DIPNAME(0x08, 0x00, "Address 140000-177777 (48K-64K)")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x08, DEF_STR(On))
INPUT_PORTS_END

ioport_constructor s100_mits_16mcs_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(mits_16mcs);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_1MCS, device_s100_card_interface, s100_mits_1mcs_device, "s100_mits_1mcs", "MITS 88-1MCS 1K Static RAM")
DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_4MCS, device_s100_card_interface, s100_mits_4mcs_device, "s100_mits_4mcs", "MITS 88-4MCS 4K Static RAM")
DEFINE_DEVICE_TYPE_PRIVATE(S100_MITS_16MCS, device_s100_card_interface, s100_mits_16mcs_device, "s100_mits_16mcs", "MITS 88-16MCS 16K Static RAM")
