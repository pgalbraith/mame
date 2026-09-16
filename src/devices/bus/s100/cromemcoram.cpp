// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    Cromemco RAM boards

    16KZ (1977)
    16K of 4Kx1 dynamic RAM in one block. Two switches set A14 and A15,
    putting the block at 0000, 4000, 8000 or C000.

    64KZ (1979)
    64K of 4116 dynamic RAM in two 32K blocks, A and B. Each block has its
    own A15 switch, its own eight bank switches and its own reset setting,
    so the two halves are configured separately. Both blocks may sit in the
    same half of the address space if they are in different banks.

    Bank select
    Every Cromemco memory and PROM card with bank select carries an output
    port at 40H, and one write reaches all of them. Each bit of the byte
    controls one bank: bit 0 is bank 0 and bit 7 is bank 7. A card enables
    when any bit that is set matches one of its own bank switches, and
    disables when none do. A disabled card stops answering, which frees the
    address space for another card.

    The two cards differ in what happens at reset. On the 16KZ the bank 0
    switch decides: the card comes up enabled only if it is mapped into
    bank 0. On the 64KZ each block has a RESET switch, IN or OUT, that sets
    that block's state directly. The CDOS setup in the 64KZ manual uses
    this: block A at 0000 with RESET IN, block B at 8000 with RESET OUT, so
    the 4FDC's RDOS at C000 answers until the boot sector writes 01 to port
    40H, which turns block B on and RDOS off.

    Not emulated
    - Refresh. Both cards refresh during M1 and on their own while the CPU
      is stopped, and neither needs wait states at 2 or 4 MHz. RAM here
      never fades.
    - The 64KZ's IC60 socket, which takes a 74905 PROM (a programmed 74S288)
      to move the bank select port to any of 40H-4FH or C0H-CFH. The socket
      is empty as shipped and the card answers 40H, which is what this does.
    - MDSBL, bus pin 67, which disables a card while it is low. The 16KZ
      manual says this lets a ROM bootstrap overlap RAM. The bus has a
      s100_phantom_w hook, but s100_bus_device never drives it, and nothing
      in a Z-2 pulls pin 67 anyway.

    References
    - 16KZ manual 023-0007, January 1982 edition, which is typeset and
      readable where the February 1979 scan is not. Bank select logic on
      page 9, parts list on page 15, schematic on page 20.
    - 64KZ manual 023-0008, January 1980. Bank select on printed page 24,
      the IC60 PROM option on printed page 18. This scan has no schematic
      although its contents page lists one.

**********************************************************************/

#include "emu.h"
#include "cromemcoram.h"


namespace {

// one switch per bank, shared by every Cromemco card with bank select
#define BANK_SWITCH(bit) \
	PORT_DIPNAME(1 << (bit), 1 << (bit), "Bank " #bit) \
	PORT_DIPSETTING(0, DEF_STR(Off)) \
	PORT_DIPSETTING(1 << (bit), DEF_STR(On))


//**************************************************************************
//  16KZ
//**************************************************************************

class s100_cromemco_16kz_device : public device_t, public device_s100_card_interface
{
public:
	s100_cromemco_16kz_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_smemr_r(offs_t offset) override;
	virtual void s100_mwrt_w(offs_t offset, u8 data) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;
	virtual void s100_phlda_w(int state) override;

private:
	// the two address switches compare against A14 and A15
	bool addressed(offs_t offset) const { return (offset >> 14) == m_address->read(); }
	bool selected(offs_t offset) const;

	required_ioport m_address;
	required_ioport m_banks;
	required_ioport m_dma;
	u8 m_ram[0x4000];
	bool m_enabled;
	int m_phlda;
};

s100_cromemco_16kz_device::s100_cromemco_16kz_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_CROMEMCO_16KZ, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_address(*this, "ADDRESS")
	, m_banks(*this, "BANKS")
	, m_dma(*this, "DMA")
	, m_ram{ }
	, m_enabled(false)
	, m_phlda(0)
{
}

void s100_cromemco_16kz_device::device_start()
{
	save_item(NAME(m_ram));
	save_item(NAME(m_enabled));
	save_item(NAME(m_phlda));
}

void s100_cromemco_16kz_device::device_reset()
{
	// RESET and POC hand the board to the bank 0 switch
	m_enabled = BIT(m_banks->read(), 0);
}

bool s100_cromemco_16kz_device::selected(offs_t offset) const
{
	if (!addressed(offset))
		return false;

	// With the override enabled the bank latch is ignored during DMA and the
	// lockout switch decides instead. Nothing drives pHLDA in a Z-2, so this
	// only matters once a bus master card exists.
	u8 const dma = m_dma->read();
	if (m_phlda && BIT(dma, 1))
		return !BIT(dma, 0);

	return m_enabled;
}

u8 s100_cromemco_16kz_device::s100_smemr_r(offs_t offset)
{
	return selected(offset) ? m_ram[offset & 0x3fff] : 0xff;
}

void s100_cromemco_16kz_device::s100_mwrt_w(offs_t offset, u8 data)
{
	if (selected(offset))
		m_ram[offset & 0x3fff] = data;
}

void s100_cromemco_16kz_device::s100_sout_w(offs_t offset, u8 data)
{
	// the port decodes A0-A7 only, so mask off whatever the CPU left in the
	// high half of the I/O address
	if ((offset & 0xff) == 0x40)
		m_enabled = (data & m_banks->read()) != 0;
}

void s100_cromemco_16kz_device::s100_phlda_w(int state)
{
	m_phlda = state;
}

static INPUT_PORTS_START( cromemco_16kz )
	PORT_START("ADDRESS")
	PORT_DIPNAME(0x03, 0x00, "Address")
	PORT_DIPSETTING(0x00, "0000-3FFF")
	PORT_DIPSETTING(0x01, "4000-7FFF")
	PORT_DIPSETTING(0x02, "8000-BFFF")
	PORT_DIPSETTING(0x03, "C000-FFFF")

	PORT_START("BANKS")
	BANK_SWITCH(0)
	BANK_SWITCH(1)
	BANK_SWITCH(2)
	BANK_SWITCH(3)
	BANK_SWITCH(4)
	BANK_SWITCH(5)
	BANK_SWITCH(6)
	BANK_SWITCH(7)

	PORT_START("DMA")
	PORT_DIPNAME(0x02, 0x00, "DMA override")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x02, DEF_STR(On))
	PORT_DIPNAME(0x01, 0x00, "Lock out during DMA")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x01, DEF_STR(On))
INPUT_PORTS_END

ioport_constructor s100_cromemco_16kz_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cromemco_16kz);
}


//**************************************************************************
//  64KZ
//**************************************************************************

class s100_cromemco_64kz_device : public device_t, public device_s100_card_interface
{
public:
	s100_cromemco_64kz_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_smemr_r(offs_t offset) override;
	virtual void s100_mwrt_w(offs_t offset, u8 data) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;
	virtual void s100_phlda_w(int state) override;

private:
	static constexpr unsigned BLOCKS = 2;   // A and B

	bool addressed(unsigned block, offs_t offset) const { return (offset >> 15) == BIT(m_control->read(), block); }
	bool selected(unsigned block, offs_t offset) const;
	int block_for(offs_t offset) const;

	required_ioport m_control;
	required_ioport_array<BLOCKS> m_banks;
	u8 m_ram[BLOCKS][0x8000];
	bool m_enabled[BLOCKS];
	int m_phlda;
};

s100_cromemco_64kz_device::s100_cromemco_64kz_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_CROMEMCO_64KZ, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_control(*this, "SW1")
	, m_banks(*this, "SW%u", 2U)
	, m_ram{ }
	, m_enabled{ }
	, m_phlda(0)
{
}

void s100_cromemco_64kz_device::device_start()
{
	save_item(NAME(m_ram));
	save_item(NAME(m_enabled));
	save_item(NAME(m_phlda));
}

void s100_cromemco_64kz_device::device_reset()
{
	// each block's RESET switch sets its own state, IN for enabled
	u8 const control = m_control->read();
	for (unsigned block = 0; block < BLOCKS; block++)
		m_enabled[block] = BIT(control, 2 + block);
}

bool s100_cromemco_64kz_device::selected(unsigned block, offs_t offset) const
{
	if (!addressed(block, offset))
		return false;

	u8 const control = m_control->read();
	if (m_phlda && BIT(control, 4 + block))
		return BIT(control, 6 + block);

	return m_enabled[block];
}

int s100_cromemco_64kz_device::block_for(offs_t offset) const
{
	// Both blocks can be switched to the same half of memory. Software is
	// expected to keep them in different banks; if both answer at once the
	// real card puts two drivers on the bus, so returning block A here is as
	// arbitrary as the hardware is.
	for (unsigned block = 0; block < BLOCKS; block++)
		if (selected(block, offset))
			return block;

	return -1;
}

u8 s100_cromemco_64kz_device::s100_smemr_r(offs_t offset)
{
	int const block = block_for(offset);
	return (block < 0) ? 0xff : m_ram[block][offset & 0x7fff];
}

void s100_cromemco_64kz_device::s100_mwrt_w(offs_t offset, u8 data)
{
	int const block = block_for(offset);
	if (block >= 0)
		m_ram[block][offset & 0x7fff] = data;
}

void s100_cromemco_64kz_device::s100_sout_w(offs_t offset, u8 data)
{
	if ((offset & 0xff) == 0x40)
		for (unsigned block = 0; block < BLOCKS; block++)
			m_enabled[block] = (data & m_banks[block]->read()) != 0;
}

void s100_cromemco_64kz_device::s100_phlda_w(int state)
{
	m_phlda = state;
}

#define BLOCK_BANK_SWITCHES(block) \
	PORT_START("SW" #block) \
	BANK_SWITCH(0) \
	BANK_SWITCH(1) \
	BANK_SWITCH(2) \
	BANK_SWITCH(3) \
	BANK_SWITCH(4) \
	BANK_SWITCH(5) \
	BANK_SWITCH(6) \
	BANK_SWITCH(7)

static INPUT_PORTS_START( cromemco_64kz )
	PORT_START("SW1")
	PORT_DIPNAME(0x01, 0x00, "Block A address")
	PORT_DIPSETTING(0x00, "0000-7FFF")
	PORT_DIPSETTING(0x01, "8000-FFFF")
	PORT_DIPNAME(0x02, 0x02, "Block B address")
	PORT_DIPSETTING(0x00, "0000-7FFF")
	PORT_DIPSETTING(0x02, "8000-FFFF")
	PORT_DIPNAME(0x04, 0x04, "Block A after reset")
	PORT_DIPSETTING(0x00, "Out")
	PORT_DIPSETTING(0x04, "In")
	PORT_DIPNAME(0x08, 0x00, "Block B after reset")
	PORT_DIPSETTING(0x00, "Out")
	PORT_DIPSETTING(0x08, "In")
	PORT_DIPNAME(0x10, 0x00, "Block A DMA override")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x10, DEF_STR(On))
	PORT_DIPNAME(0x20, 0x00, "Block B DMA override")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x20, DEF_STR(On))
	PORT_DIPNAME(0x40, 0x40, "Block A during DMA")
	PORT_DIPSETTING(0x00, "Out")
	PORT_DIPSETTING(0x40, "In")
	PORT_DIPNAME(0x80, 0x80, "Block B during DMA")
	PORT_DIPSETTING(0x00, "Out")
	PORT_DIPSETTING(0x80, "In")

	BLOCK_BANK_SWITCHES(2)
	BLOCK_BANK_SWITCHES(3)
INPUT_PORTS_END

ioport_constructor s100_cromemco_64kz_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cromemco_64kz);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_CROMEMCO_16KZ, device_s100_card_interface, s100_cromemco_16kz_device, "s100_cromemco_16kz", "Cromemco 16KZ 16K RAM")
DEFINE_DEVICE_TYPE_PRIVATE(S100_CROMEMCO_64KZ, device_s100_card_interface, s100_cromemco_64kz_device, "s100_cromemco_64kz", "Cromemco 64KZ 64K RAM")
