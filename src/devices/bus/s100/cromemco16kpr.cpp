// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/**********************************************************************

    Cromemco 16KPR EPROM board (1977)

    Sixteen sockets for 1K 2708 EPROMs, giving 16K of firmware on one
    card. Two switches set A15 and A14, so the block sits at 0000, 4000,
    8000 or C000, and the card carries the same bank select and DMA
    override circuits as the Cromemco RAM boards.

    The Z-2 manual's minimum system wants the Z-80 Monitor at E000. This
    card reaches that with the block at C000 and the monitor in the socket
    nine tenths of the way up, at offset 2000. The manual itself recommends
    the 8K Bytesaver, which is an 8K card and lands E000 in its first
    socket, but it lists the 16K PROM card as the alternative and the
    address the CPU sees is the same either way.

    Sockets
    An empty socket drives nothing, so a read of that kilobyte comes back
    as the idle bus, FF. That is what an unpopulated part of the region
    holds here.

    Not emulated
    - Wait states. The card senses the 2/4 MHz line on bus pin 98 and
      inserts an occasional wait during stack operations at 4 MHz. Nothing
      here needs the delay and s100_bus_device does not carry the line.
    - MWRT. The card is read only and ignores writes, which is what the
      real one does.

    References
    - 16KPR manual 023-0010. Block select in section 2.1, bank select in
      2.2, DMA override in 2.3, the board disable and bank select jumpers
      in 2.4 and 2.5, block disable in 2.7.
    - Z-2/Z-2D instruction manual, 1978, for the minimum system and the
      E000 monitor address.

**********************************************************************/

#include "emu.h"
#include "cromemco16kpr.h"


namespace {

// one switch per bank, shared by every Cromemco card with bank select
#define BANK_SWITCH(bit) \
	PORT_DIPNAME(1 << (bit), 1 << (bit), "Bank " #bit) \
	PORT_DIPSETTING(0, DEF_STR(Off)) \
	PORT_DIPSETTING(1 << (bit), DEF_STR(On))


class s100_cromemco_16kpr_device : public device_t, public device_s100_card_interface
{
public:
	s100_cromemco_16kpr_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

	// device_s100_card_interface implementation
	virtual u8 s100_smemr_r(offs_t offset) override;
	virtual void s100_sout_w(offs_t offset, u8 data) override;
	virtual void s100_phlda_w(int state) override;

private:
	// the two block switches compare against A14 and A15
	bool addressed(offs_t offset) const;
	bool selected(offs_t offset) const;

	required_region_ptr<u8> m_rom;
	required_ioport m_address;
	required_ioport m_banks;
	required_ioport m_dma;
	required_ioport m_jumpers;
	bool m_enabled;
	int m_phlda;
};

s100_cromemco_16kpr_device::s100_cromemco_16kpr_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, S100_CROMEMCO_16KPR, tag, owner, clock)
	, device_s100_card_interface(mconfig, *this)
	, m_rom(*this, "roms")
	, m_address(*this, "ADDRESS")
	, m_banks(*this, "BANKS")
	, m_dma(*this, "DMA")
	, m_jumpers(*this, "JUMPERS")
	, m_enabled(false)
	, m_phlda(0)
{
}

void s100_cromemco_16kpr_device::device_start()
{
	save_item(NAME(m_enabled));
	save_item(NAME(m_phlda));
}

void s100_cromemco_16kpr_device::device_reset()
{
	// With the board disable jumper fitted the card comes up off whatever
	// the bank switches say, and the bank select port has to turn it on.
	// Without it the bank 0 switch decides, because bank 0 is the bank that
	// is active at power up.
	m_enabled = !BIT(m_jumpers->read(), 0) && BIT(m_banks->read(), 0);
}

bool s100_cromemco_16kpr_device::addressed(offs_t offset) const
{
	if ((offset >> 14) != m_address->read())
		return false;

	// Block disable takes out either half of the card, which is how the
	// user tailors how much EPROM sits at one address.
	u8 const jumpers = m_jumpers->read();

	if (BIT(offset, 13))
		return !BIT(jumpers, 2);    // upper 8K

	return !BIT(jumpers, 1);        // lower 8K
}

bool s100_cromemco_16kpr_device::selected(offs_t offset) const
{
	if (!addressed(offset))
		return false;

	// With the override enabled the bank latch is ignored during DMA and the
	// lockout switch decides instead.
	u8 const dma = m_dma->read();
	if (m_phlda && BIT(dma, 1))
		return !BIT(dma, 0);

	return m_enabled;
}

u8 s100_cromemco_16kpr_device::s100_smemr_r(offs_t offset)
{
	return selected(offset) ? m_rom[offset & 0x3fff] : 0xff;
}

void s100_cromemco_16kpr_device::s100_sout_w(offs_t offset, u8 data)
{
	// the port decodes A0-A7 only, so mask off whatever the CPU left in the
	// high half of the I/O address
	if ((offset & 0xff) != 0x40)
		return;

	// the bank select enable jumper cuts the port off altogether
	if (BIT(m_jumpers->read(), 3))
		return;

	m_enabled = (data & m_banks->read()) != 0;
}

void s100_cromemco_16kpr_device::s100_phlda_w(int state)
{
	m_phlda = state;
}


static INPUT_PORTS_START( cromemco_16kpr )
	PORT_START("ADDRESS")
	PORT_DIPNAME(0x03, 0x03, "Address")
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

	PORT_START("JUMPERS")
	PORT_CONFNAME(0x01, 0x00, "Board disable")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x01, DEF_STR(On))
	PORT_CONFNAME(0x02, 0x00, "Lower 8K disable")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x02, DEF_STR(On))
	PORT_CONFNAME(0x04, 0x00, "Upper 8K disable")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x04, DEF_STR(On))
	PORT_CONFNAME(0x08, 0x00, "Bank select disable")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x08, DEF_STR(On))
INPUT_PORTS_END

ioport_constructor s100_cromemco_16kpr_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cromemco_16kpr);
}


// The card ships with sixteen empty sockets. What goes in them here is the
// Z-80 Monitor the Z-2 manual's minimum system calls for, in the socket
// that puts it at E000 with the block switched to C000.
ROM_START( cromemco_16kpr )
	ROM_REGION( 0x4000, "roms", ROMREGION_ERASEFF )
	ROM_LOAD( "zm_monitor_1.4_2708.bin", 0x2000, 0x0400, CRC(62f50531) SHA1(3071e2ab7fc6b2ca889e4fb5cf7cc9ee8fbe53d3) )
ROM_END

const tiny_rom_entry *s100_cromemco_16kpr_device::device_rom_region() const
{
	return ROM_NAME( cromemco_16kpr );
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(S100_CROMEMCO_16KPR, device_s100_card_interface, s100_cromemco_16kpr_device, "cromemco_16kpr", "Cromemco 16KPR EPROM Board")
