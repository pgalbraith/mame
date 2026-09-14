// license:BSD-3-Clause
// copyright-holders:Paul Galbraith
/*********************************************************************

    formats/mits_cas.cpp

    MITS Altair 88-ACR cassette tapes

    A .tap file holds the bytes as the 88-ACR's UART sends them, at 300
    baud with a start bit, eight data bits and a stop bit. Loading one
    makes the tones the 88-ACR's modulator records: its presettable counter
    divides the 2 MHz bus clock by 104 for a 1 and by 135 for a 0, taking
    each new count as it runs out, and a divide by 8 after it gives a square
    wave of 2404 Hz or 1852 Hz with no break in phase.

    A second of steady tone goes before the data and after it.

*********************************************************************/

#include "mits_cas.h"

#include "coretmpl.h" // BIT


namespace {

constexpr double BUS_CLOCK = 2'000'000.0;
constexpr double BIT_TIME = 1.0 / 300.0;
constexpr int32_t LEVEL = 0x40000000;

cassette_image::error mits_tap_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44'100;
	return cassette_image::error::SUCCESS;
}

cassette_image::error mits_tap_load(cassette_image *cassette)
{
	cassette_image::error err = cassette_image::error::SUCCESS;
	double bit_end = 0.0;       // where the bit being sent ends
	double count_end = 0.0;     // where the counter next runs out
	double edge = 0.0;          // where the tone last changed level
	unsigned divider = 0;
	int32_t level = LEVEL;

	auto const send_bit =
			[&] (unsigned value)
			{
				bit_end += BIT_TIME;
				while ((cassette_image::error::SUCCESS == err) && (count_end < bit_end))
				{
					count_end += (value ? 104.0 : 135.0) / BUS_CLOCK;
					divider = (divider + 1) & 7;
					if (!(divider & 3))
					{
						err = cassette->put_sample(0, edge, count_end - edge, level);
						edge = count_end;
						level = -level;
					}
				}
			};

	for (int i = 0; i < 300; i++)
		send_bit(1);

	uint64_t const image_size = cassette->image_size();
	for (uint64_t image_pos = 0; image_pos < image_size; image_pos++)
	{
		uint8_t const data = cassette->image_read_byte(image_pos);

		send_bit(0);
		for (int bit = 0; bit < 8; bit++)
			send_bit(util::BIT(data, bit));
		send_bit(1);
	}

	for (int i = 0; i < 300; i++)
		send_bit(1);

	return err;
}

const cassette_image::Format mits_tap_format =
{
	"tap",
	mits_tap_identify,
	mits_tap_load,
	nullptr
};

} // anonymous namespace


CASSETTE_FORMATLIST_START( mits_cassette_formats )
	CASSETTE_FORMAT( mits_tap_format )
CASSETTE_FORMATLIST_END
