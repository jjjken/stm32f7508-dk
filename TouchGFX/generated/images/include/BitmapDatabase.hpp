// Generated by imageconverter. Please, do not edit!

#ifndef TOUCHGFX_BITMAPDATABASE_HPP
#define TOUCHGFX_BITMAPDATABASE_HPP

#include <touchgfx/hal/Types.hpp>
#include <touchgfx/Bitmap.hpp>

const uint16_t BITMAP_BACKGROUND_ID = 0;
const uint16_t BITMAP_BLUE_BUTTONS_ROUND_EDGE_ICON_BUTTON_PRESSED_ID = 1;
const uint16_t BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_ID = 2;
const uint16_t BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_PRESSED_ID = 3;
const uint16_t BITMAP_CLOCKS_BACKGROUNDS_CLOCK_CLASSIC_BACKGROUND_ID = 4;
const uint16_t BITMAP_CLOCKS_HANDS_CLOCK_CLASSIC_HOUR_HAND_ID = 5;
const uint16_t BITMAP_CLOCKS_HANDS_CLOCK_CLASSIC_MINUTE_HAND_ID = 6;
const uint16_t BITMAP_CLOCKS_HANDS_CLOCK_CLASSIC_SECOND_HAND_ID = 7;

namespace BitmapDatabase
{
const touchgfx::Bitmap::BitmapData* getInstance();
uint16_t getInstanceSize();
} // namespace BitmapDatabase

#endif // TOUCHGFX_BITMAPDATABASE_HPP
