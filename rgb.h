/*
  rgb.h - typedefs, API structure and helper functions for RGB lights and LED strips

  Part of grblHAL

  Copyright (c) 2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t B      :1,
                G      :1,
                R      :1,
                W      :1,
                unused :4;
    };
} rgb_color_mask_t;

typedef union {
    uint32_t value;
    struct {
        uint8_t B; //!< Blue
        uint8_t G; //!< Green
        uint8_t R; //!< Red
        uint8_t W; //!< White
    };
} rgb_color_t;

typedef union {
    uint8_t mask;
    struct {
        uint8_t is_blocking :1,
                is_strip    :1,
                unassigned  :6;
    };
} rgb_properties_t;

/*! \brief Pointer to function for setting RGB (LED) output.
\param color a \a rgb_color_t union.
*/
typedef void (*rgb_set_color_ptr)(uint16_t device, rgb_color_t color);

/*! \brief Pointer to function for setting RGB (LED) output, with mask for which LEDs to change.
\param color a \a rgb_color_t union.
\param mask a \a rgb_color_mask_t union.
*/
typedef void (*rgb_set_color_masked_ptr)(uint16_t device, rgb_color_t color, rgb_color_mask_t mask);

/*! \brief Pointer to function for setting RGB (LED) intensity.
\param intensity in the range 0 - 255.
\returns previuous intensity.
*/
typedef uint8_t (*rgb_set_intensity_ptr)(uint8_t intensity);

/*! \brief Pointer to function for outputting RGB (LED) data to Neopixel strip.
*/
typedef void (*rgb_write_ptr)(void);

typedef struct {
    rgb_set_color_ptr out;                  //!< Optional handler for setting device (LED) color.
    rgb_set_color_masked_ptr out_masked;    //!< Optional handler for setting device (LED) color, with mask for which LEDs to change.
    rgb_write_ptr write;                    //!< Optional handler for outputting data to Neopixel strip.
    rgb_set_intensity_ptr set_intensity;    //!< Optional handler for setting intensity, range 0 - 255.
    rgb_color_t cap;                        //!< Driver capability, color value: 0 - not available, 1 - on off, > 1 - intensity range 0 - n.
    rgb_properties_t flags;                 //!< Driver property flags.
    uint16_t num_devices;                   //!< Number of devices (LEDs) available.
} rgb_ptr_t;

// helper structure and functions, not used by the core

typedef struct {
    uint16_t num_leds;
    uint16_t num_bytes;
    uint8_t *leds;
    uint8_t intensity;
} neopixel_cfg_t;

static inline bool rgb_is_neopixels (rgb_ptr_t *device)
{
    return device->out != NULL && device->cap.R > 126 && device->cap.G > 126 && device->cap.B > 126;
}

static inline bool rgb_is_onoff (rgb_ptr_t *device)
{
    return device->out != NULL && device->cap.R == 1 && device->cap.G == 1 && device->cap.B == 1;
}

// Intensity conversions

static inline rgb_color_t rgb_set_intensity (rgb_color_t color, uint8_t intensity)
{
    color.R = (uint8_t)(((color.R + 1) * intensity) >> 8);
    color.G = (uint8_t)(((color.G + 1) * intensity) >> 8);
    color.B = (uint8_t)(((color.B + 1) * intensity) >> 8);

    return color;
}

static inline rgb_color_t rgb_reset_intensity (rgb_color_t color, uint8_t intensity)
{
    color.R = (uint8_t)((color.R << 8) / (intensity + 1));
    color.G = (uint8_t)((color.G << 8) / (intensity + 1));
    color.B = (uint8_t)((color.B << 8) / (intensity + 1));

    return color;
}

// RGB to/from 3 bytes per pixel packed format

static inline void  rgb_3bpp_pack (uint8_t *led, rgb_color_t color, rgb_color_mask_t mask, uint8_t intensity)
{
    uint32_t R = 0, G = 0, B = 0;
    uint8_t bitmask = 0b10000000;

    color = rgb_set_intensity(color, intensity);

    do {
        R <<= 3;
        R |= color.R & bitmask ? 0b110 : 0b100;
        G <<= 3;
        G |= color.G & bitmask ? 0b110 : 0b100;
        B <<= 3;
        B |= color.B & bitmask ? 0b110 : 0b100;
    } while(bitmask >>= 1);

    if(mask.G) {
        *led++ = (uint8_t)(G >> 16);
        *led++ = (uint8_t)(G >> 8);
        *led++ = (uint8_t)G;
    } else
        led += 3;

    if(mask.R) {
        *led++ = (uint8_t)(R >> 16);
        *led++ = (uint8_t)(R >> 8);
        *led++ = (uint8_t)R;
    } else
        led += 3;

    if(mask.B) {
        *led++ = (uint8_t)(B >> 16);
        *led++ = (uint8_t)(B >> 8);
        *led   = (uint8_t)B;
    }
}

static inline rgb_color_t rgb_3bpp_unpack (uint8_t *led, uint8_t intensity)
{
    rgb_color_t color = {0};

    if(intensity) {

        uint32_t R = 0, G = 0, B = 0;
        uint8_t bitmask = 0b00000001;

        G = *led++ << 16;
        G |= *led++ << 8;
        G |= *led++;
        R = *led++ << 16;
        R |= *led++ << 8;
        R |= *led++;
        B = *led++ << 16;
        B |= *led++ << 8;
        B |= *led;

        do {
            if((R & 0b110) == 0b110)
               color.R |= bitmask;
            R >>= 3;
            if((G & 0b110) == 0b110)
               color.G |= bitmask;
            G >>= 3;
            if((B & 0b110) == 0b110)
               color.B |= bitmask;
            B >>= 3;
        } while(bitmask <<= 1);

        color = rgb_reset_intensity(color, intensity);
    }

    return color;
}

// RGB to/from 1 byte per pixel packed format

static inline void rgb_1bpp_assign (uint8_t *led, rgb_color_t color, rgb_color_mask_t mask)
{
    if(mask.G)
        *led++ = color.G;
    else
        led++;

    if(mask.R)
        *led++ = color.R;
    else
        led++;

    if(mask.B)
        *led = color.B;
}

static inline void  rgb_1bpp_pack (uint8_t *led, rgb_color_t color, rgb_color_mask_t mask, uint8_t intensity)
{
    color = rgb_set_intensity(color, intensity);
    rgb_1bpp_assign(led, color, mask);
}

static inline rgb_color_t rgb_1bpp_unpack (uint8_t *led, uint8_t intensity)
{
    rgb_color_t color = {0};

    if(intensity) {

        color.G = *led++;
        color.R = *led++; 
        color.B = *led; 

        color = rgb_reset_intensity(color, intensity);
    }

    return color;
}

//
