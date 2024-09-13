#ifndef LED_WS2812_H__
#define LED_WS2812_H__

#include <stdint.h>

#define NR_OF_PIXELS 20

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led_ws2812b_pixel_t;


static led_ws2812b_pixel_t led_green = {0, 41, 0};
static led_ws2812b_pixel_t led_red = {41,0, 0};
static led_ws2812b_pixel_t led_blue = {0, 0, 41};
static led_ws2812b_pixel_t led_black = {0, 0, 0};

void led_ws2812b_init(uint8_t pin);
void led_ws2812b_set_pixel_rgb(uint8_t pixel_nr, uint8_t red, uint8_t green, uint8_t blue);
void led_ws2812b_set_pixel(uint8_t pixel_nr, led_ws2812b_pixel_t *color);
void led_ws2812b_show(void);
void led_ws2812b_reset(void);
void rgb_led_glow(uint8_t r,uint8_t g,uint8_t b);
void set_color_ws2812b(uint8_t color,uint8_t brightness);

void timeout_pattern();

#endif //LED_WS2812
