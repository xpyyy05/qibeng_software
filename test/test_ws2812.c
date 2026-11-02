/**
  ******************************************************************************
  * @file    test_ws2812.c
  * @brief   Unit test: WS2812 color cycling.
  *
  *          On power-up the indicator shows red. Each debounced PA0 button
  *          press advances red -> green -> blue -> white -> red. The test exercises the
  *          production ws2812 module (production brightness scaling, GRB wire order,
  *          SPI2 blocking transmit, WS2812_EN power control), so it validates
  *          the real LED driver path. Non-blocking: timing comes from
  *          HAL_GetTick() in the main loop.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "test.h"
#include "ws2812.h"

/* Private defines -----------------------------------------------------------*/
#define TEST_KEY_DEBOUNCE_MS  30U

/* Private types -------------------------------------------------------------*/
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} test_color_t;

/* Private variables ---------------------------------------------------------*/
static const test_color_t test_colors[] = {
    {255U,   0U,   0U},   /* red   */
    {  0U, 255U,   0U},   /* green */
    {  0U,   0U, 255U},   /* blue  */
    {255U, 255U, 255U}    /* white */
};
#define TEST_COLOR_COUNT (sizeof(test_colors) / sizeof(test_colors[0]))

static uint32_t test_color_index;
static uint32_t test_key_last_ms;
static volatile uint32_t test_key_event_pending;

/* Exported functions --------------------------------------------------------*/
void test_ws2812_init(void)
{
    test_color_index       = 0U;
    test_key_last_ms       = HAL_GetTick() - TEST_KEY_DEBOUNCE_MS;
    test_key_event_pending = 0U;

    ws2812_init();   /* WS2812_EN low (LED unpowered) until the first frame */

    /* Power the LED. Send a blank frame first so the WS2812 gets a clean
     * power-on reset and the first real color is not the very first frame
     * after VDD ramp-up (which the LED may fail to latch). */
    ws2812_enable();
    ws2812_send(0U, 0U, 0U);

    /* Show the first color immediately. */
    ws2812_send(test_colors[0].r, test_colors[0].g, test_colors[0].b);
}

void test_ws2812_process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t primask;

    if (test_key_event_pending == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    test_key_event_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    /* Reject additional rising edges caused by contact bounce. The EXTI event
     * remains valid even if PA0's high pulse is much shorter than 30ms. */
    if ((uint32_t)(now - test_key_last_ms) < TEST_KEY_DEBOUNCE_MS)
    {
        return;
    }

    test_key_last_ms = now;
    test_color_index = (test_color_index + 1U) % (uint32_t)TEST_COLOR_COUNT;
    ws2812_send(test_colors[test_color_index].r,
                test_colors[test_color_index].g,
                test_colors[test_color_index].b);
}

#if defined(TEST_WS2812)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_EXTI_Pin)
    {
        test_key_event_pending = 1U;
    }
}
#endif

/* ------------------------------------------------------------------ */
/* Red-only diagnostic build (TEST_WS2812_RED_ONLY).                   */
/*                                                                     */
/* On power-up light ONLY red and continuously re-send the red frame.  */
/* This produces a dense PA4 waveform for oscilloscope triggering.     */
/* Full brightness (100%) is selected by the TestRed build target.     */
/* ------------------------------------------------------------------ */
#if defined(TEST_WS2812_RED_ONLY)

void test_ws2812_red_init(void)
{
    ws2812_init();
    ws2812_enable();

    /* Power-on reset blank frame, then pure red at full brightness. */
    ws2812_send(0U, 0U, 0U);
    ws2812_send(255U, 0U, 0U);
}

void test_ws2812_red_process(void)
{
    ws2812_send(255U, 0U, 0U);
}

#endif /* TEST_WS2812_RED_ONLY */
