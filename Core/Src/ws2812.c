/**
  ******************************************************************************
  * @file    ws2812.c
  * @brief   Single WS2812 indicator driven by SPI2 blocking transmit.
  *
 *          This board uses a WS2812C-2020-V6 (LCSC C55109522). At 10 Mbit/s,
 *          each LED bit uses a fixed 12-SPI-bit cell: logical 0 is 3 high +
 *          9 low bits (0.3/0.9us), and logical 1 is 6 high + 6 low bits
 *          (0.6/0.6us). SPI remains in 8-bit frame mode; 16-bit writes to DR
 *          pack two frames into the FIFO so the 20MHz CPU can keep it fed.
 *          Data is followed by >280us reset low.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"
#include "spi.h"

/* Private defines -----------------------------------------------------------*/
/* Production default brightness. A test build overrides this at compile time
 * (WS2812_BRIGHTNESS_PERCENT=100) so all colors are clearly visible. */
#ifndef WS2812_BRIGHTNESS_PERCENT
#define WS2812_BRIGHTNESS_PERCENT  20U
#endif

#define WS2812_BITS_PER_LED         24U
#define WS2812_SPI_BITS_PER_BIT     12U
#define WS2812_CODE_0_HIGH_BITS      3U
#define WS2812_CODE_0_LOW_BITS       9U
#define WS2812_CODE_1_HIGH_BITS      6U
#define WS2812_CODE_1_LOW_BITS       6U
#define WS2812_PREAMBLE_BYTES         4U  /* low level while SPI TXFIFO settles */
#define WS2812_DATA_BYTES           ((WS2812_BITS_PER_LED * WS2812_SPI_BITS_PER_BIT) / 8U) /* 36 */
#define WS2812_RESET_BYTES          400U  /* 320us low at 10 Mbit/s */
#define WS2812_TX_BYTES             (WS2812_PREAMBLE_BYTES + WS2812_DATA_BYTES + WS2812_RESET_BYTES)

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[WS2812_TX_BYTES];

/* Private functions ---------------------------------------------------------*/
static uint8_t ws2812_scale(uint8_t c)
{
    return (uint8_t)(((uint32_t)c * WS2812_BRIGHTNESS_PERCENT) / 100U);
}

static void ws2812_append_level(uint8_t *buf, uint32_t *bit_pos,
                                uint32_t count, uint8_t high)
{
    while (count > 0U)
    {
        if (high != 0U)
        {
            uint32_t idx = *bit_pos >> 3U;
            uint32_t off = 7U - (*bit_pos & 7U);
            buf[idx] |= (uint8_t)(1U << off);
        }
        (*bit_pos)++;
        count--;
    }
}

/* Encode one byte (8 WS2812 bits, MSB first) into the SPI bit stream. */
static void ws2812_encode_byte(uint8_t *buf, uint32_t *bit_pos, uint8_t data)
{
    int32_t b;

    for (b = 7; b >= 0; b--)
    {
        if (((data >> (uint32_t)b) & 1U) != 0U)
        {
            ws2812_append_level(buf, bit_pos, WS2812_CODE_1_HIGH_BITS, 1U);
            ws2812_append_level(buf, bit_pos, WS2812_CODE_1_LOW_BITS, 0U);
        }
        else
        {
            ws2812_append_level(buf, bit_pos, WS2812_CODE_0_HIGH_BITS, 1U);
            ws2812_append_level(buf, bit_pos, WS2812_CODE_0_LOW_BITS, 0U);
        }
    }
}

/* With DS=8, a half-word write to SPI_DR packs two 8-bit frames into TXFIFO.
 * The low byte goes first on STM32G0, so this preserves byte-stream order and
 * halves the CPU refill rate compared with HAL_SPI_Transmit() byte writes. */
static void ws2812_spi_write(const uint8_t *data, uint16_t size)
{
    uint16_t i;

    if ((hspi2.Instance->CR1 & SPI_CR1_SPE) == 0U)
    {
        __HAL_SPI_ENABLE(&hspi2);
    }

    for (i = 0U; i < size; i += 2U)
    {
        uint16_t packed = (uint16_t)data[i]
                        | (uint16_t)((uint16_t)data[i + 1U] << 8U);

        while ((hspi2.Instance->SR & SPI_SR_TXE) == 0U)
        {
        }
        *((__IO uint16_t *)&hspi2.Instance->DR) = packed;
    }

    while ((hspi2.Instance->SR & SPI_SR_TXE) == 0U)
    {
    }
    while ((hspi2.Instance->SR & SPI_SR_BSY) != 0U)
    {
    }

    __HAL_SPI_CLEAR_OVRFLAG(&hspi2);
}

/* Exported functions --------------------------------------------------------*/
void ws2812_init(void)
{
    /* Keep the indicator unpowered until the first frame is sent. */
    HAL_GPIO_WritePin(WS2812_EN_GPIO_Port, WS2812_EN_Pin, GPIO_PIN_RESET);
}

void ws2812_send(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t bit_pos = WS2812_PREAMBLE_BYTES * 8U;
    uint32_t primask;
    uint32_t i;
    uint8_t  grb[3];

    /* The fitted indicator uses the standard WS2812 GRB wire order. */
    grb[0] = ws2812_scale(g);
    grb[1] = ws2812_scale(r);
    grb[2] = ws2812_scale(b);

    /* Clear the whole frame; the reset tail is the all-zero region. */
    for (i = 0U; i < WS2812_TX_BYTES; i++)
    {
        tx_buffer[i] = 0U;
    }

    /* Encode GRB after an all-low preamble. It lets SPI/FIFO reach steady
     * state before the first rising edge, so the first WS2812 bit has the same
     * pulse width as all following bits. */
    for (i = 0U; i < 3U; i++)
    {
        ws2812_encode_byte(tx_buffer, &bit_pos, grb[i]);
    }

    /* At 10 Mbit/s an interrupt between SPI FIFO refills can stretch the
     * current MOSI level beyond the WS2812 timing window. Keep only the short
     * 24-bit LED data section contiguous; restore interrupts before sending
     * the reset-low section. Preserve the caller's original PRIMASK state. */
    primask = __get_PRIMASK();
    __disable_irq();
    ws2812_spi_write(tx_buffer,
                     WS2812_PREAMBLE_BYTES + WS2812_DATA_BYTES);
    if (primask == 0U)
    {
        __enable_irq();
    }

    /* A separate all-zero transfer provides 320us reset time. Any delay before
     * or during this transfer only extends the required low level. */
    ws2812_spi_write(&tx_buffer[WS2812_PREAMBLE_BYTES + WS2812_DATA_BYTES],
                     WS2812_RESET_BYTES);
}

void ws2812_enable(void)
{
    HAL_GPIO_WritePin(WS2812_EN_GPIO_Port, WS2812_EN_Pin, GPIO_PIN_SET);

    /* Keep DIN low while the switched LED supply and its power-on reset settle. */
    HAL_Delay(1U);
}

void ws2812_disable(void)
{
    HAL_GPIO_WritePin(WS2812_EN_GPIO_Port, WS2812_EN_Pin, GPIO_PIN_RESET);
}

void ws2812_off(void)
{
    /* Send the off frame and complete the latch before cutting power. */
    ws2812_send(0U, 0U, 0U);
    ws2812_disable();
}
