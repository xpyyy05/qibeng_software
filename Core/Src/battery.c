/**
  ******************************************************************************
  * @file    battery.c
  * @brief   Battery voltage sensing via ADC1 + circular DMA.
  *
  *          ADC1 scans [ADC_CHANNEL_1 (battery / PA1), ADC_CHANNEL_VREFINT] in
  *          continuous mode. DMA1_Channel1 is circular and keeps filling a
  *          32-half-word buffer. The buffer is two 16-half-word frames; each
  *          frame holds 8x (BAT, VREF) pairs. The half-transfer callback sums
  *          the stable first frame, the transfer-complete callback sums the
  *          stable second frame; averaging and voltage math run in the main
  *          loop so the ISR never touches division, floats or SPI.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "battery.h"
#include "adc.h"

/* Private defines -----------------------------------------------------------*/
#define ADC_CHANNEL_COUNT  2U
#define ADC_AVERAGE_COUNT  8U
#define ADC_FRAME_LENGTH   (ADC_CHANNEL_COUNT * ADC_AVERAGE_COUNT)   /* 16 half-words */
#define ADC_DMA_LENGTH     (ADC_FRAME_LENGTH * 2U)                  /* 32 half-words */

#define UV_ENTER_MV        2800U   /* Below this -> undervoltage candidate */
#define UV_EXIT_MV         3000U   /* At/above this -> recovery candidate */
#define UV_HOLD_MS         500U    /* Candidate must persist this long */

/* Private variables ---------------------------------------------------------*/
/* Circular DMA buffer. Frame order within each half: BAT0,VREF0,...,BAT7,VREF7. */
static volatile uint16_t adc_dma_buffer[ADC_DMA_LENGTH];

/* Shadow sums written by the DMA callbacks, consumed by battery_process(). */
static volatile uint32_t g_bat_sum;
static volatile uint32_t g_vref_sum;
static volatile uint32_t g_frame_ready;

/* Validated results consumed by the application. */
static uint32_t g_battery_mv;
static uint32_t g_vdda_mv;
static bool     g_data_valid;

/* Undervoltage latch state machine. */
static bool     g_uv_latched;
static bool     g_uv_candidate;
static uint32_t g_uv_candidate_ms;

/* Private functions ---------------------------------------------------------*/
/* Accumulate one stable 16-half-word frame into shadow sums. */
static void battery_accumulate_half(const volatile uint16_t *half)
{
    uint32_t b = 0U;
    uint32_t v = 0U;
    uint32_t i;

    for (i = 0U; i < ADC_FRAME_LENGTH; i += ADC_CHANNEL_COUNT)
    {
        b += half[i];        /* even index: ADC_CHANNEL_1 (battery) */
        v += half[i + 1U];   /* odd  index: ADC_CHANNEL_VREFINT   */
    }

    g_bat_sum  = b;
    g_vref_sum = v;
    g_frame_ready = 1U;
}

/* HAL ADC DMA callbacks -----------------------------------------------------*/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    battery_accumulate_half(&adc_dma_buffer[0]);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    battery_accumulate_half(&adc_dma_buffer[ADC_FRAME_LENGTH]);
}

/* Exported functions --------------------------------------------------------*/
void battery_init(void)
{
    g_battery_mv = 0U;
    g_vdda_mv    = 0U;
    g_data_valid = false;
    g_uv_latched    = false;
    g_uv_candidate  = false;
    g_uv_candidate_ms = 0U;
    g_frame_ready = 0U;
    g_bat_sum  = 0U;
    g_vref_sum = 0U;

    /* Enable the VREFINT measurement path (internal channel). */
    LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_VREFINT);

    /* Datasheet VREFINT start-up/stabilization is ~12us; 1ms is safely above. */
    HAL_Delay(1U);

    /* STM32G0 requires an ADC hardware self-calibration before first conversion. */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start continuous ADC + circular DMA once; it runs until deinit. */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_DMA_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }
}

void battery_deinit(void)
{
    /* Stop continuous conversion and circular DMA (shutdown path). */
    (void)HAL_ADC_Stop_DMA(&hadc1);

    g_frame_ready = 0U;
    g_data_valid  = false;
}

void battery_resume(void)
{
    /* Re-arm the continuous ADC+DMA after an aborted Shutdown entry. Unlike
     * battery_init(), the undervoltage latch and the last valid readings are
     * preserved so a shutdown abort cannot defeat the undervoltage inhibit. */
    LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_VREFINT);
    HAL_Delay(1U);

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_DMA_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }
}

void battery_process(void)
{
    uint32_t bat_sum;
    uint32_t vref_sum;
    uint32_t avg_bat;
    uint32_t avg_vref;
    uint32_t vdda_mv;
    uint32_t adc_pin_mv;
    uint32_t battery_mv;
    uint32_t now = HAL_GetTick();

    if (g_frame_ready == 0U)
    {
        return;
    }

    /* Short critical section: consume the shadow sums and clear the flag so a
     * concurrent ISR write cannot tear the two 32-bit values. */
    __disable_irq();
    bat_sum  = g_bat_sum;
    vref_sum = g_vref_sum;
    g_frame_ready = 0U;
    __enable_irq();

    avg_bat  = bat_sum  / ADC_AVERAGE_COUNT;
    avg_vref = vref_sum / ADC_AVERAGE_COUNT;

    /* Guard against a degenerate/empty VREFINT reading (avoids divide-by-zero). */
    if (avg_vref == 0U)
    {
        g_data_valid = false;
        return;
    }

    /* Real-time VDDA from VREFINT + factory calibration (12-bit resolution). */
    vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(avg_vref, ADC_RESOLUTION_12B);

    /* Battery pin voltage, then apply the hardware 1/2 divider (x2). */
    adc_pin_mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vdda_mv, avg_bat, ADC_RESOLUTION_12B);
    battery_mv = adc_pin_mv * 2U;

    g_vdda_mv    = vdda_mv;
    g_battery_mv = battery_mv;
    g_data_valid = true;

    /* Undervoltage latch: enter <2800mV / exit >=3000mV, each held 500ms. */
    if (!g_uv_latched)
    {
        if (battery_mv < UV_ENTER_MV)
        {
            if (!g_uv_candidate)
            {
                g_uv_candidate = true;
                g_uv_candidate_ms = now;
            }
            else if ((uint32_t)(now - g_uv_candidate_ms) >= UV_HOLD_MS)
            {
                g_uv_latched = true;
                /* Re-arm the recovery (exit) debounce: the shared candidate
                 * flag/timestamp otherwise still hold the entry timestamp, so
                 * the first >=3000mV sample would un-latch immediately and skip
                 * the required 500ms hold. */
                g_uv_candidate = false;
            }
        }
        else
        {
            g_uv_candidate = false;
        }
    }
    else
    {
        if (battery_mv >= UV_EXIT_MV)
        {
            if (!g_uv_candidate)
            {
                g_uv_candidate = true;
                g_uv_candidate_ms = now;
            }
            else if ((uint32_t)(now - g_uv_candidate_ms) >= UV_HOLD_MS)
            {
                g_uv_latched = false;
                g_uv_candidate = false;
            }
        }
        else
        {
            g_uv_candidate = false;
        }
    }
}

uint32_t battery_get_mv(void)
{
    return g_battery_mv;
}

uint32_t battery_get_vdda_mv(void)
{
    return g_vdda_mv;
}

bool battery_is_undervoltage(void)
{
    return g_uv_latched;
}

bool battery_data_valid(void)
{
    return g_data_valid;
}
