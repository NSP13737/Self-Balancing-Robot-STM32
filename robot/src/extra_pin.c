#include "extra_pin.h"

#include "stm32f4xx_hal.h"

static uint8_t is_initialized = 0;
static ADC_HandleTypeDef adc_config;

void extra_pin_init(void)
{
    if (is_initialized) {
        return;
    }

    is_initialized = 1;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef gpio_config = {0};
    gpio_config.Pin = GPIO_PIN_3;
    gpio_config.Mode = GPIO_MODE_ANALOG;
    gpio_config.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_config);

    adc_config.Instance = ADC1;
    adc_config.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    adc_config.Init.Resolution = ADC_RESOLUTION_12B;
    adc_config.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_config.Init.ScanConvMode = DISABLE;
    adc_config.Init.ContinuousConvMode = DISABLE;
    adc_config.Init.NbrOfConversion = 1;
    adc_config.Init.DiscontinuousConvMode = DISABLE;
    adc_config.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc_config.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    adc_config.Init.DMAContinuousRequests = DISABLE;
    adc_config.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    (void)HAL_ADC_Init(&adc_config);
}

float extra_pin_read(void)
{
    if (!is_initialized) {
        extra_pin_init();
    }

    ADC_ChannelConfTypeDef channel_config = {0};
    channel_config.Channel = ADC_CHANNEL_13; /* PC3 / ADC1_IN13 */
    channel_config.Rank = 1;
    channel_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    (void)HAL_ADC_ConfigChannel(&adc_config, &channel_config);

    (void)HAL_ADC_Start(&adc_config);
    (void)HAL_ADC_PollForConversion(&adc_config, 0xFF);
    uint32_t adc_val = HAL_ADC_GetValue(&adc_config);
    (void)HAL_ADC_Stop(&adc_config);

    float voltage = ((float)adc_val / 4095.0f) * 3.3f;
    return voltage;
}
