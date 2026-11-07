#include "current_sens.h"

#include "stm32f4xx_hal.h"

static uint8_t _is_init = 0;
static ADC_HandleTypeDef adc_config;

/* The STM32F446 maps PC4 to ADC1_IN14 and PA4 to ADC1_IN4. */
static void current_sense_gpio_init(void)
{
    GPIO_InitTypeDef gpio_config = {0};
    gpio_config.Mode = GPIO_MODE_ANALOG;
    gpio_config.Pull = GPIO_NOPULL;
    gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio_config.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOC, &gpio_config);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_config.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &gpio_config);
}

void current_sense_init(void)
{
    if (_is_init) {
        return;
    }

    _is_init = 1;
    __HAL_RCC_ADC1_CLK_ENABLE();
    current_sense_gpio_init();

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

static float current_sense_read(uint32_t channel)
{
    ADC_ChannelConfTypeDef adc_channelconf = {0};
    adc_channelconf.Channel = channel;
    adc_channelconf.Rank = 1;
    adc_channelconf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    (void)HAL_ADC_ConfigChannel(&adc_config, &adc_channelconf);

    (void)HAL_ADC_Start(&adc_config);
    (void)HAL_ADC_PollForConversion(&adc_config, 0xFF);
    uint32_t adc_val = HAL_ADC_GetValue(&adc_config);
    (void)HAL_ADC_Stop(&adc_config);


    // We set adc to have 12bit range (4095 vals), and we have to scale it by VDD, which
    // for our board is 3.3V
    //float voltage = ((float)adc_val/4095.0f) * 3.3f;

    // after getting pin voltage, we need to get current sense voltage
    // which we get by reverse engineering voltage div w/ 20k and 10k
    // and from there convert to current using 400mv/A at 2.5v bias
    // Gets this equation

    /* Existing sensor calibration: 2.5 V bias and 400 mV/A sensitivity. */
    return 0.006043956f * (float)adc_val - 6.25f;
}

float left_current_read(void)
{
    return current_sense_read(ADC_CHANNEL_14); /* PC4 */
}

float right_current_read(void)
{
    return current_sense_read(ADC_CHANNEL_4); /* PA4 */
}
