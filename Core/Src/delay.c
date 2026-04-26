#include "delay.h"

#include "main.h"

static uint8_t delay_ready = 0U;

void Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    delay_ready = 1U;
}

void Delay_Us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (delay_ready == 0U) {
        Delay_Init();
    }

    start = DWT->CYCCNT;
    ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks) {
    }
}

void Delay_Ms(uint32_t ms)
{
    HAL_Delay(ms);
}
