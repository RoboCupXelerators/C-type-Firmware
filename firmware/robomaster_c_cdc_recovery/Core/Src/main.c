/*
 * Minimal CDC Recovery Firmware for RoboMaster C Board
 * STM32F407IGH6, bare-metal, no CAN, no motor, no DMA, no EXTI.
 *
 * Boot flow: SystemInit (FPU+VTOR) -> main -> direct-reg LED -> HAL_Init
 *            -> SystemClock_Config -> MX_USB_DEVICE_Init -> loop
 *
 * LED diagnostics use direct register access (no HAL, no FPU, no clock dependency):
 *   Red  on  = entered main
 *   Red+Green = HAL_Init done
 *   Green only = SystemClock_Config done
 *   Green+Blue = MX_USB_DEVICE_Init done
 *   Red blink (200ms) = Error_Handler
 *
 * Official C board: LED_R=PH12, LED_G=PH11, LED_B=PH10, active HIGH.
 */

#include "main.h"
#include "usb_device.h"
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);

/* The shared CDC class calls this hook after an IN transfer. The recovery
 * image does not queue application data, so the callback is intentionally a
 * no-op while keeping the shared USB middleware linkable. */
void cboard_usb_cdc_tx_complete_isr(void)
{
}

/* Direct register LED helpers — no HAL, no FPU, works on default HSI clock. */
static inline void led_init_direct(void)
{
    /* Enable GPIOH clock (RCC AHB1ENR bit 7) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    (void)RCC->AHB1ENR;  /* read-back to ensure clock enable takes effect */

    /* Configure PH10, PH11, PH12 as push-pull output, no pull, low speed.
     * MODER: 01 = general purpose output (bits 20-21, 22-23, 24-25)
     * OTYPER: 0 = push-pull (bits 10, 11, 12)
     * PUPDR: 00 = no pull (bits 20-21, 22-23, 24-25)
     */
    uint32_t moder = GPIOH->MODER;
    moder &= ~((3U << (10U * 2U)) | (3U << (11U * 2U)) | (3U << (12U * 2U)));
    moder |=  ((1U << (10U * 2U)) | (1U << (11U * 2U)) | (1U << (12U * 2U)));
    GPIOH->MODER = moder;

    GPIOH->OTYPER &= ~((1U << 10U) | (1U << 11U) | (1U << 12U));

    uint32_t pupdr = GPIOH->PUPDR;
    pupdr &= ~((3U << (10U * 2U)) | (3U << (11U * 2U)) | (3U << (12U * 2U)));
    GPIOH->PUPDR = pupdr;
}

static inline void led_on(uint32_t pin)
{
    GPIOH->BSRR = (1U << pin);       /* set = high = LED on */
}

static inline void led_off(uint32_t pin)
{
    GPIOH->BSRR = (1U << (pin + 16)); /* reset = low = LED off */
}

int main(void)
{
    /* Phase 0: Direct-register LED init, before HAL, before clock config.
     * SystemInit has already run (called from Reset_Handler), FPU is enabled.
     * This code uses only integer operations — no FPU, no SysTick, no HAL. */
    led_init_direct();
    led_on(12);   /* Red on = entered main */

    HAL_Init();

    /* Phase 2: Red+Green = HAL_Init done */
    led_on(11);   /* Green on */

    SystemClock_Config();

    /* Phase 3: Green only = SystemClock_Config done */
    led_off(12);  /* Red off */

    /* GPIOA clock for USB PA11/PA12 (AF config done in HAL_PCD_MspInit) */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    MX_USB_DEVICE_Init();

    /* Phase 4: Green+Blue = USB init returned */
    led_on(10);   /* Blue on */

    while (1)
    {
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV30;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    /* Red blink 200ms = fault */
    led_off(11);  /* Green off */
    led_off(10);  /* Blue off */
    while (1)
    {
        GPIOH->BSRR = (1U << 12);        /* Red on */
        /* Simple delay without HAL_Delay (which needs SysTick) */
        for (volatile uint32_t i = 0; i < 1000000; i++) {}
        GPIOH->BSRR = (1U << (12 + 16)); /* Red off */
        for (volatile uint32_t i = 0; i < 1000000; i++) {}
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1) {}
}
#endif
