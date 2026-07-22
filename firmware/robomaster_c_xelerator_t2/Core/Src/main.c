/*
 * main.c — Xelerator T2 C Board firmware entry point.
 *
 * Based on RoboMaster C Board official example 20.standard_robot,
 * stripped to only CAN + USB CDC + GPIO for DM-S2325 motor control.
 *
 * Official source: reference/vendor/robomaster-c-official/20.standard_robot
 * Commit: 59d12b1a
 */

#include "main.h"
#include "can.h"
#include "adc.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "dma.h"
#include "usb_device.h"
#include <string.h>

/* Private includes */
#include "bsp_can.h"

/* Xelerator T2 motor control */
#include "pi_c_protocol.h"
#include "motor_safety.h"
#include "dm2325_can.h"
#include "cboard_diag.h"
#include "cboard_imu.h"

/* External functions from main_loop.c */
extern void cboard_main_loop(void);
extern void cboard_usb_rx_callback(const uint8_t *data, uint16_t len);
extern void cboard_can_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len);

/* HAL tick override for main_loop.c */
uint32_t cboard_hal_get_tick(void) {
    return HAL_GetTick();
}

/* USB CDC TX wrapper for main_loop.c */
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define CDC_TX_QUEUE_DEPTH 8U

static uint8_t cdc_tx_queue[CDC_TX_QUEUE_DEPTH][PI_C_MAX_FRAME_SIZE];
static uint16_t cdc_tx_lengths[CDC_TX_QUEUE_DEPTH];
static uint8_t cdc_tx_head = 0U;
static uint8_t cdc_tx_tail = 0U;
static uint8_t cdc_tx_in_flight = 0U;
static volatile uint8_t cdc_tx_completed = 0U;

void cboard_usb_cdc_tx_complete_isr(void) {
    cdc_tx_completed = 1U;
    cboard_diag_event(CBOARD_DIAG_CDC_TX_COMPLETE, 0, 0, 0);
}

void cboard_usb_cdc_tx(const uint8_t *data, uint16_t len) {
    uint8_t next;
    if (data == NULL || len == 0U || len > PI_C_MAX_FRAME_SIZE) return;

    next = (uint8_t)((cdc_tx_head + 1U) % CDC_TX_QUEUE_DEPTH);
    if (next == cdc_tx_tail) {
        g_cboard_diag.cdc_tx_fail_count++;
        cboard_diag_event(CBOARD_DIAG_TX_QUEUE_FULL, 0, len, 0);
        return;
    }

    memcpy(cdc_tx_queue[cdc_tx_head], data, len);
    cdc_tx_lengths[cdc_tx_head] = len;
    cdc_tx_head = next;
    g_cboard_diag.cdc_tx_enqueue_count++;
    g_cboard_diag.cdc_tx_queue_depth = (cdc_tx_head + CDC_TX_QUEUE_DEPTH - cdc_tx_tail) % CDC_TX_QUEUE_DEPTH;
    cboard_diag_observe_max(&g_cboard_diag.cdc_tx_queue_high_water, g_cboard_diag.cdc_tx_queue_depth);
}

void cboard_usb_cdc_tx_poll(void) {
    if (hUsbDeviceFS.pClassData == NULL) return;

    if (cdc_tx_in_flight) {
        if (!cdc_tx_completed) {
            g_cboard_diag.cdc_tx_busy_count++;
            return;
        }
        cdc_tx_tail = (uint8_t)((cdc_tx_tail + 1U) % CDC_TX_QUEUE_DEPTH);
        cdc_tx_in_flight = 0U;
        cdc_tx_completed = 0U;
        g_cboard_diag.cdc_tx_complete_count++;
        cboard_diag_event(CBOARD_DIAG_STATUS_TX_ENQUEUE, 0xFFFFU, 0, 0);
    }

    if (cdc_tx_tail == cdc_tx_head) return;
    if (CDC_Transmit_FS(cdc_tx_queue[cdc_tx_tail], cdc_tx_lengths[cdc_tx_tail]) == USBD_OK) {
        cdc_tx_in_flight = 1U;
        cboard_diag_event(CBOARD_DIAG_CDC_TX_START, cdc_tx_lengths[cdc_tx_tail], 0, 0);
    } else {
        g_cboard_diag.cdc_tx_busy_count++;
    }
}

/* Private function prototypes */
void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    cboard_diag_init(168000000U);

    MX_GPIO_Init();
    MX_DMA_Init();

    /* Initialize USB CDC FIRST so the device enumerates even if CAN fails */
    MX_USB_DEVICE_Init();

    /* CAN init after USB — non-fatal if CAN transceiver unpowered */
    MX_CAN1_Init();
    can_filter_init();

    /* IMU is independent from motor control. Initialization failure is
     * reported through IMU status and never enables or commands a motor. */
    MX_ADC1_Temperature_Init();
    MX_SPI1_Init();
    MX_TIM10_Init();
    cboard_imu_init();

    /* Enter motor control main loop (never returns) */
    cboard_main_loop();

    while (1) {}
}

/**
  * @brief System Clock Configuration
  * Copied verbatim from official 20.standard_robot.
  * HSE = 12 MHz, PLLM=6, PLLN=168, PLLP=2, PLLQ=7
  * -> SYSCLK = 168 MHz, USB 48 MHz from PLLQ.
  */
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
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1) {}
}
#endif
