/*
 * stm32f4xx_it.c — Interrupt Service Routines (bare-metal, no FreeRTOS).
 * Based on official 20.standard_robot, stripped of cmsis_os dependencies.
 */

#include "main.h"
#include "stm32f4xx_it.h"
#include "cboard_diag.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void DebugMon_Handler(void) {}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void CAN1_RX0_IRQHandler(void) {
    uint32_t started = cboard_diag_time_us();
    HAL_CAN_IRQHandler(&hcan1);
    g_cboard_diag.can1_rx_irq_count++;
    cboard_diag_observe_max(&g_cboard_diag.can1_rx_irq_duration_max_us,
                             cboard_diag_time_us() - started);
}

void CAN2_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan2);
}

void OTG_FS_IRQHandler(void) {
    uint32_t started = cboard_diag_time_us();
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
    g_cboard_diag.usb_irq_count++;
    cboard_diag_observe_max(&g_cboard_diag.usb_irq_duration_max_us,
                             cboard_diag_time_us() - started);
}
