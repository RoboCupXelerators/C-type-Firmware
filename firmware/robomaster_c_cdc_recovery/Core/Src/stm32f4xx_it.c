/*
 * Minimal ISR for CDC recovery firmware.
 * Only fault handlers + SysTick + OTG_FS. No EXTI, no DMA, no CAN.
 */

#include "main.h"
#include "stm32f4xx_hal.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void DebugMon_Handler(void) {}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void OTG_FS_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
