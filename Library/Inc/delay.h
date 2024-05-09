#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f1xx.h"

void System_Clock_Cfg(void);
void Systick_Initialize(void);
void delay_us(uint32_t time);
void SysTick_Handler(void);
void delay_ms(uint32_t ms);

#endif /*__SYSTICK_H*/