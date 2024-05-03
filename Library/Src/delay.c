#include "delay.h"

void System_Clock_Cfg(void)
{
	RCC->CR |= (1);   /* HSION */
	while(!(RCC->CR & (1<<1)));
	RCC->APB1ENR |= (1<<28);
	FLASH->ACR |= 0x12;   /* PRFTBE ON, latency 2 */
	while(!(FLASH->ACR & (1<<5)));
}

void Systick_Initialize(void)
{
	STK->CTRL = 0;    /*Disable Systick*/ 
	STK->LOAD = 60-1;		/*Set reload value*/   /* systick interval = 1 us  (error 3us) */
	STK->VAL = 0;		/*RESET system tick counter value */
	STK->CTRL |= (1<<2);	/* select processor clock source : AHB 8 MHZ*/
	STK->CTRL |= (1<<1);  /* enable systick interupt */
	STK->CTRL |= 1;       /* enable systick */
}

void SysTick_Handler(void)
{
	if(TimeDelay > 0) TimeDelay--;
}

void delay_us(uint32_t time)
{
	TimeDelay = time;
	while(TimeDelay != 0);
}

void delay_ms(uint32_t ms)
{
	while(ms--)
	{
		delay_us(1000);
	}
}
