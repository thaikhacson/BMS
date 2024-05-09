#include "delay.h"

// For store tick counts in us
static volatile uint32_t usTicks;

// For storing the total elapsed time in us since the program started
static volatile uint64_t usTimeElapsed = 0;

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

void SysTick_Handler()
{
	if (usTicks != 0)
	{
		usTicks--;
	}
	usTimeElapsed++;  // Increment the elapsed time counter
}

void delay_us(uint32_t us)
{
	// Reload us value
	usTicks = us;
	// Wait until usTick reaches zero
	while (usTicks);
}

void delay_ms(uint32_t ms)
{
	// Wait until ms reach zero
	while (ms--)
	{
		// Delay 1ms
		delay_us(1000);
	}
}

// Function to get the total elapsed time in microseconds since the program started
uint64_t micros()
{
	return usTimeElapsed;
}

// Optionally, you can create a function to get the time in milliseconds
uint64_t millis()
{
	return usTimeElapsed / 1000;  // Convert microseconds to milliseconds
}
