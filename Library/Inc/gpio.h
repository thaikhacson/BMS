#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f1xx.h"

/*
 * This in configuration structure for GPIO pin
 */
typedef struct
{
	uint32_t GPIO_PinNumber; 		
	uint32_t GPIO_PinMode;				
	uint32_t	GPIO_IOType	;					
}GPIO_PinConfig_t;


/*
 * This in handle structure for GPIO pin
 */
typedef struct
{
	GPIO_Regdef_t* pGPIOx; 					/* pointer to hold the base address of GPIOx*/
	GPIO_PinConfig_t GPIO_PinConfig;		/*this hold GPIO pin configuration*/
}GPIO_Handle_t;

/*
 * GPIO pin value
 */
#define GPIO_PIN_0	0
#define GPIO_PIN_1	1
#define GPIO_PIN_2	2
#define GPIO_PIN_3	3
#define GPIO_PIN_4	4
#define GPIO_PIN_5	5
#define GPIO_PIN_6	6
#define GPIO_PIN_7	7
#define GPIO_PIN_8	8
#define GPIO_PIN_9	9
#define GPIO_PIN_10	10
#define GPIO_PIN_11	11
#define GPIO_PIN_12	12
#define GPIO_PIN_13	13
#define GPIO_PIN_14	14
#define GPIO_PIN_15	15





#define port_code(x)	((x == GPIOA)?0:\
											(x == GPIOB)?1:\
											(x == GPIOC)?2:\
											(x == GPIOD)?3:\
											(x == GPIOE)?4:0)
\

/*
 * GPIO pin mode
 */
#define INPUT 						0
#define OUTPUT_10MHZ			1
#define OUTPUT_2MHZ				2
#define OUTPUT_50MHZ			3
#define IT_RE							4
#define IT_FE							5

/*
 * GPIO output type
 */
#define PUSH_PULL 				0
#define OPEN_DRAIN 				1
#define AF_PUSH_PULL 			2
#define AF_OPEN_DRAIN 		3

/*
 * GPIO input type
 */
#define ANALOG		 				0
#define FLOATING 					1
#define PUPD 							2


#define GPIO_PIN RESET 	0
#define GPIO_PIN_SET	1

/******************************************************************************
 * APIs support by this driver
 ******************************************************************************/
/* GPIO clock control */
void GPIO_PeriClockControl(GPIO_Regdef_t* pGPIOx, uint8_t EnOrDi);

/* GPIO init and deinit */
void GPIO_SetState(GPIO_Regdef_t* pGPIOx, uint8_t pinNum, uint8_t pinMode, uint8_t IOType);
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_Regdef_t* pGPIOx);

/* GPIO read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t* pGPIOx, uint8_t pinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Regdef_t* pGPIOx, uint8_t PinNum, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_Regdef_t* pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_Regdef_t* pGPIOx, uint8_t PinNum);

/* IRQ config and handling */
void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNum);

#endif /*__GPIO_H*/