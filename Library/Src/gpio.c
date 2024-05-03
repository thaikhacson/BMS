#include "gpio.h"

/***************************************************************************************
 * @func				GPIO_PeriClockControl
 *
 * @brief				This function enable or disable clock for a given GPIO port
 *
 * @param[in]			Base address of GPIO port
 * @param[in]			Enable/Disable macros
 * @param[in]
 *
 * @return 				None
 *
 * @Note				Node
 * */
void GPIO_PeriClockControl(GPIO_Regdef_t* pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)  			{GPIOA_CLK_EN();}
		else if(pGPIOx == GPIOB)		{GPIOB_CLK_EN();}
		else if(pGPIOx == GPIOC)		{GPIOC_CLK_EN();}
		else if(pGPIOx == GPIOD)		{GPIOD_CLK_EN();}
		else if(pGPIOx == GPIOE)		{GPIOE_CLK_EN();}
	}
	else if(EnOrDi == DISABLE)
	{
		if(pGPIOx == GPIOA)  			{GPIOA_CLK_DI();}
		else if(pGPIOx == GPIOB)		{GPIOB_CLK_DI();}
		else if(pGPIOx == GPIOC)		{GPIOC_CLK_DI();}
		else if(pGPIOx == GPIOD)		{GPIOD_CLK_DI();}
		else if(pGPIOx == GPIOE)		{GPIOE_CLK_DI();}

	}
}



/***************************************************************************************
 * @func				GPIO_Init
 *
 * @brief				This function initialize GPIO pin of port (mode, speed, type,...)
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;
	uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8 ;
	uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
	
	/* Enable the peripheral clock */
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	
	/*1.Configure the mode of GPIO pin*/
	
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= OUTPUT_50MHZ)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4*temp2));
		pGPIOHandle->pGPIOx->CR[temp1] &= ~(0x3 << (4*temp2));
		pGPIOHandle->pGPIOx->CR[temp1] |= temp;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == IT_RE)
		{
		/* 1. Configure the RTSR - rising trigger selection register */
				EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the corresponding bit of falling trigger selection register*/
				EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == IT_FE)
		{
			/* 1. Configure the FTSR - falling trigger selection register */
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the corresponding bit of rising trigger selection register*/
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		/* 2. Configure the port selection in AFIO_EXTICRx */
		uint32_t Temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
		uint32_t Temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		uint8_t portcode = port_code(pGPIOHandle->pGPIOx);
		AFIO_CLK_EN();  /*enable clock for AFIO controller*/
		AFIO->EXTICR[Temp1] |= (portcode << (4*Temp2));
		/* 3. Enable interupt delivery by using EXTI_IMR */
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	/*2.Configure the I/O type*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_IOType << (2 + (4*temp2)));
	pGPIOHandle->pGPIOx->CR[temp1] &= ~(0x3 << (2 + (4*temp2)));
	pGPIOHandle->pGPIOx->CR[temp1] |= temp;

	/*3.Configure the ALT function*/
	
}



void GPIO_SetState(GPIO_Regdef_t* pGPIOx, uint8_t pinNum, uint8_t pinMode, uint8_t IOType)
{
	GPIO_Handle_t GPIO_pin;
	GPIO_pin.pGPIOx = pGPIOx;
	GPIO_pin.GPIO_PinConfig.GPIO_PinNumber = pinNum ;
	GPIO_pin.GPIO_PinConfig.GPIO_PinMode = pinMode ;
	GPIO_pin.GPIO_PinConfig.GPIO_IOType = IOType;
	GPIO_Init(&GPIO_pin);
}



/***************************************************************************************
 * @func				GPIO_DeInit
 *
 * @brief				This function reset GPIO pin of port (bring GPIO Pin to reset state)
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */
void GPIO_DeInit(GPIO_Regdef_t* pGPIOx)
{
	if(pGPIOx == (GPIO_Regdef_t*)GPIOA_BASEADDR) {GPIOA_REG_RESET();}
	if(pGPIOx == (GPIO_Regdef_t*)GPIOB_BASEADDR) {GPIOB_REG_RESET();}
	if(pGPIOx == (GPIO_Regdef_t*)GPIOC_BASEADDR) {GPIOC_REG_RESET();}
	if(pGPIOx == (GPIO_Regdef_t*)GPIOD_BASEADDR) {GPIOD_REG_RESET();}
	if(pGPIOx == (GPIO_Regdef_t*)GPIOE_BASEADDR) {GPIOE_REG_RESET();}
}

/***************************************************************************************
 * @func				GPIO_ReadFromInputPin
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t* pGPIOx, uint8_t pinNum)
{
	uint8_t value = 0;
	value = (uint8_t)(pGPIOx->IDR >> pinNum) & 1 ;
	return value;
}


/***************************************************************************************
 * @func
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t* pGPIOx)
{
	uint16_t value = 0;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/***************************************************************************************
 * @func
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */
void GPIO_WriteToOutputPin(GPIO_Regdef_t* pGPIOx, uint8_t PinNum, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNum);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNum);
	}
}

/***************************************************************************************
 * @func
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */
void GPIO_WriteToOutputPort(GPIO_Regdef_t* pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value ;
}

/***************************************************************************************
 * @func
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */
void GPIO_ToggleOutputPin(GPIO_Regdef_t* pGPIOx, uint8_t PinNum)
{
	pGPIOx->ODR ^= (1<<PinNum);
}

/***************************************************************************************
 * @func	GPIO_IRQConfig
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */
void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	uint8_t value1 = IRQNumber/32;
	uint8_t value2 = IRQNumber%32;
 	if(EnOrDi == ENABLE)
	{
 		NVIC->ISER[value1] |= (1<<value2);
	}
 	else
 	{
 		NVIC->ICER[value1] |= (1<<value2);
 	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t pr_reg_x = (IRQNumber / 4); 			// similar to div 4
	uint8_t pr_section = (IRQNumber % 4);			// similar to mod 4
	uint8_t shift_amount = (8 * pr_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	NVIC->IPR[pr_reg_x] |= (IRQPriority << (8*shift_amount));
}
/***************************************************************************************
 * @func 	GPIO_IRQHandling
 *
 * @brief
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @Note
 * */
void GPIO_IRQHandling(uint8_t PinNum)
{
	// clear the corresponding bit of pending register
//	if((EXTI->PR >> PinNum) & 0x1 )
//	{
		EXTI->PR |= (1<<PinNum);
//	}
}
