#include "spi.h"

/* PRIVATE, this is helper function, user application can not call these function */
static void spi_txe_interupt_handle(SPI_Handle_t* pSPIHandle);
static void spi_rxe_interupt_handle(SPI_Handle_t* pSPIHandle);
static void spi_ovr_err_interupt_handle(SPI_Handle_t* pSPIHandle);


void SPI_PeriClockControl(SPI_Regdef_t* pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)  					{SPI1_CLK_EN();}
		else if(pSPIx == SPI2)			{SPI2_CLK_EN();}
		else if(pSPIx == SPI3)			{SPI3_CLK_EN();}
	}
	else if(EnOrDi == DISABLE)
	{
		if(pSPIx == SPI1)  					{SPI1_CLK_DI();}
		else if(pSPIx == SPI2)			{SPI2_CLK_DI();}
		else if(pSPIx == SPI3)			{SPI3_CLK_DI();}
	}
}

void SPI_Init(SPI_Handle_t* pSPIHandle)
{
	
	/* Enable clock */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	
	uint32_t tempreg=0;
	/* 1.Configure the device mode */
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << 2;
	/* 2.Configure the bus config */
	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/*BIDI mode should be clear*/
		tempreg &= ~(1<<15);
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/*BIDI mode should be set*/
		tempreg |= (1<<15);
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_RXONLY)
	{
		/*BIDI mode should be clear*/
		tempreg &= ~(1<<15);
		/*RXONLY bit must be set*/
		tempreg |= (1<<10);
	}
	/* 3. Configure clock speed */
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << 3;
	
	/* 4.Configure the DFF */
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DFF << 11;
	
	/* 5.Configure the CPOL */
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPOL << 1;
	
	/* 6.Configure the CPHA */
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPHA << 0;
	
	/* 6.Configure the SSM */
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_SSM << 9;
	
	pSPIHandle->pSPIx->CR1 = tempreg;
}

uint8_t SPI_GetFlagStatus(SPI_Regdef_t* pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_SendData(SPI_Regdef_t* pSPIx, uint8_t* pTxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		/* Wait until TXE is set */
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		
		/* Check the DFF bit in CR1 */
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			/* 16 bits DFF */
			/* 1. Load data into DR */
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++; 
		}
		else 
		{
			/* 8 bits DFF */
			pSPIx->DR = *((uint8_t*)pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_Regdef_t* pSPIx, uint8_t* pRxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		/* Wait until RXNE is set */
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
		
		/* Check the DFF bit in CR1 */
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			/* 16 bits DFF */
			/* 1. Load data from DR to RxBuffer */
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++; 
		}
		else 
		{
			/* 8 bits DFF */
			*((uint8_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

void SPI_SSIConfig(SPI_Regdef_t* pSPIx, uint8_t EnOrDi)   /* Master : SSI must be HIGH , Slave : SSI must be LOW */
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_Regdef_t* pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


void SPI_PeripheralControl(SPI_Regdef_t* pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t pr_reg_x = (IRQNumber / 4); 			// similar to div 4
	uint8_t pr_section = (IRQNumber % 4);			// similar to mod 4
	uint8_t shift_amount = (8 * pr_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	NVIC->IPR[pr_reg_x] |= (IRQPriority << (8*shift_amount));
}

uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
		
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
		
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t* pHandle)
{
	uint8_t temp1, temp2;
	
	/* first, check for TXE */
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		spi_txe_interupt_handle(pHandle);
	}
	/* second, check for RXNE */
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//spi_rxe_interupt_handle();
	}
	
	/* last, check for ovr error */
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//spi_ovr_err_interupt_handle();
	}
}

static void spi_txe_interupt_handle(SPI_Handle_t* pSPIHandle)
{
/* Check the DFF bit in CR1 */
		if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			/* 16 bits DFF */
			/* 1. Load data into DR */
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}
		else 
		{
			/* 8 bits DFF */
			pSPIHandle->pSPIx->DR = *((uint8_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}
		if(!pSPIHandle->TxLen)
		{
			/* TxLen is zero, so close communication */
			/* Tx is over */
			/* prevent interupt from setting of TXE */
			pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
			pSPIHandle->pTxBuffer = ((void*)0);
			pSPIHandle->TxLen = 0;
			pSPIHandle->TxState = SPI_READY;
		}
}
static void spi_rxe_interupt_handle(SPI_Handle_t* pSPIHandle)
{
	/* Check the DFF bit in CR1 */
		if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			/* 16 bits DFF */
			/* 1. Load data into DR */
			*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
			(uint16_t*)pSPIHandle->pRxBuffer--;
		}
		else 
		{
			/* 8 bits DFF */
			*((uint8_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
		}
		if(!pSPIHandle->RxLen)
		{
			/* TxLen is zero, so close communication */
			/* Tx is over */
			/* prevent interupt from setting of TXE */
			pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
			pSPIHandle->pRxBuffer = ((void*)0);
			pSPIHandle->RxLen = 0;
			pSPIHandle->RxState = SPI_READY;
		}
}
static void  spi_ovr_err_interupt_handle(SPI_Handle_t* pSPIHandle);
