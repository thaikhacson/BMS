#ifndef __SPI_H
#define __SPI_H 

#include "stm32f1xx.h"

/*
 * This in configuration structure for SPI pin
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_PinConfig_t;


/*
 * This in handle structure for SPI pin
 */
typedef struct
{
	SPI_Regdef_t* pSPIx; 					/* pointer to hold the base address of SPIx*/
	SPI_PinConfig_t SPI_PinConfig;		/*this hold SPI pin configuration*/
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;


/*
 *SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 *BUS_CONFIG
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2	
#define SPI_BUS_CONFIG_RXONLY			3

/*
 *SPI_CLKSPEED
 */
#define SPI_CLK_SPEED_DIV2				0
#define SPI_CLK_SPEED_DIV4				1
#define SPI_CLK_SPEED_DIV8				2
#define SPI_CLK_SPEED_DIV16				3
#define SPI_CLK_SPEED_DIV32				4
#define SPI_CLK_SPEED_DIV64				5
#define SPI_CLK_SPEED_DIV128			6
#define SPI_CLK_SPEED_DIV256			7

/*
 *SPI_DFF
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS			1

/*
 *SPI_CPOL
 */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 *SPI_CPHA
 */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/*
 *SPI_SSM
 */
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0
 
 
 
/* Flag position */
#define SPI_TXE_FLAG (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG (1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1<<SPI_SR_BSY)


/* Possible SPI application state */
#define SPI_READY						0
#define SPI_BUSY_IN_RX			1
#define SPI_BUSY_IN_TX			2



/* GPIO clock control */
void SPI_PeriClockControl(SPI_Regdef_t* pSPIx, uint8_t EnOrDi);

/* GPIO init and deinit */
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_Regdef_t* pSPIx);

/* Data Send and Reveive */
void SPI_SendData(SPI_Regdef_t* pSPIx, uint8_t* pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_Regdef_t* pSPIx, uint8_t* pRxBuffer, uint32_t Len);



uint8_t SPI_GetFlagStatus(SPI_Regdef_t* pSPIx, uint32_t FlagName);
/* IRQ Configure and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pHandle);
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pRxBuffer, uint32_t Len);



void SPI_SSIConfig(SPI_Regdef_t* pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_Regdef_t* pSPIx, uint8_t EnOrDi);
void SPI_PeripheralControl(SPI_Regdef_t* pSPIx, uint8_t EnOrDi);

#endif /*__SPI_H*/