#ifndef __STM32F1XX_H
#define __STM32F1XX_H

#include <stdint.h>

#define NULL  (void*)0

/*
 * Base address of Flash and Sram memmories
 */

#define FLASH_BASEADDR 		0x08000000
#define SRAM_BASEADDR 		0x20000000


/*
 * Base address of different bus domain
 */

#define PERIPH_BASE				0x40000000
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000
#define AHBPERIPH_BASE		0x40020000


/*
 * Base address of peripherals which are hanging on AHB bus
 */

#define CRC_BASEADDR		(AHBPERIPH_BASE + 0xB000)
#define RCC_BASEADDR		(AHBPERIPH_BASE + 0x1000)


/*
 * Base address of peripherals which are hanging on APB1 bus
 */

#define TIM2_BASEADDR							(APB1PERIPH_BASE)
#define TIM3_BASEADDR							(APB1PERIPH_BASE + 0x400)
#define SPI2_BASEADDR							(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR							(APB1PERIPH_BASE + 0x3C00)


/*
 * Base address of peripherals which are hanging on APB2 bus
 */

#define GPIOA_BASEADDR		(APB2PERIPH_BASE + 0x800)
#define GPIOB_BASEADDR		(APB2PERIPH_BASE + 0xC00)
#define GPIOC_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR		(APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR		(APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR		(APB2PERIPH_BASE + 0x2000)
#define AFIO_BASEADDR							 APB2PERIPH_BASE
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x400)
#define ADC1_BASEADDR			(APB2PERIPH_BASE + 0x2400)
#define ADC2_BASEADDR			(APB2PERIPH_BASE + 0x2800)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)


/*
 * Base address of peripherals which are hanging on AHB bus
 */
 
#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000)
#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x0008)
#define DMA1_Channel2_BASE    (AHBPERIPH_BASE + 0x001C)
#define DMA1_Channel3_BASE    (AHBPERIPH_BASE + 0x0030)
#define DMA1_Channel4_BASE    (AHBPERIPH_BASE + 0x0044)
#define DMA1_Channel5_BASE    (AHBPERIPH_BASE + 0x0058)
#define DMA1_Channel6_BASE    (AHBPERIPH_BASE + 0x006C)
#define DMA1_Channel7_BASE    (AHBPERIPH_BASE + 0x0080)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x0400)
#define DMA2_Channel1_BASE    (AHBPERIPH_BASE + 0x0408)
#define DMA2_Channel2_BASE    (AHBPERIPH_BASE + 0x041C)
#define DMA2_Channel3_BASE    (AHBPERIPH_BASE + 0x0430)
#define DMA2_Channel4_BASE    (AHBPERIPH_BASE + 0x0444)
#define DMA2_Channel5_BASE    (AHBPERIPH_BASE + 0x0458)
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)
#define FLASH_BASE						(AHBPERIPH_BASE + 0x2000) /*!< Flash registers base address */

/******************************Peripheral Register Definition Structures*************/


/** 
  * @brief General Purpose I/O
  */

typedef struct
{
	volatile unsigned int CR[2];			/* Configure register low */
	volatile unsigned int IDR;			
	volatile unsigned int ODR;
	volatile unsigned int BSRR;
	volatile unsigned int BRR;
	volatile unsigned int LCKR;
} GPIO_Regdef_t;


/** 
  * @brief Alternate Function I/O
  */

typedef struct
{
	volatile unsigned int EVCR;
	volatile unsigned int MAPR;
	volatile unsigned int EXTICR[4];
	volatile unsigned int MAPR2;
} AFIO_Regdef_t;


/** 
  * @brief Reset and Clock Control
  */

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBSTR;
	volatile uint32_t CFGR2;
} RCC_Regdef_t;


/** 
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_Regdef_t;


/** 
  * @brief DMA Controller
  */

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_Regdef_t;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_Regdef_t;


/** 
  * @brief NVIC controller
  */

typedef struct
{
	volatile uint32_t ISER[3];
	volatile uint32_t dummy1[29];
	volatile uint32_t ICER[3];
	volatile uint32_t dummy2[29];
	volatile uint32_t ISPR[3];
	volatile uint32_t dummy3[29];
	volatile uint32_t ICPR[3];
	volatile uint32_t dummy4[29];
	volatile uint32_t IABR[3];
	volatile uint32_t dummy5[61];
	volatile uint32_t IPR[21];
	volatile uint32_t dummy6[695];
	volatile uint32_t STIR;
} NVIC_Regdef_t;


/** 
  * @brief NVIC Systick
  */

typedef struct
{
	volatile uint32_t CTRL;
	volatile uint32_t LOAD;
	volatile uint32_t VAL;
	volatile uint32_t CALIB;
} STK_Regdef_t;


/** 
  * @brief Analog to Digital Converter  
  */

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_Regdef_t;


/** 
  * @brief Serial Peripheral Interface
  */

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_Regdef_t;

/** 
  * @brief FLASH Registers
  */

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
#ifdef STM32F10X_XL
  uint32_t RESERVED1[8]; 
  volatile uint32_t KEYR2;
  uint32_t RESERVED2;   
  volatile uint32_t SR2;
  volatile uint32_t CR2;
  volatile uint32_t AR2; 
#endif /* STM32F10X_XL */  
} FLASH_TypeDef;

typedef struct
{
  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  uint32_t RESERVED0[5U];
  uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

typedef enum 
{
	RESET = 0, 
	SET = !RESET
} FlagStatus, ITStatus;


#define NVIC_BASEADDR				(0xE000E100)
#define NVIC								((NVIC_Regdef_t*)NVIC_BASEADDR)
#define SCS_BASE            (0xE000E000UL) 														/*!< System Control Space Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */

#define SYSTICK_BASEADDR		(0xE000E010)
#define STK									((STK_Regdef_t*)SYSTICK_BASEADDR)

#define NVIC_ISER0					((volatile uint32_t*)0xE000E100)

#define NO_PR_BITS_IMPLEMENTED	4			/* Number of pr bits actually specific to MCU, ST case is 4*/

#define RCC				((RCC_Regdef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_Regdef_t*)EXTI_BASEADDR)
#define AFIO			((AFIO_Regdef_t*)AFIO_BASEADDR)

/* SCB System Handler Control and State Register Definitions */
#define SCB_SHCSR_USGFAULTENA_Pos          18U                                            /*!< SCB SHCSR: USGFAULTENA Position */
#define SCB_SHCSR_USGFAULTENA_Msk          (1UL << SCB_SHCSR_USGFAULTENA_Pos)             /*!< SCB SHCSR: USGFAULTENA Mask */

#define SCB_SHCSR_BUSFAULTENA_Pos          17U                                            /*!< SCB SHCSR: BUSFAULTENA Position */
#define SCB_SHCSR_BUSFAULTENA_Msk          (1UL << SCB_SHCSR_BUSFAULTENA_Pos)             /*!< SCB SHCSR: BUSFAULTENA Mask */

#define SCB_SHCSR_MEMFAULTENA_Pos          16U                                            /*!< SCB SHCSR: MEMFAULTENA Position */
#define SCB_SHCSR_MEMFAULTENA_Msk          (1UL << SCB_SHCSR_MEMFAULTENA_Pos)             /*!< SCB SHCSR: MEMFAULTENA Mask */

#define SCB_SHCSR_SVCALLPENDED_Pos         15U                                            /*!< SCB SHCSR: SVCALLPENDED Position */
#define SCB_SHCSR_SVCALLPENDED_Msk         (1UL << SCB_SHCSR_SVCALLPENDED_Pos)            /*!< SCB SHCSR: SVCALLPENDED Mask */

#define SCB_SHCSR_BUSFAULTPENDED_Pos       14U                                            /*!< SCB SHCSR: BUSFAULTPENDED Position */
#define SCB_SHCSR_BUSFAULTPENDED_Msk       (1UL << SCB_SHCSR_BUSFAULTPENDED_Pos)          /*!< SCB SHCSR: BUSFAULTPENDED Mask */

#define SCB_SHCSR_MEMFAULTPENDED_Pos       13U                                            /*!< SCB SHCSR: MEMFAULTPENDED Position */
#define SCB_SHCSR_MEMFAULTPENDED_Msk       (1UL << SCB_SHCSR_MEMFAULTPENDED_Pos)          /*!< SCB SHCSR: MEMFAULTPENDED Mask */

#define SCB_SHCSR_USGFAULTPENDED_Pos       12U                                            /*!< SCB SHCSR: USGFAULTPENDED Position */
#define SCB_SHCSR_USGFAULTPENDED_Msk       (1UL << SCB_SHCSR_USGFAULTPENDED_Pos)          /*!< SCB SHCSR: USGFAULTPENDED Mask */

#define SCB_SHCSR_SYSTICKACT_Pos           11U                                            /*!< SCB SHCSR: SYSTICKACT Position */
#define SCB_SHCSR_SYSTICKACT_Msk           (1UL << SCB_SHCSR_SYSTICKACT_Pos)              /*!< SCB SHCSR: SYSTICKACT Mask */

#define SCB_SHCSR_PENDSVACT_Pos            10U                                            /*!< SCB SHCSR: PENDSVACT Position */
#define SCB_SHCSR_PENDSVACT_Msk            (1UL << SCB_SHCSR_PENDSVACT_Pos)               /*!< SCB SHCSR: PENDSVACT Mask */

#define SCB_SHCSR_MONITORACT_Pos            8U                                            /*!< SCB SHCSR: MONITORACT Position */
#define SCB_SHCSR_MONITORACT_Msk           (1UL << SCB_SHCSR_MONITORACT_Pos)              /*!< SCB SHCSR: MONITORACT Mask */

#define SCB_SHCSR_SVCALLACT_Pos             7U                                            /*!< SCB SHCSR: SVCALLACT Position */
#define SCB_SHCSR_SVCALLACT_Msk            (1UL << SCB_SHCSR_SVCALLACT_Pos)               /*!< SCB SHCSR: SVCALLACT Mask */

#define SCB_SHCSR_USGFAULTACT_Pos           3U                                            /*!< SCB SHCSR: USGFAULTACT Position */
#define SCB_SHCSR_USGFAULTACT_Msk          (1UL << SCB_SHCSR_USGFAULTACT_Pos)             /*!< SCB SHCSR: USGFAULTACT Mask */

#define SCB_SHCSR_BUSFAULTACT_Pos           1U                                            /*!< SCB SHCSR: BUSFAULTACT Position */
#define SCB_SHCSR_BUSFAULTACT_Msk          (1UL << SCB_SHCSR_BUSFAULTACT_Pos)             /*!< SCB SHCSR: BUSFAULTACT Mask */

#define SCB_SHCSR_MEMFAULTACT_Pos           0U                                            /*!< SCB SHCSR: MEMFAULTACT Position */
#define SCB_SHCSR_MEMFAULTACT_Msk          (1UL /*<< SCB_SHCSR_MEMFAULTACT_Pos*/)         /*!< SCB SHCSR: MEMFAULTACT Mask */


/*
 * Clock enable for GPIOx peripherals
 */
#define GPIOA_CLK_EN()  (RCC->APB2ENR |= (1<<2))
#define GPIOB_CLK_EN()  (RCC->APB2ENR |= (1<<3))
#define GPIOC_CLK_EN()  (RCC->APB2ENR |= (1<<4))
#define GPIOD_CLK_EN()  (RCC->APB2ENR |= (1<<5))
#define GPIOE_CLK_EN()  (RCC->APB2ENR |= (1<<6))


/*
 * Clock disable for GPIOx peripherals
 */
#define GPIOA_CLK_DI()  (RCC->APB2ENR &=~ (1<<2))
#define GPIOB_CLK_DI()  (RCC->APB2ENR &=~ (1<<3))
#define GPIOC_CLK_DI()  (RCC->APB2ENR &=~ (1<<4))
#define GPIOD_CLK_DI()  (RCC->APB2ENR &=~ (1<<5))
#define GPIOE_CLK_DI()  (RCC->APB2ENR &=~ (1<<6))


/* Clock enable for SYSCFG controller*/
#define AFIO_CLK_EN() (RCC->APB2ENR |= (1))

/* Clock disable for SYSCFG controller*/
#define AFIO_CLK_DI() (RCC->APB2ENR &= ~(1))

/* clock enable for SPIx */

#define SPI1_CLK_EN()	(RCC->APB2ENR |= (1<<12))
#define SPI2_CLK_EN()	(RCC->APB1ENR |= (1<<14))
#define SPI3_CLK_EN()	(RCC->APB1ENR |= (1<<15))

#define SPI1_CLK_DI()	(RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLK_DI()	(RCC->APB1ENR &= ~(1<<14))
#define SPI3_CLK_DI()	(RCC->APB1ENR &= ~(1<<15))


/* clock enable for DMAx */

#define DMA1_CLK_EN() (RCC->AHBENR |= (1<<0))
#define DMA2_CLK_EN() (RCC->AHBENR |= (1<<1))

#define DMA1_CLK_DI() (RCC->AHBENR &= ~(1<<0))
#define DMA2_CLK_DI() (RCC->AHBENR &= ~(1<<1))


/*
 * Macros to reset peripheral
 */

#define GPIOA_REG_RESET()	do{RCC->APB2RSTR |= (1<<2);   RCC->APB2RSTR &= ~(1<<2);}while(0)  /*do...while...condition zero - technique in C language*/
#define GPIOB_REG_RESET()	do{RCC->APB2RSTR |= (1<<3);   RCC->APB2RSTR &= ~(1<<3);}while(0)
#define GPIOC_REG_RESET()	do{RCC->APB2RSTR |= (1<<4);   RCC->APB2RSTR &= ~(1<<4);}while(0)
#define GPIOD_REG_RESET()	do{RCC->APB2RSTR |= (1<<5);   RCC->APB2RSTR &= ~(1<<5);}while(0)
#define GPIOE_REG_RESET()	do{RCC->APB2RSTR |= (1<<6);   RCC->APB2RSTR &= ~(1<<6);}while(0)

/* Some generic macro */

#define IRQ_EXTI0		6
#define IRQ_EXTI1		7
#define IRQ_EXTI2		8
#define IRQ_EXTI3		9
#define IRQ_EXTI4		10


/* Bit position definitions SPI_CR1 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST	7	
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE		14	
#define SPI_CR1_BIDIMODE	15

/* Bit position definitions SPI_CR2 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE				2
#define SPI_CR2_TXEIE				7
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_ERRIE				5

/* Bit position definitions SPI_SR */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7


/** @addtogroup Peripheral_declaration
  * @{
  */

#define GPIOA			          ((GPIO_Regdef_t*)GPIOA_BASEADDR)
#define GPIOB			          ((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC			          ((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD			          ((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE			          ((GPIO_Regdef_t*)GPIOE_BASEADDR)

#define ADC1			          ((ADC_Regdef_t*)ADC1_BASEADDR)
#define ADC2			          ((ADC_Regdef_t*)ADC2_BASEADDR)

#define SPI1			          ((SPI_Regdef_t*)SPI1_BASEADDR)
#define SPI2			          ((SPI_Regdef_t*)SPI2_BASEADDR)
#define SPI3			          ((SPI_Regdef_t*)SPI3_BASEADDR)

#define DMA1                ((DMA_Regdef_t *) DMA1_BASE)
#define DMA2                ((DMA_Regdef_t *) DMA2_BASE)
#define DMA1_Channel1       ((DMA_Channel_Regdef_t *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_Regdef_t *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_Regdef_t *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_Regdef_t *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_Regdef_t *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_Regdef_t *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_Regdef_t *) DMA1_Channel7_BASE)
#define DMA2_Channel1       ((DMA_Channel_Regdef_t *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_Regdef_t *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_Regdef_t *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_Regdef_t *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_Regdef_t *) DMA2_Channel5_BASE)

#define FLASH               ((FLASH_TypeDef *) FLASH_BASE)

/******************  FLASH Keys  **********************************************/
#define RDP_Key                  ((uint16_t)0x00A5)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)


#define RESET 		    0
#define SET 			    1
#define FLAG_RESET 		0
#define FLAG_SET 			1
#define DISABLE 			0
#define ENABLE 			  1
#define ERROR 			  0
#define SUCCESS 			1

#endif /*__STM32F1XX_H*/