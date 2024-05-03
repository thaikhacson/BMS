#include "dma.h"

void DMA_PeriClockControl(DMA_Regdef_t* pDMAx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pDMAx == DMA1)  					{DMA1_CLK_EN();}
		else if(pDMAx == DMA2)			{DMA2_CLK_EN();}
	}
	else if(EnOrDi == DISABLE)
	{
		if(pDMAx == DMA1)  					{DMA1_CLK_DI();}
		else if(pDMAx == DMA2)			{DMA2_CLK_DI();}
	}
}

void DMA_DeInit(DMA_Channel_Regdef_t* DMAy_Channelx) {
	/* Disable the selected DMAy Channelx */
  DMAy_Channelx->CCR &= (0<<0);
  
  /* Reset DMAy Channelx control register */
  DMAy_Channelx->CCR  = 0;
  
  /* Reset DMAy Channelx remaining bytes register */
  DMAy_Channelx->CNDTR = 0;
  
  /* Reset DMAy Channelx peripheral address register */
  DMAy_Channelx->CPAR  = 0;
  
  /* Reset DMAy Channelx memory address register */
  DMAy_Channelx->CMAR = 0;
  
  if (DMAy_Channelx == DMA1_Channel1)
  {
    /* Reset interrupt pending bits for DMA1 Channel1 */
    DMA1->IFCR |= (1<<0) | (1<<1) | (1<<2) | (1<<3);
  }
  else if (DMAy_Channelx == DMA1_Channel2)
  {
    /* Reset interrupt pending bits for DMA1 Channel2 */	
    DMA1->IFCR |= (1<<4) | (1<<5) | (1<<6) | (1<<7);
  }
  else if (DMAy_Channelx == DMA1_Channel3)
  {
    /* Reset interrupt pending bits for DMA1 Channel3 */
    DMA1->IFCR |= (1<<8) | (1<<9) | (1<<10) | (1<<11);
  }
  else if (DMAy_Channelx == DMA1_Channel4)
  {
    /* Reset interrupt pending bits for DMA1 Channel4 */
    DMA1->IFCR |= (1<<12) | (1<<13) | (1<<14) | (1<<15);
  }
  else if (DMAy_Channelx == DMA1_Channel5)
  {
    /* Reset interrupt pending bits for DMA1 Channel5 */
    DMA1->IFCR |= (1<<16) | (1<<17) | (1<<18) | (1<<19);
  }
  else if (DMAy_Channelx == DMA1_Channel6)
  {
    /* Reset interrupt pending bits for DMA1 Channel6 */
    DMA1->IFCR |= (1<<20) | (1<<21) | (1<<22) | (1<<23);
  }
  else if (DMAy_Channelx == DMA1_Channel7)
  {
    /* Reset interrupt pending bits for DMA1 Channel7 */
    DMA1->IFCR |= (1<<24) | (1<<25) | (1<<26) | (1<<27);
  }
  else if (DMAy_Channelx == DMA2_Channel1)
  {
    /* Reset interrupt pending bits for DMA2 Channel1 */
    DMA2->IFCR |= (1<<0) | (1<<1) | (1<<2) | (1<<3);
  }
  else if (DMAy_Channelx == DMA2_Channel2)
  {
    /* Reset interrupt pending bits for DMA2 Channel2 */
    DMA2->IFCR |= (1<<4) | (1<<5) | (1<<6) | (1<<7);
  }
  else if (DMAy_Channelx == DMA2_Channel3)
  {
    /* Reset interrupt pending bits for DMA2 Channel3 */
    DMA2->IFCR |= (1<<8) | (1<<9) | (1<<10) | (1<<11);
  }
  else if (DMAy_Channelx == DMA2_Channel4)
  {
    /* Reset interrupt pending bits for DMA2 Channel4 */
    DMA2->IFCR |= (1<<12) | (1<<13) | (1<<14) | (1<<15);
  }
  else
  { 
    if (DMAy_Channelx == DMA2_Channel5)
    {
      /* Reset interrupt pending bits for DMA2 Channel5 */
      DMA2->IFCR |= (1<<16) | (1<<17) | (1<<18) | (1<<19);
    }
  }
}

void DMA_Init(DMA_Channel_Regdef_t* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct) {
	uint32_t tmpreg = 0;
	/*--------------------------- DMAy Channelx CCR Configuration -----------------*/
  /* Get the DMAy_Channelx CCR value */
  tmpreg = DMAy_Channelx->CCR;
  /* Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
  tmpreg &= ((uint32_t)0xFFFF800F);
  /* Configure DMAy Channelx: data transfer, data size, priority level and mode */
  /* Set DIR bit according to DMA_DIR value */
  /* Set CIRC bit according to DMA_Mode value */
  /* Set PINC bit according to DMA_PeripheralInc value */
  /* Set MINC bit according to DMA_MemoryInc value */
  /* Set PSIZE bits according to DMA_PeripheralDataSize value */
  /* Set MSIZE bits according to DMA_MemoryDataSize value */
  /* Set PL bits according to DMA_Priority value */
  /* Set the MEM2MEM bit according to DMA_M2M value */
  tmpreg |= DMA_InitStruct->DMA_DIR | DMA_InitStruct->DMA_Mode |
            DMA_InitStruct->DMA_PeripheralInc | DMA_InitStruct->DMA_MemoryInc |
            DMA_InitStruct->DMA_PeripheralDataSize | DMA_InitStruct->DMA_MemoryDataSize |
            DMA_InitStruct->DMA_Priority | DMA_InitStruct->DMA_M2M;

  /* Write to DMAy Channelx CCR */
  DMAy_Channelx->CCR = tmpreg;

/*--------------------------- DMAy Channelx CNDTR Configuration ---------------*/
  /* Write to DMAy Channelx CNDTR */
  DMAy_Channelx->CNDTR = DMA_InitStruct->DMA_BufferSize;

/*--------------------------- DMAy Channelx CPAR Configuration ----------------*/
  /* Write to DMAy Channelx CPAR */
  DMAy_Channelx->CPAR = DMA_InitStruct->DMA_PeripheralBaseAddr;

/*--------------------------- DMAy Channelx CMAR Configuration ----------------*/
  /* Write to DMAy Channelx CMAR */
  DMAy_Channelx->CMAR = DMA_InitStruct->DMA_MemoryBaseAddr;
}

void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct) {
	/*-------------- Reset DMA init structure parameters values ------------------*/
  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStruct->DMA_PeripheralBaseAddr = 0;
  /* Initialize the DMA_MemoryBaseAddr member */
  DMA_InitStruct->DMA_MemoryBaseAddr = 0;
  /* Initialize the DMA_DIR member */
  DMA_InitStruct->DMA_DIR = DMA_DIR_PeripheralSRC;
  /* Initialize the DMA_BufferSize member */
  DMA_InitStruct->DMA_BufferSize = 0;
  /* Initialize the DMA_PeripheralInc member */
  DMA_InitStruct->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  /* Initialize the DMA_MemoryInc member */
  DMA_InitStruct->DMA_MemoryInc = DMA_MemoryInc_Disable;
  /* Initialize the DMA_PeripheralDataSize member */
  DMA_InitStruct->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  /* Initialize the DMA_MemoryDataSize member */
  DMA_InitStruct->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  /* Initialize the DMA_Mode member */
  DMA_InitStruct->DMA_Mode = DMA_Mode_Normal;
  /* Initialize the DMA_Priority member */
  DMA_InitStruct->DMA_Priority = DMA_Priority_Low;
  /* Initialize the DMA_M2M member */
  DMA_InitStruct->DMA_M2M = DMA_M2M_Disable;
}

void DMA_Cmd(DMA_Channel_Regdef_t* DMAy_Channelx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE)
  {
    /* Enable the selected DMAy Channelx */
    DMAy_Channelx->CCR |= (1<<0);
  }
  else if (EnOrDi == DISABLE)
  {
    /* Disable the selected DMAy Channelx */
    DMAy_Channelx->CCR &= ~(1<<0);
  }
}
