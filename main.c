#include "stm32f1xx.h"
#include "gpio.h"
#include "adc.h"
#include "delay.h"
#include "spi.h"
#include "dma.h"
#include "kalman.h"
#include "string.h"
#include "flash.h"

#define VREF 								3.3
#define SENSITIVITY					0.1
#define BUF_SIZE						3
#define SAMPLING_INTERVAL		3600.0
#define ADDR_DATA      			0x08008000

#define KALMAN_VOLTAGE			1
#define KALMAN_CURRENT			2
#define KALMAN_TEMPERATURE	3

#define R0 									0.03417
#define R1									0.02221
#define R2									0.01902
#define C1									1498.26
#define C2									65453.28

uint8_t data_SOC = 0, data_SOH = 0;


/*------------------------------------------------------ RCC CONFIG ------------------------------------------------------*/

void RCC_Config() {
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	ADC_PeripheralClockControl(ADC1, ENABLE);
	DMA_PeriClockControl(DMA1, ENABLE);
}

/*-----------------------------------------------------------------------------------------------------------------------*/


/*------------------------------------------------------ GPIO CONFIG -----------------------------------------------------*/

void GPIO_Config() {

	// Config GPIO ADC
	GPIO_SetState(GPIOA, 0, INPUT, ANALOG);
	GPIO_SetState(GPIOA, 1, INPUT, ANALOG);
	GPIO_SetState(GPIOA, 2, INPUT, ANALOG);
	
	// Config GPIO LED
	GPIO_SetState(GPIOC, 14, OUTPUT_50MHZ, PUSH_PULL);
	
	//Config GPIO Interrupt
	GPIO_SetState(GPIOB, 1, IT_FE, PUPD);
	
	// Config SPI2 pin
	GPIO_SetState(GPIOB, 12, OUTPUT_50MHZ, AF_PUSH_PULL);
	GPIO_SetState(GPIOB, 13, OUTPUT_50MHZ, AF_PUSH_PULL);
	GPIO_SetState(GPIOB, 14, OUTPUT_50MHZ, AF_PUSH_PULL);
	GPIO_SetState(GPIOB, 15, OUTPUT_50MHZ, AF_PUSH_PULL);
	
	// Config SPI1 pin
	GPIO_SetState(GPIOA, 4, OUTPUT_50MHZ, AF_PUSH_PULL);
	GPIO_SetState(GPIOA, 5, OUTPUT_50MHZ, AF_PUSH_PULL);
	GPIO_SetState(GPIOA, 6, OUTPUT_50MHZ, AF_PUSH_PULL);
	GPIO_SetState(GPIOA, 7, OUTPUT_50MHZ, AF_PUSH_PULL);
}

/*-----------------------------------------------------------------------------------------------------------------------*/


/*------------------------------------------------------ ADC CONFIG ------------------------------------------------------*/

// Declare the temperature sensor measurement variable
uint16_t adc_buffer[3];

uint16_t kal_voltage = 0;
float v1_in, v1_out, Vs;
float r1 = 30000.0;
float r2 = 7500.0;

// Declare the current sensor measurement variable
uint16_t kal_current = 0;
float current = 0;
float v2_in, v2_out;
	
// Declare the temperature sensor measurement variable
uint16_t kal_temperature = 0;
float temperature;
float v3;

// ADC configuration
void ADC_Config() {
	ADC_InitTypeDef ADC_InitStruct;
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_NbrOfChannel = 3;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	
	ADC_Init(ADC1, &ADC_InitStruct);
	// Configure channel, rank, sampling time
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	// Allow DMA
	ADC_DMACmd(ADC1, ENABLE);
	// Allow ADC1 to operate
	ADC_Cmd(ADC1, ENABLE);	
	// Start through the ADC conversion process
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*-----------------------------------------------------------------------------------------------------------------------*/


/*------------------------------------------------------ DMA CONFIG ------------------------------------------------------*/

void DMA_Config() {
	DMA_DeInit(DMA1_Channel1);
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &adc_buffer;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_BufferSize = BUF_SIZE;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

/*-----------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------- SENSOR -------------------------------------------------------*/

float Voltage_Sensor() {
	kal_voltage = (float) updateEstimate((float) adc_buffer[0], 1);
	v1_out = (kal_voltage * VREF) / 4096.0;
	v1_in = v1_out * (r1 + r2) / r2;
	Vs = v1_in * (30000 + 3642.857) / 3642.857;
	return Vs;
}

float Current_Sensor() {
	kal_current = (float) updateEstimate((float) adc_buffer[1], 2);
	v2_out = (float) (kal_current * VREF) / 4096.0;
	v2_in = v1_out * (30000 + 3643) / 3643;
	current = (v2_in - (VREF / 2)) / SENSITIVITY;
	return current;
}

float Temperature_Sensor() {
	kal_temperature = (float) updateEstimate((float) adc_buffer[2], 3);
	v3 = (kal_temperature * VREF) / 4096.0;
	temperature = v3 * 100;
	return temperature;
}

/*-----------------------------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------- ESTIMATE SOC ---------------------------------------------------*/

float SOC_init, C_init, currentC, previousC, I;
float currentSOC, previousSOC;
long previousTime = 0;
long currentTime = 0;

void SOC_Init(SOC_init, C_init) {
	currentSOC = SOC_init;
	currentC = C_init;
}

double CalculateBatteryCapacity(float current) {
    float capacity = current * (currentTime - previousTime) / 3600; 
    return capacity;
}

double CalculateSOH(double current_capacity, double reference_capacity) {
	double SOH = (current_capacity / reference_capacity) * 100.0;
	return SOH;
}

double SOC_Calculate() {
	currentSOC = previousSOC + (I * (currentTime - previousTime)) / currentC;
	previousSOC = currentSOC;
	
	// Update C
	previousC = currentC;
	currentC = CalculateBatteryCapacity(I);
	
	return currentSOC;
}

/*---------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------- SPI --------------------------------------------------------*/

void SPI1_Init(void)
{
	SPI_Handle_t SPI1Handle;
	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPI_PinConfig.SPI_SclkSpeed = SPI_CLK_SPEED_DIV128;  // sclk 2MHZ
	SPI1Handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI;  // hardware management
	SPI1Handle.pTxBuffer = ((void*)0);
	SPI1Handle.pRxBuffer = ((void*)0);
	SPI1Handle.TxLen = 0;
	SPI1Handle.RxLen = 0;
	SPI1Handle.TxState = SPI_READY;
	SPI1Handle.RxState = SPI_READY;
	
	SPI_Init(&SPI1Handle);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_PinConfig.SPI_SclkSpeed = SPI_CLK_SPEED_DIV128;  // sclk 2MHZ
	SPI2Handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI;  // hardware management
	SPI2Handle.pTxBuffer = ((void*)0);
	SPI2Handle.pRxBuffer = ((void*)0);
	SPI2Handle.TxLen = 0;
	SPI2Handle.RxLen = 0;
	SPI2Handle.TxState = SPI_READY;
	SPI2Handle.RxState = SPI_READY;
	
	SPI_Init(&SPI2Handle);
}

/*---------------------------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------ BOOTLOADER ---------------------------------------------------*/

uint32_t RxBuf[10];

void Flash_Erase(uint32_t addresspage){
	FLASH_Unlock();
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	FLASH_ErasePage(addresspage);
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	FLASH_Lock();
}

void Flash_WriteInt(uint32_t address, uint16_t value){
	FLASH_Unlock();
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	FLASH_ProgramHalfWord(address, value);
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	FLASH_Lock();
}

void Flash_WriteNumByte(uint32_t address, uint8_t *data, int num){
		
	FLASH_Unlock();
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	uint16_t *ptr = (uint16_t*)data;
	for(int i=0; i<((num+1)/2); i++){
		FLASH_ProgramHalfWord(address+2*i, *ptr);
		while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
		ptr++;
	}
	FLASH_Lock();
}

void Flash_ReadData(uint32_t address, uint32_t *RxBuf) {
	for(int i = 0; i < 2; i++) {
		uint32_t data = *(volatile uint32_t*)address;
		*RxBuf = data & 0xFF;
		RxBuf++;
		address++;
	}
}

/*---------------------------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------ INTERRUPT ----------------------------------------------------*/

void EXTI1_IRQHandler(void) {
	// Write/Read Data on Memory Flash
	uint8_t data[2] = {data_SOC, data_SOH};
	Flash_Erase(ADDR_DATA);
	Flash_Erase(ADDR_DATA + 1024);
	Flash_WriteNumByte(ADDR_DATA, &data, sizeof(data));
	GPIO_IRQHandling(1);
}

/*---------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------- MAIN -------------------------------------------------------*/

int main() {
	RCC_Config();
	GPIO_Config();
	ADC_Config();
	DMA_Config();
	GPIO_IRQInteruptConfig(IRQ_EXTI1, ENABLE);
	Systick_Initialize();
	SPI2_Init();
	SPI1_Init();
	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_SSOEConfig(SPI1, ENABLE);
	
	// Initialize SOC from flash memory
	uint8_t data[2];
	Flash_ReadData(ADDR_DATA, data);
	uint8_t SOC_init = data[0];
	uint8_t SOH_init = data[1];
	SOC_Init(SOC_init, 12000);
	
	// Kalman Filter
	SimpleKalmanFilter(1, 2, 0.001, KALMAN_VOLTAGE);
	SimpleKalmanFilter(1, 2, 0.001, KALMAN_CURRENT);
	SimpleKalmanFilter(1, 2, 0.001, KALMAN_TEMPERATURE);
	
	currentTime = millis();
	
	while(1) {
		
		// Sensor
		Voltage_Sensor();
		Current_Sensor();
		Temperature_Sensor();
		
		data_SOH = CalculateSOH(currentC, 12000);
		data_SOC = SOC_Calculate();
		
		//SPI1 send data SOC
		SPI_PeripheralControl(SPI1, ENABLE);
		SPI_SendData(SPI1, &data_SOC, 1);
		while (SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI1, DISABLE);
		
		//SPI2 send data SOH
		SPI_PeripheralControl(SPI2, ENABLE);
		SPI_SendData(SPI2, &data_SOH, 1);
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		SPI_PeripheralControl(SPI2, DISABLE);
		delay_ms(100);
		
	}
	
}

/*---------------------------------------------------------------------------------------------------------------------*/