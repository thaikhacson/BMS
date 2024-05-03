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

volatile uint32_t TimeDelay;

/*------------------------------------------------------ RCC CONFIG ------------------------------------------------------*/

void RCC_Config() {
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	ADC_PeripheralClockControl(ADC1, ENABLE);
	DMA_PeriClockControl(DMA1, ENABLE);
}

/*-----------------------------------------------------------------------------------------------------------------------*/


/*------------------------------------------------------ GPIO CONFIG -----------------------------------------------------*/

// Config GPIO ADC
void GPIO_Config() {
	GPIO_Handle_t GPIO_Config;
	GPIO_Config.pGPIOx = GPIOA;
	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = INPUT;
	GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = 0;
	GPIO_Config.GPIO_PinConfig.GPIO_IOType = ANALOG;
	GPIO_Init(&GPIO_Config);
	
	GPIO_Config.pGPIOx = GPIOA;
	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = INPUT;
	GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = 1;
	GPIO_Config.GPIO_PinConfig.GPIO_IOType = ANALOG;
	GPIO_Init(&GPIO_Config);
	
	GPIO_Config.pGPIOx = GPIOA;
	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = INPUT;
	GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = 2;
	GPIO_Config.GPIO_PinConfig.GPIO_IOType = ANALOG;
	GPIO_Init(&GPIO_Config);
	
	//Config GPIO Interrupt
	GPIO_Config.pGPIOx = GPIOA;
	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = IT_FE;
	GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = 3;
	GPIO_Config.GPIO_PinConfig.GPIO_IOType = PUPD;
	GPIO_Init(&GPIO_Config);
	
	GPIO_Config.pGPIOx = GPIOC;
	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = OUTPUT_50MHZ;
	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = 13;
	GPIO_Config.GPIO_PinConfig.GPIO_IOType = PUSH_PULL;
	GPIO_Init(&GPIO_Config);
	
//	// GPIO pins for MOSI, MISO, and SCK
//	GPIO_Config.pGPIOx = GPIOB;
//	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = OUTPUT_50MHZ;
//	GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = 13;
//	GPIO_Config.GPIO_PinConfig.GPIO_IOType = AF_PUSH_PULL;
//	GPIO_Init(&GPIO_Config);
//	
//	GPIO_Config.pGPIOx = GPIOB;
//	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = OUTPUT_50MHZ;
//	GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = 14;
//	GPIO_Config.GPIO_PinConfig.GPIO_IOType = AF_PUSH_PULL;
//	GPIO_Init(&GPIO_Config);
//	
//	GPIO_Config.pGPIOx = GPIOB;
//	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = OUTPUT_50MHZ;
//	GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = 15;
//	GPIO_Config.GPIO_PinConfig.GPIO_IOType = AF_PUSH_PULL;
//	GPIO_Init(&GPIO_Config);
//	
//	// GPIO pin for SS
//	GPIO_Config.pGPIOx = GPIOA;
//	GPIO_Config.GPIO_PinConfig.GPIO_PinMode = OUTPUT_50MHZ;
//	GPIO_Config.GPIO_PinConfig.GPIO_PinNumber = 12;
//	GPIO_Config.GPIO_PinConfig.GPIO_IOType = PUSH_PULL;
//	GPIO_Init(&GPIO_Config);
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

//float SOC_init, C_init, currentC, previousC, I;
//float currentSOC, previousSOC;
//long previousTime = 0;
//long currentTime = 0;

//void SOC_Init(SOC_init, C_init) {
//	currentSOC = SOC_init;
//	currentC = C_init;
//	
//	currentTime = s();
//}

////double CalculateBatteryCapacity() {
////	float I_current = Current_Sensor();
////	float I_previous;
////		
////	float currentTime = s();
////	
////	if(currentTime - previousTime > 1) {
////		previousTime = currentTime;
////		
////		currentC = (I_current - I_previous) / 3600;
////	}
////	
////	return currentC;
////}

//double CalculateSOH(double current_capacity, double reference_capacity) {
//	double SOH = (current_capacity / reference_capacity) * 100.0;
//	return SOH;
//}

//float CalculateCurrent(float Q_current, float Q_previous, float delta_t) {
//	return (Q_current - Q_previous) / delta_t;
//}

//void SOC_Calculate() {
//	// Calculate current time
//	// currentTime = millis();
//	
//	currentSOC = previousSOC + (I * (currentTime - previousTime)) / currentC;
//	previousSOC = currentSOC;
//	
//	// Update C
//	previousC = currentC;
//	currentC = CalculateBatteryCapacity(10);
//	
//	// Updated to previous time
//	previousTime = currentTime;
//}

/*---------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------- SPI --------------------------------------------------------*/

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = OUTPUT_10MHZ;
	SPIPins.GPIO_PinConfig.GPIO_IOType = AF_PUSH_PULL;

	/* SCLK*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPIPins);
	/* MOSI*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPIPins);
	/* MISO*/
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 14;
//	GPIO_Init(&SPIPins);
	/* NSS*/
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&SPIPins);
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

uint8_t data[2] = {0x56, 0x59};

void EXTI1_IRQHandler(void) {
	// Write/Read Data on Memory Flash
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
	
	int a = 13, b = 12, c = 31;
	
	Systick_Initialize();
	char user_data[3] = {a, b, c};
	SPI2_GPIOInit();
	SPI2_Init();
	SPI_SSOEConfig(SPI2, ENABLE);
	
	// Kalman Filter
	SimpleKalmanFilter(1, 2, 0.001, KALMAN_VOLTAGE);
	SimpleKalmanFilter(1, 2, 0.001, KALMAN_CURRENT);
	SimpleKalmanFilter(1, 2, 0.001, KALMAN_TEMPERATURE);
	
	// Write/Read Data on Memory Flash
//	Flash_Erase(ADDR_DATA);
//	Flash_Erase(ADDR_DATA + 1024);
//	Flash_WriteNumByte(ADDR_DATA, &data, sizeof(data));
//	Flash_ReadData(ADDR_DATA, RxBuf);
	
	while(1) {
		
		// Sensor
		Voltage_Sensor();
		//user_data = Current_Sensor();
		
		Temperature_Sensor();
		
		SPI_PeripheralControl(SPI2, ENABLE);
		for (int i = 0; i < 3; i++) {
			SPI_SendData(SPI2, &user_data[i], 1);
			while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
			delay_ms(100);
		}
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	
}

/*---------------------------------------------------------------------------------------------------------------------*/