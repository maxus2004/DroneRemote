#include <stdbool.h>

#include "radio.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_pwr.h"
#include "string.h"

float J_MIN[] = { 0,0,0, 0 };
float J_MAX[] = { 4095,4095,4095, 4095 };
float J_SNAP = 0.2f;

struct Telemetry
{
	float yaw, pitch, roll, thrust, altitude;
	uint8_t buffer[64];
};
struct Telemetry telemetry;

bool altHold = false;
bool prevAltHoldBtn = true;

volatile bool gotResponse = false;
int packetLoss = 100;
int packetsLost = 0;
int packetsSent = 0;
int packetsPerSecond = 0;

uint32_t prevPacketLossCalculation = 0;

volatile bool ADC_DONE = false;
volatile bool ADC_FAIL = false;
uint16_t ADC_Data[4];

volatile uint32_t time = 0;

float joysticks[4];

void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
	{
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_HSI_Enable();

	/* Wait till HSI is ready */
	while (LL_RCC_HSI_IsReady() != 1)
	{
	}

	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 84, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1)
	{
	}

	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{
	}

	LL_Init1msTick(84000000);
	LL_SetSystemCoreClock(84000000);
	LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
	LL_SYSTICK_EnableIT();

}
void SysTick_Handler(void)
{
	time++;
}

void GPIO_init(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	//LEDs
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);

	//digital inputs
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);

}

void init_ADC(void)
{
	//ADC pins
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);

	//ADC
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
	LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);
	LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_SEQUENCE_CONV);
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_7);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_1);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_2);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_480CYCLES);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_480CYCLES);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_480CYCLES);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_480CYCLES);


	LL_ADC_Enable(ADC1);


	//DMA
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_MEDIUM);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);
	LL_DMA_ConfigAddresses(DMA2,
		LL_DMA_CHANNEL_0,
		LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
		(uint32_t)&ADC_Data,
		LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_0, 4);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_0);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_0);
	LL_DMA_EnableStream(DMA2, LL_DMA_CHANNEL_0);
	NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void DMA2_Stream0_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC0(DMA2))
	{
		ADC_DONE = true;
		LL_DMA_ClearFlag_TC0(DMA2);
	}
	if (LL_DMA_IsActiveFlag_TE0(DMA2))
	{
		ADC_FAIL = true;
		LL_DMA_ClearFlag_TE0(DMA2);
	}
}


void processData(uint8_t* data, uint8_t length)
{
	if (data[0] != 77)return;

	gotResponse = true;
	memcpy(&telemetry, data + 1, length - 1);

}

void sendData(float* joysticks, bool altitudeHold)
{
	uint8_t data[18] = { 0 };
	data[0] = 66;
	memcpy(data + 1, joysticks, 16);
	data[17] = altitudeHold;
	radio_send(data, 18);
}

void EXTI9_5_IRQHandler(void)
{
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);

	//read radio interrupt register
	uint8_t status = radio_readReg(0x03);
	radio_readReg(0x04);

	//packet sent
	if (status & 1 << 2)
	{
		radio_startReceiving();
	}
	//packet received
	if (status & 1 << 1)
	{
		uint8_t buffer[64];
		uint8_t length = radio_receive(buffer);
		processData(buffer, length);
	}
	//crc error
	if (status & 1 << 0)
	{
		radio_writeReg(0x08, 0x02);
		radio_writeReg(0x08, 0x00);
		radio_startReceiving();
	}
}

void setLED(int id, bool state) {
	if (state == true) {
		switch (id) {
		case 0:
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10);
			return;
		case 1:
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
			return;
		case 2:
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12);
			return;
		}
	}
	else {
		switch (id) {
		case 0:
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
			return;
		case 1:
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
			return;
		case 2:
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);
			return;
		}
	}
}
bool isPressed(int id) {
	switch (id) {
	case 0:
		return (GPIOA->IDR & (1 << 5));
	}
}


int main(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	NVIC_SetPriorityGrouping(5);

	SystemClock_Config();

	LL_mDelay(100);

	GPIO_init();
	init_ADC();

	radio_setup();

	//turn power led on
	setLED(0, true);

	for (;;)
	{

		//get joystick values
		ADC_DONE = false;
		ADC_FAIL = false;
		LL_ADC_REG_StartConversionSWStart(ADC1);
		while (!ADC_DONE && !ADC_FAIL) {}

		//convert and calibrate joystick values
		for (int i = 0; i < 4; i++)
		{
			float j = 0;
			j = ((float)ADC_Data[i] - J_MIN[i]) / (J_MAX[i] - J_MIN[i]) * 2 - 1;
			if (j < -J_SNAP) {
				j += J_SNAP;
			}
			else if (j > J_SNAP) {
				j -= J_SNAP;
			}
			else {
				j = 0;
			}
			j /= 1 - J_SNAP;
			joysticks[i] = j;
		}

		bool altHoldBtn = isPressed(0);
		if (altHoldBtn && !prevAltHoldBtn)
		{
			altHold = !altHold;
			setLED(1, altHold);
		}
		prevAltHoldBtn = altHoldBtn;

		gotResponse = false;
		sendData(joysticks, altHold);
		packetsSent++;

		LL_mDelay(15);

		//indicate lost packet
		setLED(2, !gotResponse);
		//calculate packet loss
		if (!gotResponse)
		{
			packetsLost++;
		}
		if (packetsSent > 0 && time - prevPacketLossCalculation > 1000)
		{
			prevPacketLossCalculation = time;
			packetLoss = packetsLost * 100 / packetsSent;
			packetsPerSecond = packetsSent;
			packetsLost = 0;
			packetsSent = 0;
		}
	}
}
