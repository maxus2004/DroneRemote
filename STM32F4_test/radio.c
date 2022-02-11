#include "radio.h"
#include <stm32f4xx_ll_spi.h>
#include <stm32f4xx_ll_gpio.h>
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stdbool.h"


static void radio_SPI_init(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);

	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);

	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF_5);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_5);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_5, LL_GPIO_AF_5);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);
	LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
	LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
	LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV32);
	LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
	LL_SPI_DisableCRC(SPI1);
	LL_SPI_Enable(SPI1);
}
static void radio_GPIO_init(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	//CS pin
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);

	//IRQ pin
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);

	//IRQ interrupt
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE6);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_6);
	NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
}

uint8_t SPI_sendReceive(uint8_t data)
{
	while (LL_SPI_IsActiveFlag_BSY(SPI1)) {}
	LL_SPI_TransmitData8(SPI1, data);
	while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
	while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {}
	return LL_SPI_ReceiveData8(SPI1);
}

static void cs1(void)
{
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
}
static void cs0(void)
{
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
}

bool irq(void)
{
	return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_6)) == 0;
}

void radio_writeReg(uint8_t reg, uint8_t data)
{
	cs0();
	SPI_sendReceive(0x80 | reg);
	SPI_sendReceive(data);
	cs1();
}

void radio_writeBurst(uint8_t reg, uint8_t* data, uint8_t length)
{
	cs0();
	SPI_sendReceive(0x80 | reg);
	for (int i = 0; i < length; i++)
	{
		SPI_sendReceive(data[i]);
	}
	cs1();
}

void radio_readBurst(uint8_t reg, uint8_t* data, uint8_t length)
{
	cs0();
	SPI_sendReceive(reg);
	for (int i = 0; i < length; i++)
	{
		data[i] = SPI_sendReceive(0xff);
	}
	cs1();
}

uint8_t radio_readReg(uint8_t reg)
{
	cs0();
	SPI_sendReceive(reg);
	uint8_t data = SPI_sendReceive(0xff);
	cs1();
	return data;
}

void radio_send(uint8_t* data, uint8_t length)
{
	//clear interrupts
	radio_readReg(0x03);
	radio_readReg(0x04);
	int val = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_6);
	//set packet length
	radio_writeReg(0x3E, length);
	//fill FIFO
	radio_writeBurst(0x7F, data, length);
	//send packet
	radio_writeReg(0x07, 0b00001001);
}

uint8_t radio_receive(uint8_t* data)
{
	//read packet length
	uint8_t length = radio_readReg(0x4B);
	//read FIFO
	radio_readBurst(0x7F, data, length);
	//clear interrupt
	radio_readReg(0x03);
	radio_readReg(0x04);
	//clear RX FIFO
	radio_writeReg(0x08, 0x02);
	radio_writeReg(0x08, 0x00);
	
	return length;
}

void radio_startReceiving(void)
{
	radio_writeReg(0x07, 0b00000101);
}

void radio_setup(void)
{
	radio_GPIO_init();
	radio_SPI_init();

	//software reset
	cs0();
	LL_mDelay(15);
	radio_readReg(0x03);
	radio_readReg(0x04);
	radio_writeReg(0x07, 0x80);
	while (!irq()) {}
	radio_readReg(0x03);
	radio_readReg(0x04);

	//set parameters from excel sheet
	//----------------------------
	//RX parameters
	radio_writeReg(0x1C, 0x82);
	radio_writeReg(0x1D, 0x40);
	radio_writeReg(0x20, 0x5E);
	radio_writeReg(0x21, 0x01);
	radio_writeReg(0x22, 0x5D);
	radio_writeReg(0x23, 0x86);
	radio_writeReg(0x24, 0x03);
	radio_writeReg(0x25, 0x7E);
	//enable packet handler and configure crc
	radio_writeReg(0x30, 0x8C);
	//disable header filtering
	radio_writeReg(0x32, 0x8C);
	//disable header, 2 bytes sync word
	radio_writeReg(0x33, 0x02);
	//preamble length
	radio_writeReg(0x34, 0x08);
	//preamble detection threshold 20 bits
	radio_writeReg(0x35, 0x24);
	//set sync word 0xABCD
	radio_writeReg(0x36, 0x2D);
	radio_writeReg(0x37, 0xD4);
	//
	radio_writeReg(0x38, 0x00);
	radio_writeReg(0x39, 0x00);
	radio_writeReg(0x3A, 0x00);
	radio_writeReg(0x3B, 0x00);
	radio_writeReg(0x3C, 0x00);
	radio_writeReg(0x3D, 0x00);
	radio_writeReg(0x3E, 0x00);
	radio_writeReg(0x3F, 0x00);
	radio_writeReg(0x40, 0x00);
	radio_writeReg(0x41, 0x00);
	radio_writeReg(0x42, 0x00);
	radio_writeReg(0x43, 0xFF);
	radio_writeReg(0x44, 0xFF);
	radio_writeReg(0x45, 0xFF);
	radio_writeReg(0x46, 0xFF);
	radio_writeReg(0x56, 0x00);
	//TX baud rate
	radio_writeReg(0x6E, 0x20);
	radio_writeReg(0x6F, 0xC5);
	radio_writeReg(0x70, 0x0C);
	//FIFO enabled and GFSK modulation
	radio_writeReg(0x71, 0x63);
	//frequency deviation
	radio_writeReg(0x72, 0x4E);
	//carrier frequency
	radio_writeReg(0x75, 0x53);
	radio_writeReg(0x76, 0x4E);
	radio_writeReg(0x77, 0x20);
	//---------------------------------

	//set SGI bit
	radio_writeReg(0x69, 0x60);
	//tx power +20dBm
	radio_writeReg(0x6D, 0b00000111);
	//oscillator capacitive load
	radio_writeReg(0x09, 0xD7);
	//configure GPIO for RF switch
	radio_writeReg(0x0B, 0x12);
	radio_writeReg(0x0C, 0x15);
	//enable data out
	radio_writeReg(0x0D, 0b00010100);

	//enable packet sent and received interrupts
	radio_writeReg(0x05, 0b00000111);
	radio_writeReg(0x06, 0b00000000);
	radio_readReg(0x03);
	radio_readReg(0x04);

	//set mode to ready
	radio_writeReg(0x07, 0b00000001);

	NVIC_EnableIRQ(EXTI9_5_IRQn);
}