#pragma once
#include <stdbool.h>
#include "stdint.h"

void radio_setup();
void radio_send(uint8_t* data, uint8_t length);
uint8_t radio_receive(uint8_t* data);
void radio_startReceiving(void);
void radio_writeReg(uint8_t reg, uint8_t data);
void radio_writeBurst(uint8_t reg, uint8_t* data, uint8_t length);
void radio_readBurst(uint8_t reg, uint8_t* data, uint8_t length);
uint8_t radio_readReg(uint8_t reg);
bool irq(void);
