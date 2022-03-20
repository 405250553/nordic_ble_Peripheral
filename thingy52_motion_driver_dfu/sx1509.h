#ifndef __SX1509_LIB_H
#define __SX1509_LIB_H

#include <stdio.h>
#include <stdlib.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#define SX1509_ADDRESS                  0X3E

void sx1509_twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void sx1509_i2c_init(uint32_t SCL_PIN,uint32_t SDA_PIN);

void sx1509_write_reg(uint8_t devAddr,uint8_t regAddr,uint8_t write_data);
void sx1509_write_word(uint8_t devAddr,uint8_t regAddr,uint16_t write_data);
void sx1509_write_bytes(uint8_t devAddr,uint8_t regAddr,uint8_t *write_data, uint8_t length);


void sx1509_read_reg(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
uint16_t sx1509_read_word(uint8_t devAddr, uint8_t regAddr);
void sx1509_data_read_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

uint8_t constrain(uint8_t val,uint8_t low,uint8_t high);
void configClock(uint8_t oscDivider);
void ledDriverInit(uint8_t pin);
void sx1509_reset(void);
void ext_gpio_output_digset(uint8_t pin,uint8_t highlow);
void ext_gpio_output_analog_set(uint8_t pin,uint8_t highlow);
uint8_t calculateLEDTRegister(uint8_t ms);
uint8_t calculateSlopeRegister(uint8_t ms, uint8_t onIntensity, uint8_t offIntensity);
void setupBlink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t onIntensity, uint8_t offIntensity, uint8_t tRise, uint8_t tFall);
void blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity);
void breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt);


#endif //__SX1509_LIB_H
