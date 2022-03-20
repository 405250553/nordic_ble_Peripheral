#include "sx1509.h"
#include "sx1509_registers.h"

#define TWI_INSTANCE_ID     1
static volatile bool sx1509_xfer_done = false;
static const nrf_drv_twi_t sx1509_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static volatile float _clkX = 0.0;

uint8_t REG_I_ON[16] = {REG_I_ON_0, REG_I_ON_1, REG_I_ON_2, REG_I_ON_3,
					REG_I_ON_4, REG_I_ON_5, REG_I_ON_6, REG_I_ON_7,
					REG_I_ON_8, REG_I_ON_9, REG_I_ON_10, REG_I_ON_11,
					REG_I_ON_12, REG_I_ON_13, REG_I_ON_14, REG_I_ON_15};
					
uint8_t REG_T_ON[16] = {REG_T_ON_0, REG_T_ON_1, REG_T_ON_2, REG_T_ON_3,
					REG_T_ON_4, REG_T_ON_5, REG_T_ON_6, REG_T_ON_7,
					REG_T_ON_8, REG_T_ON_9, REG_T_ON_10, REG_T_ON_11,
					REG_T_ON_12, REG_T_ON_13, REG_T_ON_14, REG_T_ON_15};
					
uint8_t REG_OFF[16] = {REG_OFF_0, REG_OFF_1, REG_OFF_2, REG_OFF_3,
					REG_OFF_4, REG_OFF_5, REG_OFF_6, REG_OFF_7,
					REG_OFF_8, REG_OFF_9, REG_OFF_10, REG_OFF_11,
					REG_OFF_12, REG_OFF_13, REG_OFF_14, REG_OFF_15};

uint8_t REG_T_RISE[16] = {0xFF, 0xFF, 0xFF, 0xFF,
					REG_T_RISE_4, REG_T_RISE_5, REG_T_RISE_6, REG_T_RISE_7,
					0xFF, 0xFF, 0xFF, 0xFF,
					REG_T_RISE_12, REG_T_RISE_13, REG_T_RISE_14, REG_T_RISE_15};
					
uint8_t REG_T_FALL[16] = {0xFF, 0xFF, 0xFF, 0xFF,
					REG_T_FALL_4, REG_T_FALL_5, REG_T_FALL_6, REG_T_FALL_7,
					0xFF, 0xFF, 0xFF, 0xFF,
					REG_T_FALL_12, REG_T_FALL_13, REG_T_FALL_14, REG_T_FALL_15};


void sx1509_twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            sx1509_xfer_done = true;
            break;
        default:
            break;
    }
}

void sx1509_i2c_init(uint32_t SCL_PIN,uint32_t SDA_PIN)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false,
       //.hold_bus_uninit    =  TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
    };

    err_code = nrf_drv_twi_init(&sx1509_twi, &twi_config, sx1509_twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&sx1509_twi);
}

void sx1509_write_reg(uint8_t devAddr,uint8_t regAddr,uint8_t write_data)
{
          ret_code_t err_code;
          uint8_t buf_len =2; // Register address + number of bytes
          uint8_t tx_buf[buf_len];

          tx_buf[0] = regAddr;
          tx_buf[1] = write_data;

  	  sx1509_xfer_done=false;
	  err_code = nrf_drv_twi_tx(&sx1509_twi,devAddr, tx_buf, 2, false);
	  APP_ERROR_CHECK(err_code);
          while (sx1509_xfer_done == false);
}

void sx1509_write_word(uint8_t devAddr,uint8_t regAddr,uint16_t write_data)
{
	  ret_code_t err_code;
          uint8_t buf_len =3; // Register address + number of bytes
          uint8_t tx_buf[buf_len];

          tx_buf[0] = regAddr;
          tx_buf[1] = (uint8_t)( write_data >> 8);
          tx_buf[2] = (uint8_t)( write_data );

  	  sx1509_xfer_done=false;
	  err_code = nrf_drv_twi_tx(&sx1509_twi,devAddr, tx_buf, 3, false);
	  APP_ERROR_CHECK(err_code);
          while (sx1509_xfer_done == false);
}

void sx1509_write_bytes(uint8_t devAddr,uint8_t regAddr,uint8_t *write_data, uint8_t length)
{
	  ret_code_t err_code;
          uint8_t buf_len =length+1; // Register address + number of bytes
          uint8_t tx_buf[buf_len];

          tx_buf[0] = regAddr;
          for(uint8_t i=0;i<length;i++)
              tx_buf[i+1] = write_data[i];

  	  sx1509_xfer_done=false;
	  err_code = nrf_drv_twi_tx(&sx1509_twi,devAddr, tx_buf, buf_len, false);
	  APP_ERROR_CHECK(err_code);
          while (sx1509_xfer_done == false);
}

void sx1509_read_reg(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
   sx1509_data_read_bytes(devAddr, regAddr, 1, data);
}

uint16_t sx1509_read_word(uint8_t devAddr, uint8_t regAddr)
{
	  ret_code_t err_code;
          uint8_t data[2] = {0};
		
	  sx1509_xfer_done=false;
	  err_code = nrf_drv_twi_tx(&sx1509_twi, devAddr, &regAddr, 1, false);
 	  APP_ERROR_CHECK(err_code);
          while (sx1509_xfer_done == false);


	  sx1509_xfer_done = false;
	  err_code = nrf_drv_twi_rx(&sx1509_twi,devAddr,data,2);
	  APP_ERROR_CHECK(err_code);
	  while (sx1509_xfer_done == false);

          uint16_t word_return = (uint16_t)((data[0]<<8)+data[1]);

          return word_return;
		
}

void sx1509_data_read_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	  ret_code_t err_code;
		
	  sx1509_xfer_done=false;
	  err_code = nrf_drv_twi_tx(&sx1509_twi, devAddr, &regAddr, 1, false);
 	  APP_ERROR_CHECK(err_code);
          while (sx1509_xfer_done == false);


	  sx1509_xfer_done = false;
	  err_code = nrf_drv_twi_rx(&sx1509_twi,devAddr,data,length);
	  APP_ERROR_CHECK(err_code);
	  while (sx1509_xfer_done == false);
		
}

uint8_t constrain(uint8_t val,uint8_t low,uint8_t high){
    
    uint8_t return_data =  val;
    if(return_data>high)
      return_data=high;

    if(return_data<low)
      return_data=low;
    
    return return_data;
};

void configClock(uint8_t oscDivider)
{

        uint8_t oscSource= 2;
        uint8_t oscPinFunction= 0;
        uint8_t oscFreqOut= 0;

        //uint8_t send_Byte[] = {0,0};

	// RegClock constructed as follows:
	//	6:5 - Oscillator frequency souce
	//		00: off, 01: external input, 10: internal 2MHz, 1: reserved
	//	4 - OSCIO pin function
	//		0: input, 1 ouptut
	//	3:0 - Frequency of oscout pin
	//		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
	oscSource = (oscSource & 3) << 5;		// 2-bit value, bits 6:5
	oscPinFunction = (oscPinFunction & 1) << 4; // 1-bit value bit 4
	oscFreqOut = (oscFreqOut & 15);			// 4-bit value, bits 3:0
	uint8_t regClock = oscSource | oscPinFunction | oscFreqOut;
        //send_Byte[0] = REG_CLOCK;
        //send_Byte[1] = regClock;
        //sx1509_write_reg(SX1509_ADDRESS,send_Byte);
        sx1509_write_reg(SX1509_ADDRESS,REG_CLOCK,regClock);

	// Config RegMisc[6:4] with oscDivider
	// 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
	oscDivider = constrain(oscDivider, 1, 7);
	_clkX = 2000000.0 / (1 << (oscDivider - 1)); // Update private clock variable
	oscDivider = (oscDivider & 7) << 4;		 // 3-bit value, bits 6:4

	uint8_t regMisc;
        sx1509_read_reg(SX1509_ADDRESS,REG_MISC, &regMisc);
	regMisc &= ~(7 << 4);
	regMisc |= oscDivider;
        //send_Byte[0] = REG_MISC;
        //send_Byte[1] = regMisc;
	//sx1509_write_reg(SX1509_ADDRESS,send_Byte);
        sx1509_write_reg(SX1509_ADDRESS,REG_MISC,regMisc);
}

void ledDriverInit(uint8_t pin){

    uint8_t tempByte;
    //uint8_t send_Byte[] = {0,tempByte};
    uint16_t tempWord;

    // Disable input buffer
    // Writing a 1 to the pin bit will disable that pins input buffer
    tempWord = sx1509_read_word(SX1509_ADDRESS,REG_INPUT_DISABLE_B);
    tempWord |= (1 << pin);
    sx1509_write_word(SX1509_ADDRESS,REG_INPUT_DISABLE_B,tempWord);

    // Disable pull-up
    // Writing a 0 to the pin bit will disable that pull-up resistor
    tempWord = sx1509_read_word(SX1509_ADDRESS,REG_PULL_UP_B);
    tempWord &= ~(1 << pin);
    sx1509_write_word(SX1509_ADDRESS,REG_PULL_UP_B,tempWord);

    // Set direction to output (REG_DIR_B)
    tempWord = sx1509_read_word(SX1509_ADDRESS,REG_DIR_B);
    tempWord &= ~(1 << pin); // 0=output
    sx1509_write_word(SX1509_ADDRESS,REG_DIR_B,tempWord);

    // Enable oscillator (REG_CLOCK)
    sx1509_read_reg(SX1509_ADDRESS,REG_CLOCK,&tempByte);
    tempByte |= (1 << 6);  // Internal 2MHz oscillator part 1 (set bit 6)
    tempByte &= ~(1 << 5); // Internal 2MHz oscillator part 2 (clear bit 5)
    //send_Byte[0] = REG_CLOCK;
    //send_Byte[1] = tempByte;
    //sx1509_write_reg(SX1509_ADDRESS, send_Byte);
    sx1509_write_reg(SX1509_ADDRESS,REG_CLOCK,tempByte);

    // Configure LED driver clock and mode (REG_MISC)
    sx1509_read_reg(SX1509_ADDRESS,REG_MISC,&tempByte);
    tempByte &= ~(1 << 7); // set linear mode bank B
    tempByte &= ~(1 << 3); // set linear mode bank A

    // Use configClock to setup the clock divder
    if (_clkX == 0) // Make clckX non-zero
    {
            // _clkX = 2000000.0 / (1 << (1 - 1)); // Update private clock variable
            _clkX = 2000000.0;

            // uint8_t freq = (1 & 0x07) << 4; // freq should only be 3 bits from 6:4
            // tempByte |= freq;
    }

    uint8_t freq = 1;
    freq = (freq & 0x7) << 4;	// mask only 3 bits and shift to bit position 6:4 
    tempByte |= freq;

    //send_Byte[0] = REG_MISC;
    //send_Byte[1] = tempByte;
    //sx1509_write_reg(SX1509_ADDRESS, send_Byte);
    sx1509_write_reg(SX1509_ADDRESS,REG_MISC,tempByte);

    // Enable LED driver operation (REG_LED_DRIVER_ENABLE)
    tempWord = sx1509_read_word(SX1509_ADDRESS,REG_LED_DRIVER_ENABLE_B);
    tempWord |= (1 << pin);
    sx1509_write_word(SX1509_ADDRESS,REG_LED_DRIVER_ENABLE_B,tempWord);

    // Set REG_DATA bit high ~ LED driver started
    tempWord = sx1509_read_word(SX1509_ADDRESS,REG_DATA_B);
    //tempWord |= (1 << pin);
    tempWord &= ~(1 << pin);
    sx1509_write_word(SX1509_ADDRESS,REG_DATA_B,tempWord);
}

void sx1509_reset(){
    //uint8_t tempByte[2] = {REG_RESET,0x12};
    //sx1509_write_reg(SX1509_ADDRESS,tempWord);
    //tempByte[1] = 0x34;
    //sx1509_write_reg(SX1509_ADDRESS,tempWord);
    sx1509_write_reg(SX1509_ADDRESS,REG_RESET,0x12);
    sx1509_write_reg(SX1509_ADDRESS,REG_RESET,0x34);
}

void ext_gpio_output_digset(uint8_t pin,uint8_t highlow){

    uint16_t tempWord=0;

    tempWord = sx1509_read_word(SX1509_ADDRESS,REG_DIR_B);
    tempWord &= ~(1 << pin);  // 0=output
    sx1509_write_word(SX1509_ADDRESS,REG_DIR_B,tempWord);

    
    tempWord = sx1509_read_word(SX1509_ADDRESS,REG_DATA_B);
    if(highlow)
      tempWord |= (1 << pin);
    else
      tempWord &= ~(1 << pin);
  
    sx1509_write_word(SX1509_ADDRESS,REG_DATA_B,tempWord);
}

void ext_gpio_output_analog_set(uint8_t pin,uint8_t highlow){

    ledDriverInit(pin);

    //uint8_t send_Byte[] = {REG_I_ON[pin],0};

    for(int i=0;i<256;i++){
      //send_Byte[1] = i;
      //sx1509_write_reg(SX1509_ADDRESS,send_Byte);
      sx1509_write_reg(SX1509_ADDRESS,REG_I_ON[pin],i);
      nrf_delay_ms(10);
    }
}

uint8_t calculateLEDTRegister(uint8_t ms)
{
	uint8_t regOn1, regOn2;
	float timeOn1, timeOn2;

	if (_clkX == 0)
		return 0;

	regOn1 = (float)(ms / 1000.0) / (64.0 * 255.0 / (float)_clkX);
	regOn2 = regOn1 / 8;
	regOn1 = constrain(regOn1, 1, 15);
	regOn2 = constrain(regOn2, 16, 31);

	timeOn1 = 64.0 * regOn1 * 255.0 / _clkX * 1000.0;
	timeOn2 = 512.0 * regOn2 * 255.0 / _clkX * 1000.0;

	if (abs(timeOn1 - ms) < abs(timeOn2 - ms))
		return regOn1;
	else
		return regOn2;
}

uint8_t calculateSlopeRegister(uint8_t ms, uint8_t onIntensity, uint8_t offIntensity)
{
	uint16_t regSlope1, regSlope2;
	float regTime1, regTime2;

	if (_clkX == 0)
		return 0;

	float tFactor = ((float)onIntensity - (4.0f * (float)offIntensity)) * 255.0f / (float)_clkX;
	float timeS = (float)(ms) / 1000.0f;

	regSlope1 = timeS / tFactor;
	regSlope2 = regSlope1 / 16;

	regSlope1 = constrain(regSlope1, 1, 15);
	regSlope2 = constrain(regSlope2, 16, 31);

	regTime1 = regSlope1 * tFactor * 1000.0f;
	regTime2 = 16 * regTime1;

	if (abs(regTime1 - ms) < abs(regTime2 - ms))
		return regSlope1;
	else
		return regSlope2;
}

void setupBlink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t onIntensity, uint8_t offIntensity, uint8_t tRise, uint8_t tFall)
{
    //uint8_t tempByte[2] = {REG_RESET,0x00};
    // Keep parameters within their limits:
    tOn &= 0x1F;  // tOn should be a 5-bit value
    tOff &= 0x1F; // tOff should be a 5-bit value
    offIntensity &= 0x07;
    // Write the time on
    // 1-15:  TON = 64 * tOn * (255/ClkX)
    // 16-31: TON = 512 * tOn * (255/ClkX)s
    //tempByte[0] = REG_T_ON[pin];
    //tempByte[1] = tOn;
    //sx1509_write_reg(SX1509_ADDRESS,tempByte);
    sx1509_write_reg(SX1509_ADDRESS,REG_T_ON[pin],tOn);

    // Write the time/intensity off register
    // 1-15:  TOFF = 64 * tOff * (255/ClkX)
    // 16-31: TOFF = 512 * tOff * (255/ClkX)
    // linear Mode - IOff = 4 * offIntensity
    // log mode - Ioff = f(4 * offIntensity)
    //tempByte[0] = REG_OFF[pin];
    //tempByte[1] = (tOff << 3) | offIntensity;
    //sx1509_write_reg(SX1509_ADDRESS,tempByte);
    sx1509_write_reg(SX1509_ADDRESS,REG_OFF[pin], (tOff << 3) | offIntensity );

    // Write the on intensity:
    //tempByte[0] = REG_I_ON[pin];
    //tempByte[1] = onIntensity;
    //sx1509_write_reg(SX1509_ADDRESS,tempByte);
    sx1509_write_reg(SX1509_ADDRESS,REG_I_ON[pin],onIntensity);

    // Prepare tRise and tFall
    tRise &= 0x1F; // tRise is a 5-bit value
    tFall &= 0x1F; // tFall is a 5-bit value

    // Write regTRise
    // 0: Off
    // 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
    // 16-31: TRise = 16 * (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
    if (REG_T_RISE[pin] != 0xFF){
      //tempByte[0] = REG_T_RISE[pin];
      //tempByte[1] = tRise;
      //sx1509_write_reg(SX1509_ADDRESS,tempByte);
      sx1509_write_reg(SX1509_ADDRESS,REG_T_RISE[pin],tRise);
    }
    // Write regTFall
    // 0: off
    // 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
    // 16-31: TFall = 16 * (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
    if (REG_T_FALL[pin] != 0xFF){
      //tempByte[0] = REG_T_FALL[pin];
      //tempByte[1] = tFall;
      //sx1509_write_reg(SX1509_ADDRESS,tempByte);
      sx1509_write_reg(SX1509_ADDRESS,REG_T_FALL[pin],tFall);
    }
}

void blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity)
{
        ledDriverInit(pin);
	//uint8_t offInt = constrain(offInt, 0, 7);

	uint8_t onReg = calculateLEDTRegister(tOn);
	uint8_t offReg = calculateLEDTRegister(tOff);

	setupBlink(pin, onReg, offReg, onIntensity, offIntensity, 0, 0);
}

void breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt)
{
	offInt = constrain(offInt, 0, 7);
        ledDriverInit(pin);

	uint8_t onReg = calculateLEDTRegister(tOn);
	uint8_t offReg = calculateLEDTRegister(tOff);

	uint8_t riseTime = calculateSlopeRegister(rise, onInt, offInt);
	uint8_t fallTime = calculateSlopeRegister(fall, onInt, offInt);

	setupBlink(pin, onReg, offReg, onInt, offInt, riseTime, fallTime);
}
