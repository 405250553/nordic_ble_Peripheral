#include "LIS2DH12.h"

uint8_t LIS2DH12_who_am_i(){
    uint8_t data_return;
    i2c_data_read_reg(LIS2DH_ADD,WHO_AM_I,&data_return);

    return data_return;
}

/*!
* @brief Returns interrupt status register,
* clearing any pending interrupts.
*/
uint8_t LIS2DH12_IntStatus()
{
	uint8_t u8status;
        i2c_data_read_reg(LIS2DH_ADD,INT1_SRC,&u8status);

        if(u8status){
            //char str_ble_send[5]={0x10,0x20,0x30,0x40,0x50};
            //uint16_t send_str_len = 5; 
            //ble_nus_data_send(&m_nus,str_ble_send,&send_str_len,m_conn_handle);
            nrf_delay_ms(500);
        }
	return u8status;
}


/*!
* @brief Places LIS2DH12 Accelerometer into low-power standby
*/
void LIS2DH12_PowerDown()
{       
    uint8_t power_down[] = {CTRL_REG1,ODR_PDOWN};
    // Set the ODR value to put device into power-down mode.
    i2c_data_write_reg(LIS2DH_ADD,power_down);
}

/*!
* @brief Initialise the LIS2DH12 Accelerometer
*
* Initialises the LIS2DH12 accelerometer into known state.
* After an MCU restart or Power-On-Reset, LIS2 may still be running,
* so first force a reboot to initialise registers and place device
* into power-down mode.
* Allow ample time for the boot process to complete - datasheet says
* reboot takes 'approximately' 5ms.
* Next, place device in low-power mode, high resolution, 1Hz sampling
* on a single axis.
*/
void LIS2DH12_Init (void)
{
    // If this function is called after power on, then the LIS2DH12
    // will still be in boot mode, allow time for boot to complete. 
    nrf_delay_ms(20);
    
    // Disable all interrupt sources
    uint8_t int_clear[] = {INT1_CFG,0};
    // Set the ODR value to put device into power-down mode.
    i2c_data_write_reg(LIS2DH_ADD,int_clear);

    // Put device into power-down
    LIS2DH12_PowerDown();
    nrf_delay_ms(10);
}


void LIS2DH12_EnableSampling_mode (void)
{
    // Enable X,Y,Z sensors and set a default sample rate
    uint8_t ctrl_1[] = {CTRL_REG1, (ODR_200Hz | X_EN | Y_EN | Z_EN) };
    i2c_data_write_reg(LIS2DH_ADD,ctrl_1);
    
    // Disable high-pass filtering
    uint8_t ctrl_2[] = {CTRL_REG2,0};
    i2c_data_write_reg(LIS2DH_ADD,ctrl_2);

    // Enabling FIFO high watermark interrupt
    // vTWI_Write(CTRL_REG3, I1_WTM);
    
    // Selecting 12-bit resolution, range +/- 8g
    uint8_t ctrl_4[] = {CTRL_REG4, (HIRES_MODE | FS_8G ) };
    i2c_data_write_reg(LIS2DH_ADD,ctrl_4);

    // Enable FIFO interrupt, latched on INT1 pin
    // vTWI_Write(CTRL_REG5, (FIFO_EN | LIR_INT1));
    
    // Set BYPASS mode to reset the FIFO
    //vTWI_Write(FIFO_CTRL_REG, BYPASS);
    //uint8_t u8Dummy;
    //vTWI_Read(FIFO_SRC_REG, &u8Dummy);

    // Set FIFO to streaming mode, set level of high watermark and
    // enable interrupt on reaching high watermark
    //vTWI_Write(FIFO_CTRL_REG, (STREAM_MODE | TR_INT1 | FIFO_WATERMARK_30));
}

/*!
* @brief Configures the LIS2DH12 threshold detectors.
*
* Inputs: u8Level - requested acceleration detection threshold
*	  u8Duration - acceleration duration must be sustained for this duration.
*/
void LIS2DH12_EnableThresold_mode (uint8_t u8Level, uint8_t u8Duration)
{   
    // Enable high-pass filtering with highest cut-off frequency
    // and unfiltered samples to the data registers
    //uint8_t ctrl_2[] = {CTRL_REG2, (HPF_CUTOFF_F0 | HP_IA1) };
    uint8_t ctrl_2[] = {CTRL_REG2,0};
    i2c_data_write_reg(LIS2DH_ADD,ctrl_2);
        
    // Enable INT1 interrupts
    uint8_t ctrl_3[] = {CTRL_REG3, I1_IA1 };
    i2c_data_write_reg(LIS2DH_ADD,ctrl_3);
    
    // Select measurement range to +/- 2g, 12-bit resolution
    uint8_t ctrl_4[] = {CTRL_REG4, HIRES_MODE | FS_8G };
    i2c_data_write_reg(LIS2DH_ADD,ctrl_4);
    //eRsolution = mode_12bit;
    //u8SignExtend = 8;

    // Set wake-up threshold level
    uint8_t ctrl_int1[] = {INT1_THS, u8Level };
    i2c_data_write_reg(LIS2DH_ADD,ctrl_int1);
    
    // Set duration that threshold needs to be held
    uint8_t int1_dur[] = {INT1_DURATION, u8Duration };
    i2c_data_write_reg(LIS2DH_ADD,int1_dur);

    // Enable interrupt on INT1 pin
    //boInterruptEvent = false;
    uint8_t ctrl_5[] = {CTRL_REG5, LIR_INT1};
    i2c_data_write_reg(LIS2DH_ADD,ctrl_5);
        
    // Enable threshold event (direction)
    uint8_t int1_cfg[] = {INT1_CFG, 0x7f };
    //uint8_t int1_cfg[] = {INT1_CFG, FUNC_6D };
    i2c_data_write_reg(LIS2DH_ADD,int1_cfg);

    // Enable X,Y,Z sensors and set a default sample rate
    uint8_t ctrl_1[] = {CTRL_REG1, (ODR_200Hz | X_EN | Y_EN | Z_EN) };
    i2c_data_write_reg(LIS2DH_ADD,ctrl_1);
}