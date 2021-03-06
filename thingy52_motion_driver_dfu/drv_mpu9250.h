/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file MPU-9250 motion sensor
 *
 * @defgroup mpu9250_motion_sensor MPU-9250 motion sensor
 * @{
 * @ingroup motion_driver
 * @brief MPU-9250 motion sensor.
 *
 */

#ifndef __DRV_MPU9250_H__
#define __DRV_MPU9250_H__

#include <stdint.h>
#include <stddef.h>
#include "nrf_drv_twi.h"
#include "sx1509.h"
#include "inv_mpu.h"

#define MPU9250_ADD  0X68
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

#define USE_PRINTF_DEBUG 1
#define USE_PYTHON_CLIENT 0
#define MPU_SELF_TEST 0
#define MOTION (0)
#define NO_MOTION (1)

#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define COMPASS_ON (0x04)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ (200)

#define FLASH_SIZE (512)
#define FLASH_MEM_START ((void *)0x1800)

#define PEDO_READ_MS (1000)
#define TEMP_READ_MS (500)
#define COMPASS_READ_MS (100)

/**@brief TWI communication initialization struct.
 */
typedef struct
{
    nrf_drv_twi_t         const * p_twi_instance;
    nrf_drv_twi_config_t  const * p_twi_cfg;
}drv_mpu9250_init_t;


typedef struct
{
    signed char orientation[9];
}platform_data_s;

typedef struct
{
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
}hal_s;

uint8_t mpu9250_who_am_i(void);

/**@brief Function for initializing the MPU-9250 driver.
 *
 * @param[in] p_params      Pointer to the init paramter structure.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_mpu9250_init(drv_mpu9250_init_t * p_params);

/**@brief Function for enabling or disabling the MPU-9250 driver.
 *
 * @param[in] enable    Enables the device.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_mpu9250_enable(bool enable);

/**@brief Function for writing to a MPU-9250 register.
 *
 * @param[in] slave_addr    Slave address on the TWI bus.
 * @param[in] reg_addr      Register address to write.
 * @param[in] length        Length of the data to write.
 * @param[in] p_data        Pointer to the data to write.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * p_data);

/**@brief Function for reading a MPU-9250 register.
 *
 * @param[in]  slave_addr   Slave address on the TWI bus.
 * @param[in]  reg_addr     Register address to read.
 * @param[in]  length       Length of the data to read.
 * @param[out] p_data       Pointer to where the data should be stored.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * p_data);

/**@brief Function for getting a timestamp in milliseconds.
 *
 * @param[out] p_count      Pointer to the timestamp.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_ms_get(unsigned long * p_count);

/**@brief Function for enabling and registering the MPU-9250 interrupt callback.
 *
 * @param[in] p_int_param   Pointer to the interrupt parameter structure.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_int_register(struct int_param_s * p_int_param);

#endif

/** @} */
