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

#include "drv_mpu9250.h"
#include "nrf_error.h"
#include "nrf_drv_gpiote.h"
//#include "pca20020.h"
#include "app_timer.h"
//#include "thingy_config.h"
#include "mltypes.h"
#include "app_scheduler.h"
//#include "twi_manager.h"

#define  NRF_LOG_MODULE_NAME "drv_mpu9250   "
//#include "nrf_log.h"
//#include "macros_common.h"

static struct
{
    drv_mpu9250_init_t        init;             ///< TWI configuration.
    void                      (*cb)(void);      ///< Callback. Invoked when a pin interrupt is caught by GPIOTE.
    bool                      initialized;      ///< Module initialized.
    bool                      int_registered;   ///<
    bool                      enabled;          ///< Driver enabled.
    uint32_t                  evt_sheduled;     ///< Number of scheduled events.
} m_mpu9250 = {.initialized = false, .int_registered = false};


int drv_mpu9250_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * p_data)
{
		/*
    uint32_t err_code;
    uint8_t buffer[length+1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], p_data, length);

    err_code = twi_open();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_tx( m_mpu9250.init.p_twi_instance,
                               slave_addr,
                               buffer,
                               length + 1,
                               false);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("drv_mpu9250_write Failed!\r\n");
    }

    err_code = twi_close();
    APP_ERROR_CHECK(err_code);

    return 0;
		*/
		
		sx1509_write_bytes(slave_addr,reg_addr,p_data,length);
		
		return 0;
}


int drv_mpu9250_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * data)
{
		/*
    uint32_t err_code;

    err_code = twi_open();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_tx( m_mpu9250.init.p_twi_instance,
                               slave_addr,
                               &reg_addr,
                               1,
                               true );
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("drv_mpu9250_read Failed!\r\n");
    }

    err_code = nrf_drv_twi_rx( m_mpu9250.init.p_twi_instance,
                               slave_addr,
                               data,
                               length );
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("drv_mpu9250_read Failed!\r\n");
    }

    err_code = twi_close();
    APP_ERROR_CHECK(err_code);

    return 0;
		*/
		
		sx1509_data_read_bytes(slave_addr,reg_addr,length,data);
		
		return 0;
}

int drv_mpu9250_int_register(struct int_param_s *int_param)
{
    if (!m_mpu9250.int_registered)
    {
        m_mpu9250.int_registered = true;
        m_mpu9250.cb = int_param->cb;
    }

    return 0;
}


int drv_mpu9250_ms_get(unsigned long * count)
{
    uint32_t ticks;

    ticks = app_timer_cnt_get();

    *count = (ticks * (APP_TIMER_PRESCALER + 1) * 1000) / APP_TIMER_CLOCK_FREQ;

    return 0;
}

uint32_t drv_mpu9250_init(drv_mpu9250_init_t * p_params)
{
    //NULL_PARAM_CHECK(p_params);

    m_mpu9250.init.p_twi_cfg      = p_params->p_twi_cfg;
    m_mpu9250.init.p_twi_instance = p_params->p_twi_instance;
    m_mpu9250.initialized = true;
    m_mpu9250.enabled     = false;
    m_mpu9250.evt_sheduled = 0;

    return NRF_SUCCESS;
}

