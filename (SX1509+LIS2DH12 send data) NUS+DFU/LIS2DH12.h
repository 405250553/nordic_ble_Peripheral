#include "LIS2DH12_regmap.h"
#include "nrf_drv_twi.h"
#include "i2c_outline.h"

#ifndef LIS2DH12__
#define LIS2DH12__

#ifdef __cplusplus
extern "C" {
#endif

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
void LIS2DH12_Init (void);

/*!
* @brief Configures the LIS2DH12 threshold detectors.
*
* Inputs: u8Level - requested acceleration detection threshold
*	  u8Duration - acceleration duration must be sustained for this duration.
*/
void LIS2DH12_EnableThresold_mode (uint8_t u8Level, uint8_t u8Duration);

/*!
* @brief Returns interrupt status register,
* clearing any pending interrupts.
*/
uint8_t LIS2DH12_IntStatus();

/*!
* @brief Places LIS2DH12 Accelerometer into low-power standby
*/
void LIS2DH12_PowerDown();

uint8_t LIS2DH12_who_am_i();

void LIS2DH12_EnableSampling_mode (void);


#ifdef __cplusplus
}
#endif

#endif //LIS2DH12__