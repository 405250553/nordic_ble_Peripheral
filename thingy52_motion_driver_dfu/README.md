# thingy52_motion_driver_dfu

以thingy52所開發的Project，

https://blog.csdn.net/suxiang198/article/details/75645542

https://www.liangzl.com/get-article-detail-206738.html

https://github.com/Huffer342-WSH/MPU6050_I2C

https://blog.csdn.net/zh471021698/article/details/88539143

# Motion Driver 6.12 移植

eMPL內均為Motion Driver 6.12原有的文件夾

## 修改
- Motion Driver 6.12本身就支援STM32F4，以`#define EMPL_TARGET_STM32F4`為定義，基於此額外增加 `#define EMPL_TARGET_NRF52`

#### inv_mpu.c
```
#elif defined EMPL_TARGET_NRF52
#include "nrf_delay.h"
#include "log.h"
#include "../../drv_mpu9250.h"

#define i2c_write   drv_mpu9250_write
#define i2c_read    drv_mpu9250_read
#define reg_int_cb  drv_mpu9250_int_register
#define get_ms      drv_mpu9250_ms_get
#define delay_ms    nrf_delay_ms
#define log_i       MPL_LOGI
#define log_e       MPL_LOGE
#define min(a,b) ((a<b)?a:b)   
#else
#error  Gyro driver is missing the system layer implementations.
#endif
```

#### inv_mpu_dmp_motion_driver.c
```
#elif defined EMPL_TARGET_NRF52
#include "nrf_delay.h"
//#include "pca20020.h"
#include "app_timer.h"
//#include "thingy_config.h"

#define delay_ms    nrf_delay_ms
```

## 移植
综上，实际上需要提供的只有

- i2c_write(unsigned char slave_addr, unsigned char reg_addr,*unsigned char length, unsigned char const \*data)*

- i2c_read(unsigned char slave_addr, unsigned char reg_addr,*unsigned char length, unsigned char \*data)*

- delay_ms(unsigned long num_ms)

- get_ms(unsigned long \*count)

  该部分具体代码如下

  ```c
  #define i2c_write Sensors_I2C_WriteRegister
  
  #define i2c_read Sensors_I2C_ReadRegister
  
  #define delay_ms HAL_Delay
  
  #define get_ms get_ms_user
  ```
