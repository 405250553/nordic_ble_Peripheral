# thingy52_motion_driver_dfu

以thingy52所開發的Project，需先將sx1509 i2c config好，才能使用mpu9250

### 參考

https://blog.csdn.net/suxiang198/article/details/75645542

https://www.liangzl.com/get-article-detail-206738.html

https://github.com/Huffer342-WSH/MPU6050_I2C

https://blog.csdn.net/zh471021698/article/details/88539143

# Motion Driver 6.12 移植

eMPL內均為Motion Driver 6.12原有的文件夾

## 修改

- inv_mpu_dmp_motion_driver.c : 647,648行
```c
    //__no_operation(); //IAR command
	  __ASM("nop"); //KEIL command
```
    
- Motion Driver 6.12本身就支援STM32F4，以`#define EMPL_TARGET_STM32F4`為定義，基於此額外增加 `#define EMPL_TARGET_NRF52`

#### inv_mpu.c
```c
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
```c
#elif defined EMPL_TARGET_NRF52
#include "nrf_delay.h"
//#include "pca20020.h"
#include "app_timer.h"
//#include "thingy_config.h"

#define delay_ms    nrf_delay_ms
```
- 在任一檔案中追加此 global variable

```c
unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";hal_s hal = {0};
```

## 移植

`inv_mpu.c`中定義如i2c func ...，需要此部分的define接自己的func

  ```c
#define i2c_write   drv_mpu9250_write
#define i2c_read    drv_mpu9250_read
#define reg_int_cb  drv_mpu9250_int_register
#define get_ms      drv_mpu9250_ms_get
#define delay_ms    nrf_delay_ms
#define log_i       MPL_LOGI
#define log_e       MPL_LOGE
  ```
 
drv_mpu9250_XXX... 皆為自己的Driver，參考thingy52 project中的`drv_mpu9250.c`,`drv_mpu9250.h`，由於thingy52 的 sx1509 與 mpu9250 共在同一組i2c上，且須先透過 sx1509 將 mpu9250 前置的switch enable，故`drv_mpu9250_write`,`drv_mpu9250_read`內皆是接`sx1509.c`
 
 ## 編譯

編譯器#define：

`NRF_DFU_SVCI_ENABLED` 

`NRF_DFU_TRANSPORT_BLE=1` 

`EMPL REMOVE_LOGGING` 

`MPL_LOG_NDEBUG = 0` 

`USE_DMP` 

`ARM_MATH_CM4` 

`NRF_LOG_USES_RTT=1` 

`MPU9250` 

`EMPL_TARGET_NRF52` 
