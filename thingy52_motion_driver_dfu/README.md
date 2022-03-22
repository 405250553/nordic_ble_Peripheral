# thingy52_motion_driver_dfu

以thingy52所開發的Project，

https://blog.csdn.net/suxiang198/article/details/75645542

https://www.liangzl.com/get-article-detail-206738.html

https://github.com/Huffer342-WSH/MPU6050_I2C

https://blog.csdn.net/zh471021698/article/details/88539143

# Motion Driver 6.12 移植

eMPL內均為Motion Driver 6.12原有的文件夾

- Motion Driver 6.12本身就支援STM32F4，以`#define EMPL_TARGET_STM32F4`為定義，基於此額外增加 `#define EMPL_TARGET_NRF52`
