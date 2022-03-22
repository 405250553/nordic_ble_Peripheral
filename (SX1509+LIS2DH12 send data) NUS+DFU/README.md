# (SX1509+LIS2DH12 send data) NUS+DFU

- HW:thingy52
- ble nus send 3軸資料，每十筆資料send一次
- 已加入 DFU Service
- sdk_config.h `#define NRF_SDH_CLOCK_LF_SRC 0` (ble clock 改為內部低頻震盪器)
- (P.S.) LIS2DH12_plot.py 可畫圖
- (P.S.) LIS2DH12_accData_central.hex 為接收端韌體(52DK)，baud rate = 115200
