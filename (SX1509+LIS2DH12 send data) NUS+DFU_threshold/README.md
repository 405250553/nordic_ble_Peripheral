# (SX1509+LIS2DH12 send data) NUS+DFU_threshold

- HW:thingy52
- 已加入 DFU Service
- ble nus send 3軸資料，每十筆資料send一次
- 靜止 `#define INTO_SLEEP` 秒後會停止發送data
- 當單一方向加速度大於設定值後，會觸發Interrupt開始重新發送data
- (P.S.) LIS2DH12_plot.py 可畫圖
- (P.S.) LIS2DH12_threshold_central.hex 為接收端韌體(52DK)，baud rate = 115200
