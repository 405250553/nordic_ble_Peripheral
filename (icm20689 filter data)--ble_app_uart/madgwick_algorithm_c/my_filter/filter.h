#ifndef filter_h
#define filter_h

float Filter_A(float NewValue, float FILTER_A);

float FILTER_MOVIMG_AVERAGE(float NewValue,float filter_buf[10]); 

float FILTER_A_MOVIMG_AVERAGE(float NewValue, float FILTER_A,float filter_buff[5]);

float Filter_Phase(float NewValue, float FILTER_A);

float FILTER_MOVIMG_AVERAGE_COE(float NewValue);

float complementary_filters(float angle_m, float gyro_m);

float complementary_filters_second(float angle_m,float gyro_m);

void Kalman_Filter(float *kalman_angle, float P[2][2], float accel_angle, float gyro);

#endif