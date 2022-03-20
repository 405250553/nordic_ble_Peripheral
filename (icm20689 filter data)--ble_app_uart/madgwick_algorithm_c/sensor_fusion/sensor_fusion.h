//=====================================================================================================
// sensor_fusion.h
//=====================================================================================================
//2019/8/16
//wsua futrack sensor_fusion.h create
//=====================================================================================================

#ifndef sensor_fusion_h
#define sensor_fusion_h

#include "Quaternion.h"
#include <stdint.h>
#include <stdbool.h>

//iir coefficient
#define      b0                      0.9921f
#define      b1                     -0.9921f
#define      a1                     -0.9841f


#define rad_to_angle            57.29577951f


typedef struct {
  float x[2];
  float y[2];
} iir_matrix;

void RadtoAngle(float data[3]);

void AngletoRad(float data[3]);

void velocity_integral(float a[3],float v[3]);

float iir_filter(float v_data,iir_matrix* mat);

void fist_judge(float v_data[3]);

void tilt_acc(float acc_data[3],QuaterniontMatrix* mat,float temp[3]);


#endif

//=====================================================================================================
// End of file
