#include "sensor_fusion.h"
#include "Quaternion.h"
#include <stdbool.h>
#include <stdio.h>

void RadtoAngle(float data[3])
{
    data[0] = (float)(data[0] * rad_to_angle);
    data[1] = (float)(data[1] * rad_to_angle);
    data[2] = (float)(data[2] * rad_to_angle);
}

void AngletoRad(float data[3])
{
    data[0] = (float)(data[0] / rad_to_angle);
    data[1] = (float)(data[1] / rad_to_angle);
    data[2] = (float)(data[2] / rad_to_angle);
}

void velocity_integral(float a[3],float v[3])
{
  static float vo_x = 0;
  static float vo_y = 0;
  static float vo_z = 0;
  float period = 0.01; 

  v[0] = vo_x + a[0]*period;
  v[1] = vo_y + a[1]*period;
  v[2] = vo_z + a[2]*period;

  vo_x = v[0];
  vo_y = v[1];
  vo_z = v[2];
   
}


float iir_filter(float v_data,iir_matrix* mat)
{
   float filter_data = 0;

   mat->x[0] = v_data;
    
   filter_data = b0*mat->x[0] + b1*mat->x[1] - a1*mat->y[0];
   mat->x[1] = mat->x[0];
   mat->y[0] = filter_data;
 
   //printf("%f\r\n",filter_data);

   return filter_data;

}


void fist_judge(float v_data[3])
{
   uint8_t str[20] = {0};
   uint16_t length1 = 0;
   static bool debounce = 0;

   v_data[0] *= 9.8;
   v_data[1] *= 9.8;
   v_data[2] *= 9.8;
   
   if(v_data[0]>2.1&&!debounce)
   {
      //length1 = sprintf(str,"your fist : straight\n");
      //ble_nus_data_send(&m_nus,str,&length1,m_conn_handle);
      printf("your fist : straight\r\n");
      debounce = 1;
   }

   if(v_data[1]>2.1&&!debounce)
   {
      //length1 = sprintf(str,"your fist :  Left Hook\n");
      //ble_nus_data_send(&m_nus,str,&length1,m_conn_handle);
      printf("your fist :  Left Hook\r\n");
      debounce = 1;
   }

   if(v_data[2]>2.1&&!debounce)
   {
      //length1 = sprintf(str,"your fist :  Uppercut\n");
      //ble_nus_data_send(&m_nus,str,&length1,m_conn_handle);
      printf("your fist :  Uppercut\r\n");
      debounce = 1;
   }

   if(v_data[0]<2.1&&v_data[1]<2.1&&v_data[2]<2.1)
       debounce = 0;
       
   //printf("%.4f %.4f %.4f\r\n",v_data[0],v_data[1],v_data[2]);
      
}

void tilt_acc(float acc_data[3],QuaterniontMatrix* mat,float temp[3])
{

  //printf("%.2f %.2f %.2f\n",acc_data[0],acc_data[1],acc_data[2]);

  temp[0] = mat->Rotate[0][0] * acc_data[0] + mat->Rotate[0][1] * acc_data[1] + mat->Rotate[0][2] * acc_data[2];
  temp[1] = mat->Rotate[1][0] * acc_data[0] + mat->Rotate[1][1] * acc_data[1] + mat->Rotate[1][2] * acc_data[2];
  temp[2] = mat->Rotate[2][0] * acc_data[0] + mat->Rotate[2][1] * acc_data[1] + mat->Rotate[2][2] * acc_data[2]-1;

  //printf("%.2f %.2f %.2f\r\n",temp[0],temp[1],temp[2]);

}