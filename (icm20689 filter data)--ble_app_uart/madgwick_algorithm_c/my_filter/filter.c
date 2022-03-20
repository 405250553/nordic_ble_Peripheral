#include "filter.h"
#include <stdio.h>

#define buff_size_1 10
#define buff_size_2 5
#define buff_size_3 12
#define buff_size_4 10
#define          dt 0.005f // dt = 1/ODR

int coe[buff_size_3] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // 
int sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // 

//���T�o�i�k
float Filter_A(float NewValue, float FILTER_A)
{
    static float old_Value = 0;

    if(((NewValue - old_Value) > FILTER_A) || ((old_Value - NewValue) > FILTER_A))
        return old_Value;
    else
       {
        old_Value = NewValue;
        return NewValue;
        }

}


//�ưʥ����o�i�k
float FILTER_MOVIMG_AVERAGE(float NewValue,float filter_buf[10]) 
{
    int i = 0;
    float filter_sum = 0;
    //static float filter_buf[buff_size_1]  = {0};

   for(i = 0; i < buff_size_1-1; i++) {
      filter_buf[i] = filter_buf[i+1];
      filter_sum += filter_buf[i];
    }
  
    filter_buf[buff_size_1-1] = NewValue;
    filter_sum += filter_buf[buff_size_1-1];

    return (float)(filter_sum / buff_size_1);
}


//���T�ưʥ����o�i�k
float FILTER_A_MOVIMG_AVERAGE(float NewValue, float FILTER_A, float filter_buf[5])
{
    int i;
    float filter_sum = 0;
    static float old_Value = 0;
    //float filter_buf[buff_size_2] = {0};

     old_Value = filter_buf[buff_size_2-1];

    if(((NewValue - old_Value) > FILTER_A) || ((old_Value - NewValue) > FILTER_A))  
          return (float)(filter_sum / buff_size_2);
    else{

     for(i = 0; i < buff_size_2-1; i++) {
        filter_buf[i] = filter_buf[i+1];
        filter_sum += filter_buf[i];
       }
  
    filter_buf[buff_size_2-1] = NewValue;
    filter_sum += filter_buf[buff_size_2-1];
    //printf("%f\n",filter_buf[buff_size_2-1]);

    return (float)(filter_sum / buff_size_2);
    }
}

//�@�������o�i�k
float Filter_Phase(float NewValue, float FILTER_A)
{
    static float old_Value = 0;
    float filter_Value = 0;
    filter_Value = NewValue * FILTER_A + (1.0 - FILTER_A) * old_Value;
    return filter_Value;
}


//�[�v�ưʥ����o�i�k
float FILTER_MOVIMG_AVERAGE_COE(float NewValue)
{
    int i;
    float filter_sum = 0;
    static float filter_buf[buff_size_3] = {0};

    for(i = 0; i < buff_size_3-1; i++) {
        filter_buf[i] = filter_buf[i+1];
        filter_sum += filter_buf[i]* coe[i];
      }
  
    filter_buf[buff_size_3-1] = NewValue;
    filter_sum += filter_buf[buff_size_3-1]* coe[buff_size_3-1];

    return (float)(filter_sum / sum_coe);
}


//�@�������o�i
float complementary_filters(float angle_m, float gyro_m)
{
    float K1 = 0.1;
    static float angle1 = 0;

    angle1 = K1 * angle_m+ (1-K1) * (angle1 + gyro_m * dt);

    return angle1;
}

//�G�������o�i
float complementary_filters_second(float angle_m,float gyro_m)
{
    float K2 = 0.2;
    static float angle2 = 0;
    float x1=0,x2=0;
    static float y1 = 0;

    x1=(angle_m-angle2)*(1-K2)*(1-K2);
    y1=y1+x1*dt;
    x2=y1+2*(1-K2)*(angle_m-angle2)+gyro_m;
    angle2=angle2 + x2*dt;

    return angle2;
}


//�d�����o�i
void Kalman_Filter(float *kalman_angle, float P[2][2], float accel_angle, float gyro)
{
  static float Q_angle = 0.001;//���ת��ܲ��ƪ��}��
  static float Q_gyro = 0.005;    //���t�ת��ܲ��ƪ��}��
  static float R_angle = 0.5;    //�[�t�שұo�쪺���ת����ܲ���
  static float gyro_offset = 0; //���t�ת��~�t
	
  float K[2] = { 0 }; // Kalman �Y�� [���׫Y��,���t���Ʋ��Y��]
  float E = 0; //kalman �Y�Ƥ���
  float angle_err = 0; //��ڴ��q����(�Υ[�t�רD�o)- ���״��q�ҫ�(�d���ҲĤ@����)
  float K_temp[2] = { 0 };//kalman �Y�Ƥ��l
  float P_temp[4] = { 0,0,0,0 }; //�p���@�b���@�ܲ��Ưx�}�Ȧs(�d���ҲĤG����)

	
  //�d���ҲĤ@����
  *kalman_angle += (gyro -  gyro_offset) * dt ;

  //�d���ҲĤG����:�p�⥻���ƾڪ��@�ܲ��Ưx�}
  P_temp[0] = Q_angle - P[0][1] - P[1][0] ;
  P_temp[1] = -P[1][1];
  P_temp[2] = -P[1][1];
  P_temp[3] = Q_gyro;

  P[0][0] += P_temp[0] * dt;
  P[0][1] += P_temp[1] * dt;
  P[1][0] += P_temp[2] * dt;
  P[1][1] += P_temp[3] * dt;


  //�d���ҲĤT����:�D�X�d���ҫY��
  K_temp[0] =  P[0][0];
  K_temp[1] =  P[1][0];
  E = R_angle + K_temp[0];
  K[0] = K_temp[0] / E;
  K[1] = K_temp[1] / E;
	

  //�d���Ҳĥ|����:�D�X�o�i��ƾ�
  angle_err = accel_angle - *kalman_angle;
  *kalman_angle += K[0] * angle_err;
  gyro_offset += K[1] * angle_err;  

  // �d���ҲĤ�����:�@�ܲ��Ưx�}����s
  P[0][0] -= P[0][0] * K[0];
  P[0][1] -= P[0][1] * K[0];
  P[1][0] -= P[0][0] * K[1];
  P[1][1] -= P[0][1] * K[1];
	
}

