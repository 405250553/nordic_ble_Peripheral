//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include "Quaternion.h"
#include "sensor_fusion.h"
//#include "filter.h"
#include <math.h>
#include <stdio.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	 200.0f	        // sample frequency in Hz
#define twoKpDef	(2.0f * 1.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 1.0f)	// 2 * integral gain
#define rad_to_deg  57.324f

#define betaDef		0.1f		
volatile float beta = betaDef;	
//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;
//volatile float twoKi = 0.0f;												// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
float reset_q[4] = {0};
//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

typedef struct {
  float x[3];
  float y[3];
} iir_matrix_1;

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float angle_data[3] = {0};

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az,angle_data);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;

                halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
                halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
                halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


float iir_filter_1(float v_data,iir_matrix_1* mat)
{
   float filter_data = 0;

   mat->x[0] = v_data;
    
   //filter_data = 0.9996*mat->x[0] - 1.9991*mat->x[1] + 0.9996*mat->x[2] + 1.9991*mat->y[0] - 0.9991*mat->y[1];
   filter_data = 0.99993336*mat->x[0] - 1.99986672*mat->x[1] + 0.999993336*mat->x[2] + 1.99986671*mat->y[0] - 0.99986672*mat->y[1];
   mat->x[2] = mat->x[1];
   mat->x[1] = mat->x[0];
   mat->y[1] = mat->y[0];
   mat->y[0] = filter_data;
 
   //printf("%f\r\n",filter_data);

   return filter_data;

}


float iir_filter_2(float v_data,iir_matrix* mat)
{
   float filter_data = 0;

   mat->x[0] = v_data;
    
   filter_data = 0.9984*mat->x[0] - 0.9984*mat->x[1] + 0.9969*mat->y[0];
   mat->x[1] = mat->x[0];
   mat->y[0] = filter_data;
 
   //printf("%f\r\n",filter_data);

   return filter_data;

}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

Quaternion MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float data[3]) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float Yaw,Pitch,Roll;
        Quaternion ahrs_quat;

        float acc[3] = {0};
        static iir_matrix_1 Q3;
        static iir_matrix Q3_2;
        float acc_sqrt = 0;
        static int state = 0;
        static float buff_q0[5] = {0};
        static float buff_q1[5] = {0};
        static float buff_q2[5] = {0};
        static float buff_q3[5] = {0};
        static int buff_time = 0;
        float g = 0;
        

        acc[0] = ax;
        acc[1] = ay;
        acc[2] = az;

        g = gz;

        acc_sqrt = sqrt(pow(acc[0],2)+pow(acc[1],2)+pow(acc[2],2));
	

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
                halfvx = 2*(q1 * q3 - q0 * q2);
		halfvy = 2*(q0 * q1 + q2 * q3);
		//halfvz = q0 * q0 - q1*q1 - q2*q2 + q3 * q3;
                halfvz = 1- 2*(q1*q1 + q2*q2);



		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;


        //q3 = iir_filter_1(q3,&Q3);
        //q3 = iir_filter_2(q3,&Q3_2);  
          //g = iir_filter(q3,&Q3_2);
	
        ahrs_quat.q[0] = q0;
        ahrs_quat.q[1] = q1;
        ahrs_quat.q[2] = q2;
        ahrs_quat.q[3] = q3;
	
	//printf("%.3f %.3f %.3f\n",q1,q2,q3);
	      
         // earth euler
         Pitch = atan(halfvx /sqrt(pow(halfvy,2) + pow(halfvz,2)))* rad_to_angle;
         Roll = atan(halfvy /sqrt(pow(halfvx,2) + pow(halfvz,2)))* rad_to_angle;
         //Yaw = atan(sqrt(pow(halfvx,2)+pow(halfvy,2))/halfvz)*rad_to_angle;
         Yaw = (atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2*q2 - q3*q3))* rad_to_deg;
         Roll = (atan2(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2*q2 + q3*q3))* rad_to_deg;


       /*
       // earth euler
        Yaw = (atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2*q2 - q3*q3))* rad_to_deg;
        Pitch = -(asin((-2.0f * (q1 * q3 - q0 * q2))))* rad_to_deg;
        Roll = (atan2(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2*q2 + q3*q3))* rad_to_deg;
       */
        
        /*
        Yaw = atan2(2.0f * (q1 * q2 + q0 * q3), 1- 2*(q2*q2 + q3*q3))* rad_to_deg;
        Pitch = -(asin((2.0f * (q1 * q3 - q0 * q2))))* rad_to_deg;
        Roll = atan2(2.0f * (q2 * q3 + q0 * q1), 1- 2*(q1 * q1 + q2*q2))* rad_to_deg;
*/

        //if(state%20==1)
           //printf("%.3f %.3f %.3f %.3f %.3f\n",g,q0,q1,q2,q3);

         //printf("w%.3fwa%.3fab%.3fbc%.3fc\n",q0,q1,q2,q3);
         // printf("y%.3fyp%.3fpr%.3fr\n",Yaw,Pitch,Roll);
        //printf("y%.3fyp%.3fpr%.3fr\n",Yaw,Pitch,Roll);

           /*
        if(Pitch>=73 && acc_sqrt<2.22){
             state += 1;
             }
             else
             {
               state = 0;
             }
        
        if(state>=100){
            if(buff_time<5)
            {
               buff_q0[buff_time] = q0;
               buff_q1[buff_time] = q1;
               buff_q2[buff_time] = q2;
               buff_q3[buff_time] = q3;
               buff_time+=1;
            }
            else if(buff_time<10)
            {
            //printf("static posture\r\n");
            //printf("%.3f %.3f %.3f %.3f\n",q0,q1,q2,q3);
            reset_q[0] = FILTER_A_MOVIMG_AVERAGE(q0,0.1,buff_q0);
            reset_q[1] = FILTER_A_MOVIMG_AVERAGE(q1,0.1,buff_q1);
            reset_q[2] = FILTER_A_MOVIMG_AVERAGE(q2,0.1,buff_q2);
            reset_q[3] = FILTER_A_MOVIMG_AVERAGE(q3,0.1,buff_q3);
            //printf("other:%.3f %.3f %.3f %.3f\n",reset_q[0],reset_q[1],reset_q[2],reset_q[3]);
           buff_time+=1;
            }
       else
       {
       if(fabs(q0-reset_q[0])>0.15||fabs(q2-reset_q[2])>0.15)
            {
               q0 = reset_q[0];
               q1 = reset_q[1];
               q2 = reset_q[2];
               q3 = reset_q[3];
               //printf("reset\n");
            }
            
         }
            
            state = 0;
        }
        */
        
		
	data[0] = Pitch;
	data[1] = Roll;
	data[2] = Yaw;

        return ahrs_quat;
				
				
}

Quaternion MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float data[3]) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float Yaw,Pitch,Roll;
        Quaternion ahrs_quat;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

        ahrs_quat.q[0] = q0;
        ahrs_quat.q[1] = q1;
        ahrs_quat.q[2] = q2;
        ahrs_quat.q[3] = q3;
	
	//printf("%.2f %.2f %.2f %.2f\r\n",q0,q1,q2,q3);

        printf("w%.3fwa%.3fab%.3fbc%.3fc\n",q0,q1,q2,q3);
	
	
   // linear acc is very close   
/*
     Yaw   = atan2(2.0 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)* rad_to_deg;
    Pitch = -sin(2.0 * (q1 * q3 - q0 * q2))* rad_to_deg;
    Roll  = atan2(2.0 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3+1)* rad_to_deg;
*/
    /*

 	Yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* rad_to_deg; // yaw 
        Pitch= asin(-2 * q1 * q3 + 2 * q0* q2)* rad_to_deg; // pitch 
        Roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* rad_to_deg; // roll
*/
	
        
	// 3d cube is correct
        Roll = (atan2(2.0f * (q2 * q3 - q0 * q1), 2.0f * q0 * q0 - 1.0f + 2.0f * q3 * q3))* rad_to_deg;
        Pitch = (-atan((2.0f * (q1 * q3 + q0 * q2)) / sqrt(1.0f - pow((2.0f * q1 * q3 + 2.0f * q0 * q2), 2.0f))))* rad_to_deg;
        Yaw = (atan2(2.0f * (q1 * q2 - q0 * q3), 2.0f * q0 * q0 - 1.0f + 2.0f * q1 * q1))* rad_to_deg;
	//Roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* rad_to_deg; // roll	

       //Yaw -= -0.13;
		
	data[0] = Pitch;
	data[1] = Roll;
	data[2] = Yaw;

        return ahrs_quat;
	
}


Quaternion MahonyAHRSupdateIMU_original(float gx, float gy, float gz, float ax, float ay, float az, float data[3]) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
        Quaternion ahrs_quat;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

        printf("w%.3fwa%.3fab%.3fbc%.3fc\n",q0,q1,q2,q3);

        ahrs_quat.q[0] = q0;
        ahrs_quat.q[1] = q1;
        ahrs_quat.q[2] = q2;
        ahrs_quat.q[3] = q3;

        return ahrs_quat;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
