/*
    Quaternion.cpp
    Author: Seb Madgwick

    C++ library for basic usage of quaternions.
    See: http://www.x-io.co.uk/quaternions/
*/

//------------------------------------------------------------------------------
// Includes

#include "Quaternion.h"
#include <math.h>

//------------------------------------------------------------------------------
#define rad_to_angle                 57.324f

//------------------------------------------------------------------------------

void Qua_getConjugate(Quaternion conjugate){
    conjugate.q[1] = -conjugate.q[1];
    conjugate.q[2] = -conjugate.q[2];
    conjugate.q[3] = -conjugate.q[3];
}

EulerAngles getEulerAngles(Quaternion data){
    EulerAngles eulerAnglesStruct;
    eulerAnglesStruct.roll = RadtoDegree(atan2(2.0f * (data.q[2] * data.q[3] - data.q[0] * data.q[1]), 2.0f * data.q[0] * data.q[0] - 1.0f + 2.0f * data.q[3] * data.q[3]));
    eulerAnglesStruct.pitch = RadtoDegree(-atan((2.0f * (data.q[1] * data.q[3] + data.q[0] * data.q[2])) / sqrt(1.0f - pow((2.0f * data.q[1] * data.q[3] + 2.0f * data.q[0] * data.q[2]), 2.0f))));
    eulerAnglesStruct.yaw = RadtoDegree(atan2(2.0f * (data.q[1] * data.q[2] - data.q[0] * data.q[3]), 2.0f * data.q[0] * data.q[0] - 1.0f + 2.0f * data.q[1] * data.q[1]));

   return eulerAnglesStruct;
}

QuaterniontMatrix quatern2rotMat(Quaternion* data){
     QuaterniontMatrix mat;

     mat.Rotate[0][0] = 2*(data->q[0]*data->q[0] + data->q[1]*data->q[1])-1;
     mat.Rotate[0][1] = 2*(data->q[1]*data->q[2] - data->q[0]*data->q[3]);
     mat.Rotate[0][2] = 2*(data->q[1]*data->q[3] + data->q[0]*data->q[2]);
     mat.Rotate[1][0] = 2*(data->q[0]*data->q[3] + data->q[1]*data->q[2]);  
     mat.Rotate[1][1] = 2*(data->q[0]*data->q[0] + data->q[2]*data->q[2])-1;
     mat.Rotate[1][2] = 2*(data->q[2]*data->q[3] - data->q[0]*data->q[1]);
     mat.Rotate[2][0] = 2*(data->q[1]*data->q[3] - data->q[0]*data->q[2]); 
     mat.Rotate[2][1] = 2*(data->q[2]*data->q[3] + data->q[0]*data->q[1]); 
     mat.Rotate[2][2] = 2*(data->q[0]*data->q[0] + data->q[3]*data->q[3])-1;
     
   
     return mat;

}


float RadtoDegree(float data)
{
    float re = 0;
    data = (float)(data * rad_to_angle);

    return re;
}

float DegreetoRad(float data)
{
    float re = 0;
    data = (float)(data / rad_to_angle);

    return re;
}


//------------------------------------------------------------------------------
// End of file
