/*
    Quaternion.h
    Author: Seb Madgwick

    C++ library for basic usage of quaternions.
    See: http://www.x-io.co.uk/quaternions/
*/

#ifndef Quaternion_h
#define Quaternion_h

//------------------------------------------------------------------------------
// Variable

typedef struct {
    float roll;     /* rotation around x axis in degrees */
    float pitch;    /* rotation around y axis in degrees */
    float yaw;      /* rotation around z axis in degrees */
} EulerAngles;


typedef struct quaternion{
    float q[4];
}Quaternion;

typedef struct matrix{
    float Rotate[3][3];
}QuaterniontMatrix;


//------------------------------------------------------------------------------
// Definitions
void Qua_getConjugate(Quaternion conjugate);

EulerAngles getEulerAngles(Quaternion data);

QuaterniontMatrix quatern2rotMat(Quaternion* data);

float RadtoDegree(float data);

float DegreetoRad(float data);

#endif

//------------------------------------------------------------------------------
// End of file
