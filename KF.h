
#include "Arduino.h"
#pragma once


class KalmanFilter

{
    public:
        KalmanFilter(float X_k[][], float H[][], float[][] Q, float[][] R);
        float* getState();
        void predict(float acceleration, float delta_t);
        void update(float gps_location);
    private:
        float X_k[2][1];
        float P_k[2][2];
        float H[1][2];
        float H_t[2][1]
        float Q[2][2];
        float R[1][1];
        float* multiply(float a[][], float b[][]);
        float* multiply(float a[][], float b[][], float c[][]);
        float* add(float a[][], float b[][]);
        float* transpose(float a[][]);
        float* inv(float a[][]);
}
