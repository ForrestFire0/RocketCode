#include "Arduino.h"
#include "KF.h"

KalmanFilter::KalmanFilter(float X_k[][], float H[][]], float[][] Q, float[][] R)
{
    this->X_k = X_k;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->H_t = transpose(H);
}

void KalmanFilter::predict(float acceleration, float delta_t)
{
    float F[][] = {{1, delta_t},
                   {0, 1}};
    float u[][] = {{acceleration}};
    float B[][] = {{.5 * delta_t * *2},
                   {delta_t}};

    //Predicted state estimate.
    this->X_k = add(multiply(F, this->X_k), multiply(B, u));

    //Serial.println("Change in state based on F: \n" + str(matmul(F, X_k)))
    //Serial.println("Change in state based on u: \n" + str(matmul(B, u)))
    //Serial.println("After predict state: \n" + str(a_priori_X_k))
    this->.P_k = add(multiply(F, P_k, transpose(F)), Q)
    //Serial.print("After predict covariance: \n" + str(a_priori_covariance))
}

void KalmanFilter::update(float gps_location)
{
    float Z[][] = {{gps_location}};

    float y[][] = Z - multiply(this->H, this->X_k);

    float PHT[][] = multiply(this->P_k, this->H_t);

    float S[][] = add(multiply(this->H, PHT), this->R);

    S = inv(S);

    float K[][] = multiply(PHT, S);

    //Updated state
    this->X_k = add(this->X_k, multiply(K @y));
    float I_KH[][] = {{1, 0}, {0, 1}} - multiply(K, this->H);

    this->P_k = add(multiply(I_KH, this->P_k, I_KH.T), multiply(K, this->R, transpose(K)));
}

float *KalmanFilter::getState()
{
    return this->X_k;
}

float *KalmanFilter::multiply(float a[][], float b[][])
{
    int a_rows = sizeof(a) / sizeof(a[0]);
    int a_cols = sizeof(a[0]) / sizeof(a[0][0]);

    int b_rows = sizeof(b) / sizeof(b[0]);
    int b_cols = sizeof(b[0]) / sizeof(b[0][0]);

    assert(a_cols == a_rows);
    float mul[a_rows][b_cols];

    for (i = 0; i < a_rows; i++)
    {
        for (j = 0; j < b_cols; j++)
        {
            mul[i][j] = 0;
            for (k = 0; k < b_cols; k++)
            {
                mul[i][j] += a[i][k] * b[k][j];
            }
        }
    }

    return mul;
}

float *KalmanFilter::multiply(float a[][], float b[][], float c[][])
{
    return multiply(multiply(a, b), c);
}

float *KalmanFilter::add(float a[][], float b[][])
{
    int a_rows = sizeof(a) / sizeof(a[0]);
    int a_cols = sizeof(a[0]) / sizeof(a[0][0]);

    int b_rows = sizeof(b) / sizeof(b[0]);
    int b_cols = sizeof(b[0]) / sizeof(b[0][0]);

    assert(a_rows == b_rows);
    assert(a_cols == b_cols);

    float product[a_rows][b_cols];

    for (i = 0; i < a_rows; i++)
    {
        for (j = 0; j < b_cols; j++)
        {
            product[i][j] = a[i][j] + b[i][j];
        }
    }

    return product;
}

float *KalmanFilter::transpose(float a[][])
{
    int a_rows = sizeof(a) / sizeof(a[0]);
    int a_cols = sizeof(a[0]) / sizeof(a[0][0]);

    float tran[a_cols][a_rows];

    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < column; ++j)
        {
            tran[j][i] = a[i][j];
        }
    }

    return tran;
}
float *KalmanFilter::inv(float a[][])
{
    int a_rows = sizeof(a) / sizeof(a[0]);
    int a_cols = sizeof(a[0]) / sizeof(a[0][0]);

    if (a_rows == 1 and a_cols == 1)
    {
        return {{a[0][0]}};
    }
    else
    {
        Serial.println(F("Error! a matrix needing inversing was not 1X1."))
    }
    
}