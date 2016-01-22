#include "filter.h"
#include "matrix.h"
#include <stdlib.h>

float LPF(float x, float CUTOFF,float SAMPLE_RATE)
{
  float RC, dt, alpha, y;
  static float ylast=0;
  RC = 1.0/(CUTOFF*2*3.14);
  dt = 1.0/SAMPLE_RATE;
  alpha = dt/(RC+dt);
  y = ylast + alpha * ( x - ylast ); 
  ylast = y;
  return y;
}

float HPF(float x, float CUTOFF,float SAMPLE_RATE)
{
  float RC = 1.0/(CUTOFF*2*3.14);
  float dt = 1.0/SAMPLE_RATE;
  float alpha = RC/(RC+dt);
  float y;
  static float xlast=0, ylast=0;
  y = alpha * ( ylast + x - xlast); 
  ylast = y;
  xlast = x;
  return y;
}

float kalman_single(float z, float measure_noise, float process_noise)
{
 const float R=measure_noise*measure_noise;
 const float Q=process_noise*process_noise; 
 static float x_hat,P;
 float P_,K;
 
 /********* noi suy kalman ***************/
    P_ = P + Q;                     // P_ = A*P*A' + Q;
    K = P_/(P_ + R);                // K = P_*H'*inv(H*P_*H' + R);
    x_hat = x_hat + K*(z - x_hat);  // x_hat = x_hat + K*(z - H*x_hat);
    P = ( 1 - K)*P_ ;               // P = ( 1 - K*H)*P_ ;
 /****************************************/ 
 return x_hat;
}

void kalman(float* in,float* out, float measure_noise, float process_noise)
{
  float R[5][5]={0}, Q[5][5]={0};

  /*const float A[3][3]= {
                          {1,0,0},
                          {0,1,0},
                          {0,0,1},
                          };  
  const float H[3][3]= {
                          {1,0,0},
                          {0,1,0},
                          {0,0,1},
                        }; */
  const float B[5][5]= {
                          {1,0,0,0,0},
                          {0,1,0,0,0},
                          {0,0,1,0,0},
													{0,0,0,1,0},
													{0,0,0,0,1},
                          };
   
  const float I[5][5]= {
                          {1,0,0,0,0},
                          {0,1,0,0,0},
                          {0,0,1,0,0},
													{0,0,0,1,0},
													{0,0,0,0,1},
                          }; 
  static float x_hat[5][1];
  static float P[5][5];
  float z[5][1]; 
  float x_hat_[5][1];
  float P_[5][5];
  float K[5][5];
  float P_R1[5][5],P_R2[5][5];
  float z_x_hat_[5][1],K_z_x_hat_[5][1];
  float I_K[5][5];

  R[0][0] = measure_noise*measure_noise;
  R[1][1] = measure_noise*measure_noise;
  R[2][2] = measure_noise*measure_noise;
	R[3][3] = measure_noise*measure_noise;
	R[4][4] = measure_noise*measure_noise;
  Q[0][0] = process_noise*process_noise;
  Q[1][1] = process_noise*process_noise;
  Q[2][2] = process_noise*process_noise;
	Q[3][3] = process_noise*process_noise;
	Q[4][4] = process_noise*process_noise;
												

matrix_transpose((float*)in,1,5,(float*)z);
//-------------------------------------------------------------- 
// x_hat_ = A*x_hat + B*u_;     <=>  x_hat_ = B*x_hat  
matrix_multiply((float*)B, (float*)x_hat,5,5,1,(float*)x_hat_);
//--------------------------------------------------------------  

//***************************************************************
//P_ = A*P*A' + Q;   <=>  P_ = P + Q
matrix_addition((float*)P,(float*)Q,5,5,(float*)P_);
//***************************************************************

//..............................................................
//K = P_*H'*inv(H*P_*H' + R);   <=>  K = P_*inv(P_ + R)
matrix_addition((float*)P_,(float*)R,5,5,(float*)P_R1);  // P_R1 = P_ + R
matrix_inversion((float*)P_R1,5,(float*)P_R2);  // P_R2 = inv( P_R1)
matrix_multiply((float*)P_, (float*)P_R2,5,5,5,(float*)K);
//..............................................................

//==============================================================
//x_hat = x_hat_ + K*(z - H*x_hat_);    <=>   x_hat = x_hat_ + K*(z - x_hat_)
matrix_subtraction((float*)z,(float*)x_hat_,5,1,(float*)z_x_hat_); // z_x_hat_ = z - x_hat_
matrix_multiply((float*)K, (float*)z_x_hat_,5,5,1,(float*)K_z_x_hat_); // K_z_x_hat_ = K*z_x_hat_
matrix_addition((float*)x_hat_,(float*)K_z_x_hat_,5,1,(float*)x_hat); // x_hat = x_hat_ + K_z_x_hat_
//==============================================================

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//P = ( I - K*H)*P_;    <=>   ( I - K )*P_
matrix_subtraction((float*)I,(float*)K,5,5,(float*)I_K);   // I_K = I - K
matrix_multiply((float*)I_K, (float*)P_,5,5,5,(float*)P);  // P = I_K*P_
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

matrix_transpose((float*)x_hat,5,1,(float*)out);
}  
