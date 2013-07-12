/*
 *  Controller library for implementing control algorithms
 *    Uses methods from MatrixMath library by Charlie Matlack
 *
 *  By Jeff Kornuta on March 5, 2013
 */

#include "WProgram.h"
#include "Controller.h"
#include <math.h>

/***********************
* Function Definitions *
***********************/
// Constructor
Controller::Controller(void)
{
}

// Produce new state estimate, xhat_new
void Controller::stateEstim(float u[m][1], float y[p][1], float xhat[n][1])
{
  // Define system parameters and matrices
float G[n][n] = { 0.9966, 0.0023, -0.0826, 0.0210, -0.0068, 0.0040, 
	-0.0029, 0.9851, -0.0401, -0.1784, -0.0007, 0.0202, 
	0.0607, 0.0347, 0.8900, 0.0064, -0.2843, 0.0577, 
	-0.0117, 0.1372, 0.0285, 0.8968, 0.0485, 0.2628, 
	0.0479, 0.0002, 0.3841, -0.0830, 0.8572, 0.0007, 
	-0.0022, -0.0999, -0.0652, -0.3563, 0.0288, 0.8161 };

float H[n][m] = { -0.0003, -0.0002, 
	0.0004, -0.0006, 
	-0.0021, -0.0015, 
	0.0022, -0.0010, 
	0.0065, 0.0046, 
	0.0050, -0.0075 };

float C[p][n] = { 40.6796, -130.0909, 0.9750, 9.5228, -0.7751, 4.8733, 
	140.2680, 18.9972, -4.8522, -0.8581, 1.7648, -0.8506 };

float L[n][p] = { 0.0018, 0.0103, 
	-0.0116, 0.0037, 
	0.0074, -0.0937, 
	0.0604, -0.0110, 
	0.0000, 0.0014, 
	0.0068, -0.0012 };

  //
  // Prediction term
  //
  // Multiply G*xhat
  float Gxhat[n][1];
  MatrixMult((float*)G, (float*)xhat, n, n, 1, (float*)Gxhat);
  //multiply(matarg(G), matarg(xhat), Gxhat);

  // Multiply H*u
  float Hu[n][1];
  MatrixMult((float*)H, (float*)u, n, m, 1, (float*)Hu);
  //multiply(matarg(H), matarg(u), Hu);

  // Initial estimate of new state, xhat_new_est = Gxhat + Hu
  float xhat_new_est[n][1];
  MatrixAdd((float*)Gxhat, (float*)Hu, n, 1, (float*)xhat_new_est);
  //add(matarg(Gxhat), Hu, xhat_new_est);

  //
  // Correction term
  //
  // Multiply C*xhat
  float Cxhat[p][1];
  MatrixMult((float*)C, (float*)xhat, p, n, 1, (float*)Cxhat);
  //multiply(matarg(C), matarg(xhat), Cxhat);

  // Subtract Cxhat from y
  float y_minus_Cxhat[p][1];
  MatrixSubtract((float*)y, (float*)Cxhat, p, 1, (float*)y_minus_Cxhat);
  //subtract(matarg(y), Cxhat, y_minus_Cxhat);

  // Multiply L*(y_minus_Cxhat)
  float xhat_new_correction[n][1];
  MatrixMult((float*)L, (float*)y_minus_Cxhat, n, p, 1, 
      (float*)xhat_new_correction);
  //multiply(matarg(L), matarg(y_minus_Cxhat), xhat_new_correction);

  //
  // New state estimation, xhat_new = xhat_new_est + xhat_new_correction
  //
  float xhat_new[n][1];
  MatrixAdd((float*)xhat_new_est, (float*)xhat_new_correction, n, 1, 
      (float*)xhat_new);
  //add(matarg(xhat_new_est), xhat_new_correction, xhat_new);
  
  // Update previous state estimation with new state estimation
  MatrixCopy((float*)xhat_new, n, 1, (float*)xhat);

}

// Generate system input, u, using a model predictive controller (MPC)
void Controller::mpc(float Ydk[Hp*p][1], float xhat[n][1], float u[m][1])  
{
  // Define gain matrices
float Kca[Hp*p][n] = { 40.8164, -127.2078, 2.3730, 30.9358, -0.5276, 4.0696, 
	139.5391, 18.8367, -15.9597, -1.0943, 1.8659, -0.2494, 
	40.7924, -121.2996, 4.2476, 49.9029, 0.2992, 9.1799, 
	138.1453, 18.1990, -25.7855, -1.5859, 5.1213, -0.4680, 
	40.6715, -113.3215, 6.2057, 63.9812, 1.5379, 18.5624, 
	136.3239, 17.1818, -33.1388, -2.1970, 10.6850, -1.3590, 
	40.5204, -104.3995, 7.9034, 71.7480, 2.9913, 30.1929, 
	134.3408, 15.9269, -37.3142, -2.7928, 17.5026, -2.6953, 
	40.3997, -95.6473, 9.0903, 72.8648, 4.4611, 42.0038, 
	132.4511, 14.5944, -38.1274, -3.2609, 24.4807, -4.2123 };

float K1[m][Hp*p] = { -0.0203, -0.0153, 0.0144, 0.0090, 0.0582, 0.0395, 0.1002, 0.0703, 0.1323, 0.0963, 
	0.0186, -0.0124, 0.0030, 0.0011, -0.0356, 0.0283, -0.0823, 0.0615, -0.1245, 0.0932 };

  // Multiply Kca*xhat
  float Kcaxhat[Hp*p][1];
  MatrixMult((float*)Kca, (float*)xhat, Hp*p, n, 1, (float*)Kcaxhat);
  //multiply(matarg(Kca), matarg(xhat), Kcaxhat);

  // Subtract Kcaxhat from Ydk
  float Ydk_minus_Kcaxhat[Hp*p][1];
  MatrixSubtract((float*)Ydk, (float*)Kcaxhat, Hp*p, 1, 
      (float*)Ydk_minus_Kcaxhat);
  //subtract(matarg(Ydk), Kcaxhat, Ydk_minus_Kcaxhat);

  // Multiply K1*Ydk_minus_Kcaxhat ( = u )
  MatrixMult((float*)K1, (float*)Ydk_minus_Kcaxhat, m, Hp*p, 1, (float*)u);
  //multiply(matarg(K1), matarg(Ydk_minus_Kcaxhat), u);

  // Saturation checking: Servo library has built-in 
  //  +-10 V saturation in move() method.
}

// Estimate initial condition, xhat(k=0), method
void Controller::stateInit(float y[p][1], float xhat[n][1])
{
  float norm_error = 1;
  float u_zero[m][1] = {0, 0};
  // Iterate estimator until norm_error is small
  while ( norm_error > 1e-5 )
  {
    // Copy over xhat to xhat_old
    float xhat_old[n][1];
    MatrixCopy((float*)xhat, n, 1, (float*)xhat_old);

    // Estimate new state
    stateEstim(u_zero, y, xhat);

    // Calculate error (= xhat - xhat_old)
    float error[n][1];
    MatrixSubtract((float*)xhat, (float*)xhat_old, n, 1, (float*)error);

    // Calculate norm of error
    norm_error = VectorNorm((float*)error, n);
  }
}

// Filter method for a moving average filter
float Controller::filter(float y, float Y[max_samples], int order)
{
  // Update vector, Y, of sample histories (zero is current)
  for (uint8_t i = order; i > 0; i = i-1)
  {
    // Shift everything downward
    Y[i] = Y[i-1];
  }
  Y[0] = y;

  // Moving average = (sum of samples) / (# samples)
  float average = 0;
  for (uint8_t i = 0; i <= order; i++)
  {
    average += Y[i];
  }
  average = average/((float)order + 1);

  return average;
}

// Vector norm method
float Controller::VectorNorm(float* Vector, int size)
{
  float squares = 0.0;
  // Sum the squares
  for (uint16_t i = 0; i < size; i++)
  {
    squares += Vector[i]*Vector[i];
  }

  // norm = sqrt(sum_of_the_squares)
  float norm = sqrt(squares);

  return norm;
}

// Methods from MatrixMath library

// Copy matrix
void Controller::MatrixCopy(float* A, int rowsA, int colsA, float* B)
{
	int i, j, k;
	for (i=0;i<rowsA;i++)
		for(j=0;j<colsA;j++)
		{
			B[colsA*i+j] = A[colsA*i+j];
		}
}

//Matrix Multiplication Routine
// C = A*B
void Controller::MatrixMult(float* A, float* B, int rowsA, int colsA, int colsB, float* C)
{
	// C = output matrix = A*B (rowsA x colsB)
	int i, j, k;
	for (i=0;i<rowsA;i++)
		for(j=0;j<colsB;j++)
		{
			C[colsB*i+j]=0;
			for (k=0;k<colsA;k++)
				C[colsB*i+j]= C[colsB*i+j]+A[colsA*i+k]*B[colsB*k+j];
		}
}

//Matrix Addition Routine
void Controller::MatrixAdd(float* A, float* B, int rowsA, int colsA, float* C)
{
	// C = output matrix = A+B
	int i, j;
	for (i=0;i<rowsA;i++)
		for(j=0;j<colsA;j++)
			C[colsA*i+j]=A[colsA*i+j]+B[colsA*i+j];
}


//Matrix Subtraction Routine
void Controller::MatrixSubtract(float* A, float* B, int rowsA, int colsA, float* C)
{
	// C = output matrix = A-B 
  int i, j;
	for (i=0;i<rowsA;i++)
		for(j=0;j<colsA;j++)
			C[colsA*i+j]=A[colsA*i+j]-B[colsA*i+j];
}


//Matrix Transpose Routine
void Controller::MatrixTranspose(float* A, int rowsA, int colsA, float* C)
{
	// C = output matrix = the transpose of A (colsA x rowsA)
	int i, j;
	for (i=0;i<rowsA;i++)
		for(j=0;j<colsA;j++)
			C[rowsA*j+i]=A[colsA*i+j];
}

