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
float G[n][n] = { 0.9881, -0.0141, 0.0380, -0.0263, 0.0585, -0.0346, 
	0.0169, 0.9762, -0.1287, -0.0818, 0.1772, -0.1024, 
	0.0144, 0.0479, 0.7663, -0.4201, -0.1078, -0.1773, 
	-0.0391, 0.0528, 0.2442, 0.7782, 0.0895, -0.4801, 
	-0.0635, -0.0826, 0.2312, -0.0363, 0.8000, 0.1224, 
	0.0345, 0.1490, 0.2556, 0.2840, -0.3122, 0.6145 };

float H[n][m] = { -0.0001, 0.0001, 
	0.0001, -0.0009, 
	-0.0002, -0.0050, 
	-0.0092, -0.0048, 
	-0.0041, 0.0033, 
	-0.0079, 0.0060 };

float C[p][n] = { 2.0579, -122.2165, 5.0683, -0.1801, -4.6082, -4.1844, 
	218.3934, -11.3770, 0.2459, 0.9597, 2.7897, -3.7255 };

float L[n][p] = { -0.0013, 0.0042, 
	-0.0116, -0.0014, 
	-0.0084, 0.0261, 
	0.0077, 0.0085, 
	-0.0006, 0.0019, 
	0.0166, 0.0026 };

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
float Kca[Hp*p][n] = { 0.1950, -119.3514, 17.5090, 6.6544, -24.4752, 8.4902, 
	215.2618, -14.9064, 9.8812, -5.3278, 14.2234, -8.8409, 
	0.0129, -112.0410, 26.9179, 10.8815, -44.6560, 8.1323, 
	211.5896, -19.8849, 17.3999, -15.7643, 22.5574, -8.8056, 
	1.1948, -102.6132, 29.4551, 10.2557, -60.0402, 1.0033, 
	207.8653, -25.5676, 23.0491, -26.8325, 26.3719, -3.4501, 
	3.3131, -93.1284, 24.6994, 6.4341, -68.7122, -6.4155, 
	204.5460, -30.8939, 27.5152, -35.8735, 24.9274, 5.3302, 
	5.9418, -84.7171, 15.0811, 2.8339, -71.3590, -10.4020, 
	201.9889, -34.8829, 31.1987, -41.7164, 18.6021, 14.7577 };

float K1[m][Hp*p] = { 0.0171, -0.0039, -0.0244, 0.0309, -0.0031, 0.0698, 0.0581, 0.0956, 0.1244, 0.1062, 
	0.0221, 0.0045, -0.0198, -0.0004, -0.0857, 0.0293, -0.1343, 0.0721, -0.1503, 0.1084 };

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

