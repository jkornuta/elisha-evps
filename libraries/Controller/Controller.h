/*
 *  Controller library for implementing control algorithms
 *    Uses methods from MatrixMath library by Charlie Matlack
 *
 *  By Jeff Kornuta on March 5, 2013
 */

#ifndef Controller_h
#define Controller_h

#include "WProgram.h"

/********************
* Define Statements *
********************/
#define rowsof(MATRIX) (sizeof (MATRIX) / sizeof *(MATRIX))
#define colsof(MATRIX) (sizeof *(MATRIX) / sizeof **(MATRIX))
#define matarg(MATRIX) rowsof(MATRIX), colsof(MATRIX), (MATRIX)
#define n 6 // Number of states
#define m 2 // Number of inputs
#define p 2 // Number of outputs
#define Hp 5 // Size of prediction horizon
#define max_samples 10 // Maximum size of sample history for filter

/********************
* Class Declaration * 
********************/
class Controller
{
  public:
	  Controller();
	  void stateEstim(float u[m][1], float y[p][1], float xhat[n][1]);
    void mpc(float Ydk[Hp*m][1], float xhat[n][1], float u[m][1]);
    void stateInit(float y[p][1], float xhat[n][1]);
    float filter(float y, float Y[max_samples], int order);
    float VectorNorm(float* Vector, int size);
  
    // Methods taken from MatrixMath library
    void MatrixCopy(float* A, int rowsA, int colsA, float* B);
	  void MatrixMult(float* A, float* B, int rowsA, int colsA, int colsB, float* C);
	  void MatrixAdd(float* A, float* B, int rowsA, int colsA, float* C);
  	void MatrixSubtract(float* A, float* B, int rowsA, int colsA, float* C);
	  void MatrixTranspose(float* A, int rowsA, int colsA, float* C);

};

#endif
