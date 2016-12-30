#ifndef CONTROLLER
#define CONTROLLER

#include <cmath>
#include <stdio.h>
#include "dynamicSim.h"


class controller
{
	private:
	DynamicSim cmd;
	double* torque;
	double **alpha,**M;
	double *beta;
	double *torque_p;
	double* sln;

	double* betaSln;
	double *V;
	double *G;
	double *F;

	double* torque_pSln;
	double* kp_e;
	double kv;
	double* kv_de;

	//Global Constants
	double* kp;	//orignally taken from textbook

	public:
		controller();
		double* Torque(double * jointDesired, double * velocityDesired, double * accelDesired, double * jointActual, double *  velocityActual);
		double* BetaMatrix(double * jointActual, double *velocityActual);
		double* TorquePrime(double * jointDesired, double * velocityDesired, double * accelDesired, double * jointActual,  double *  velocityActual);
		double** MASSMatrix(double* joint);
};

#endif 