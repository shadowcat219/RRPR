#ifndef DYNAMICSIM
#define DYNAMICSIM

#include <iomanip>
#include <string>
#include <iostream>
//#include  "ensc-488.h"
#include "traj_plan.h"


using namespace std;

class DynamicSim
{
private:
	//double *torque;
	//double *velocity;
	double *Accel;
	//double *joint;
	double **nextValues;
	double **MInv, **M;
	double *V;

public:
	DynamicSim();
	double* DynamicAcceleration(double *joint, double *velocity, double *torque);
	double** NumericalIntegration(double *joint, double *velocity, double *accel, double *torque);
	double* Vmatrix(double* joint, double * velocity);
	double** invMassMatrix(double* joint);
	double** massMatrix(double* joint);
};

#endif


