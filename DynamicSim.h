#ifndef DYNAMICSIM
#define DYNAMICSIM

#include <iomanip>
#include <string>
#include <iostream>
//#include  "ensc-488.h"
#include "traj_plan.h"

#define PI 3.14159265359
#define g 9.81
#define b 5*1000
#define DEG2RAD(x) (x*PI/180.)
#define RAD2DEG(x) (x*180./PI)
#define RES2 30/.1	//RES*(0.1)

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


