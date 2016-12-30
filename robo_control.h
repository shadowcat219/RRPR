#ifndef ROBO_CONTROL
#define ROBO_CONTROL
//This class handles commanding the emulator by using the previously developed matrix operations

#pragma once


#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <ctime>
#include "mat_kin.h"
#include "ensc-488.h"
#include "traj_plan.h"
#include "DynamicSim.h"
#include "controller.h"


#define HEIGHT 4
#define WIDTH 4
//#define PI 3.14159265359

class robo_control
{
private:
	mat_kin cmd;
	traj_plan cmd2;
	DynamicSim cmd3;
	controller cmd4;
	JOINT q0;
	JOINT *q1, *q2, *q3;
	double j1, j2, j3, j4;
	double x, y, z, theta;
	double** internal_form;
	double* user_form;
	double** t_matrix1;
	double** t_matrix2;
	double** r_matrix;
	double* vect_5;
	char grip;
	double** via;

	double*** traj;
	double** traj_joint0;
	double** traj_joint1;
	double** traj_joint2;
	double** traj_joint3;

	int ii;


	double* jointDesired;
	double* velocityDesired;
	double*	accelDesired;

	double* jointActual;
	double* velocityActual;

	double * accelActual;
	double **	next;

	double* torque;


public:
	robo_control();
	void zeroPosition();
	void moveJOINT();
	void currentJoints();
	void currentCartesian();
	void moveCart();
	void moveWithVelAcc();
	void initJoint();
	void stopRobot();
	void moveGriper();
	void trajectoryPlan();								//KARA
	void moveWithVelAcc(JOINT& q1, JOINT& q2, JOINT& q3);
	void RESETROBOT();
	void controlledTrajectory();
	void dynamicSim();


};

#endif 