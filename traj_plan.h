#ifndef TRAJ_PLAN
#define TRAJ_PLAN
//This class handles commanding the emulator by using the previously developed matrix operations

#pragma once


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>


#include "mat_kin.h"

class traj_plan
{
private:
	double** Output;
	double** spline;
	//double** coeff;
	double* coeff;
	double** discreteSpline;
	double** temp; //temperary variable to store spline results before moving it into 'spline
	double** temp1;
	double* coefficient; // this is not the same value a coeff
	double* tau;
	double* time;
	double* joint;

	double t;
	int ii, via0, via1, via2, via3;
	int points;

	double h_0, h_1, h_2, h_3;
	double h_4; //time btw. last via and goal?
	double delta0, delta1, delta2, delta3;
	double v1, v2, v3, v4;
	double deltaT;
	double total_time;
	double realT; //Value in real time
	int vi_1;
	int steps;

public:
	traj_plan();
	double** discreteTrajectory(double** viaPoints, int numVia, int numPoints, int jointNUM);
	void traj_plan::printTrajectory(string filename, double** jointValues, int steps);
	double** traj_plan::calcSpline(double* joint, int numVia, /*int i,*/ double *time, int numPoints);
	double* traj_plan::calcCoeff(double yi, double yi_1, double vi, double vi_1, double hi);
	double** traj_plan::calcDiscreteSpline(double taui, double taui_1, double* coeff, double hi);
	void traj_plan::appPrintTrajectory(string filename, double input);
	//void traj_plan::printResults(double*** jointValues, int steps, int i, int j, int tempCounter);
	//int traj_plan::timeSampling()
	void traj_plan::printM(double** matrix, int height, int width); //Kara: please remove once code is complete
	int traj_plan::numofPoints(double t1, double t2, double t3, double t4);
};

#endif 