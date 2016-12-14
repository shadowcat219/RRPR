#ifndef MIN_KIN
#define MIN_KIN

#include <cmath>
#include <string>
#include <iostream>

#include <iomanip>
//#include  "ensc-488.h"

#define HEIGHT 4
#define WIDTH 4
#define PI 3.14159265359

using namespace std;

class mat_kin
{
private:
	double** r_matrix;
	double point[3];
	double user_form[3];
	double* user_form_ptr;
	double x, y, z, theta, the1, the2, the4, d3, a, b, c, c12;
	double** internal_form;
	double temp;
	double px, py, r01, r11;
	double* joints;
	double* theta1;
	double* theta2;
	double* theta4;
	double** internal_joints;

public:
	mat_kin();
	double** UTOI(double* user_form);
	double* ITOU(double** internal_form);
	double** TMULT(double** t_matrix1, double** t_matrix2);
	double** TINVERT(double** internal_form);
	double** WHERE(double* joint);
	double** SOLVE(double* oldJoints, double** Tmatrix);
	//double* findTheta1(double px, double py, double oldJoint1);
	//double* findTheta2(double px, double py, double theta1_1, double theta1_2);
	//double* findTheta4(double r01, double r11, double oldJoint4, double theta1, double theta2);
	//void printInternalMatrix(double **matrix);
	void printMatrix(double** matrix, int height, int width);
	bool errorFound(double *values, int selection);

};

#endif
