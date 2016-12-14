#pragma once
#ifndef DEL_DATA
#define DEL_DATA

#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>

using namespace std;

class del_data
{
private:
	double*** data3;
	double** data2;
	double* data1;
	int dim1;
	int dim2;
	int dim3;
public:
	del_data();
	void arrary1DPointer(double *data1, int dim1);
	void array2DPointer(double **data2, int dim1, int dim2);
	void array3DPointer(double ***data3, int dim1, int dim2, int dim3);
};

#endif