#include "del_data.h"

using namespace std;

del_data::del_data()
{

}
void del_data::arrary1DPointer(double *data1, int dim1)
{
	for (int i = 0; i < dim1; i++)
	delete &data1[i];
}

void del_data::array2DPointer(double** data2, int dim1, int dim2)
{
	for (int i = 0; i < dim2; i++)
	{
		delete[] data2[i];
	}
	delete[] data2;
}

void del_data::array3DPointer(double*** data3, int dim1, int dim2, int dim3)
{	for (int i = 0; i < dim2; i++)
	{
		for (int j = 0; j < dim3; j++)
		{
			delete[] data3[i][j];
		}
		delete[] data3[i];
	}
	delete[] data3;
}

