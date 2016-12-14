
#include "DynamicSim.h"
#include <iostream>
#include <string>
#include <iomanip>

using namespace std;

void main()
{
	double torques[4], accel[4];
	double M[4][4], MInv[4][4];
	double joint[4] = { 100, 50, -175, 100 };
	double velocity[4] = { 10 ,10 ,10,10 };
	double acceleration[4] = { 25, 25, 5, 25 };
	double nextValues[3][4];
	double dynamicAcc[4];
	DynamicSim cmd;
	
	cout << "Enter torque values on each joint (1,0,0,0)" << endl;
	cin >> torques[0] >> torques[1] >> torques[2] >> torques[3];

	//M[4][4] = cmd.massMatrix(&joint[4]);
	nextValues[3][4] = cmd.NumericalIntegration(joint, velocity, acceleration, torques);

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cout << M[i][j] << setw(15);
		}
		cout << endl;
	}
	
}