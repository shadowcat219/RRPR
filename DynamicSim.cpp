// DynamicSim.cpp : Defines the entry point for the console application.
//

//#include <stdio.h>

#include "DynamicSim.h"
//Global constants:
//needs to be able to call outside
//double g = 9.81;					//effect of gravity
//double b = 5;						//Viscous friction
//double RES2 = RES*.1;  			// Sampling resolution for the dynamic simulator -> needs to be 1/10th of RES

DynamicSim::DynamicSim()
{
	//joint = new double[4];
	//velocity = new double[4];
	Accel = new double[4];
	///torque = new double[4];

	nextValues = new double*[2];
	for (int i = 0; i < 4; i++)
	{
		nextValues[i] = new double[4];
	}

	MInv= new double*[4];
	for (int i = 0; i < 4; i++) {
		MInv[i] = new double[4];
	}
	
	M = new double*[4];
	for (int i = 0; i < 4; i++) {
		M[i] = new double[4];
	}
	
	V = new double[4];
}

/* FUNCTION: DynamicAcceleration
* This function takes the current joint values and their velociites (at a specific time) and uses them to calculate the dynamic acceleration.
* Input: joint values [4x1] and joint velocities [4x1], tourque vector [4x1]
* Calls: Vmatrix,invMassMatrix
* Uses global variables G,F
* Output: 4x1 array of the results
*/
double* DynamicSim::DynamicAcceleration(double *joint, double *velocity, double *torque) {
	double temp[4];
	//double** invM;
	//double V[4];
	double sln[4];

	double G[4] = { 0, 0, 2.7*g*(joint[2]/1000), 0 };											//a 4x1 vector with a constant value, 2.7 comes from matlab
	double F[4] = { b*velocity[0] / 1000, b*velocity[1] / 1000, b*velocity[2] / 1000, b*velocity[3] / 1000 };		//When multipled by velocity this is the empiracal frinction force
	//double F[4] = { b*velocity[0], b*joint[1], b*joint[2], b*joint[3] };

	MInv = invMassMatrix(joint);
	V = Vmatrix(joint, velocity);

	for (int i = 0; i < 4; i++) {
		sln[i] = torque[i] - V[i] - G[i] - F[i];
	}
	for (int i = 0; i < 4; i++) {
		temp[i] = MInv[i][0] * sln[0] + MInv[i][1] * sln[1] + MInv[i][2] * sln[2] + MInv[i][3] * sln[3];
	}
	for (int i = 0; i < 4; i++)
	{
		Accel[i] = temp[i];
	}
	return Accel;
	}

/* FUNCTION: NumericalIntegration
* This function calculates the next postion and veloctiy of a joint
* Input: current joint values [4x1], joint velocities [4x1], joint acceleration [4x1], delta_t of step size
* Output: new joint values and joint velocities [4x2]
*/
double** DynamicSim::NumericalIntegration(double *joint, double *velocity, double *accel, double *torque) {
	double temp[2][4], tempJoint[4];

	//joint[0] = DEG2RAD(joint[0]);
	tempJoint[0] = DEG2RAD(joint[0]);// = (joint[0] * PI / 180);
	tempJoint[1] = DEG2RAD(joint[1]); //joint[1] = DEG2RAD(joint[1]);
	tempJoint[2] = joint[2];
	tempJoint[3] = DEG2RAD(joint[3]); //joint[3] = DEG2RAD(joint[3]);

	//double *nextValues = new double[3];
	for (int j = 0; j < 4; j++)
	{
		//saves joint values for next time period
		temp[1][j] = velocity[j] + accel[j] * (RES2 / 1000);

		temp[0][j] = tempJoint[j] + velocity[j] * (RES2/1000) + (0.5)*accel[j]*pow(RES2/1000, 2);
		if (j == 2) {
			temp[0][j] = temp[0][j];
		}
	else
		{
			temp[0][j] = RAD2DEG(temp[0][j]);
		}
		

	}
	
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			nextValues[i][j] = temp[i][j];
		}
	}
	return nextValues;
}

/* FUNCTION: Vmatrix
* This function handles the Centrifugal and Coriolis terms
* Input: all joint location values and their associated velocity
* Ouput: a 4x1 array of the results
*/
double* DynamicSim::Vmatrix(double* joint, double * velocity) {

	double temp[4], tempJoint[4];
	tempJoint[0] = DEG2RAD(joint[0]);// = DEG2RAD(joint[0]);
	tempJoint[1] = DEG2RAD(joint[1]);// = DEG2RAD(joint[1]);
	tempJoint[2] = joint[2];
	tempJoint[3] = DEG2RAD(joint[3]);// = DEG2RAD(joint[3]);
	
	//4260.0*pow(velocity[4],2) * sin(joint[3]) - 74763.0*pow(velocity[2],2) * sin(joint[1]) - 5850.0*pow(velocity[2],2) * sin(joint[1] - joint[3]) - 5850.0*pow(velocity[4],2)* sin(joint[1] - joint[3]) - 149526.0*velocity[1]*velocity[2]*sin(joint[1]) - 8520.0*velocity[1]*velocity[4]*sin(joint[3]) - 8520.0*velocity[2]*velocity[4]*sin(joint[3]) - 11700.0*velocity[1]*velocity[2]*sin(joint[1] - joint[3]) + 11700.0*velocity[1]*velocity[4]*sin(joint[1] - joint[3]) + 11700.0*velocity[2]*velocity[4]*sin(joint[1] - joint[3]);
	//74763.0*pow(velocity[1], 2) * sin(joint[1]) + 4260.0*pow(velocity[4],2) * sin(joint[3]) + 5850.0*pow(velocity[1],2) * sin(joint[1] - joint[3]) - 8520.0*velocity[1]*velocity[4]*sin(joint[3]) - 8520.0*velocity[2]*velocity[4]*sin(joint[3]);

	//temp[0] = 4260.0*pow(velocity[3], 2) * sin(joint[3]) - 74763.0*pow(velocity[1], 2) * sin(joint[1]) - 5850.0*pow(velocity[1], 2) * sin(joint[1] - joint[3]) - 5850.0*pow(velocity[3], 2)* sin(joint[1] - joint[3]) - 149526.0*velocity[1] * velocity[1] * sin(joint[1]) - 8520.0*velocity[1] * velocity[3] * sin(joint[3]) - 8520.0*velocity[1] * velocity[3] * sin(joint[3]) - 11700.0*velocity[1] * velocity[1] * sin(joint[1] - joint[3]) + 11700.0*velocity[1] * velocity[3] * sin(joint[1] - joint[3]) + 11700.0*velocity[1] * velocity[3] * sin(joint[1] - joint[3]);
	//temp[1] = 74763.0*pow(velocity[1], 2) * sin(joint[1]) + 4260.0*pow(velocity[3], 2) * sin(joint[3]) + 5850.0*pow(velocity[1], 2) * sin(joint[1] - joint[3]) - 8520.0*velocity[1] * velocity[3] * sin(joint[3]) - 8520.0*velocity[1] * velocity[3] * sin(joint[3]);
	//temp[2] = 0;
	//temp[3] = sin(joint[3])*(5850.0*pow(velocity[1], 2)*cos(joint[1]) + 30 * (velocity[1] + velocity[1])*(142.0*velocity[1] + 142.0*velocity[1])) - (5850.0*pow(velocity[1], 2) * cos(joint[3])*sin(joint[1]));

	temp[0] = 4260.0*pow(velocity[3], 2) * sin(tempJoint[3]) - 74763.0*pow(velocity[1], 2) * sin(tempJoint[1]) - 5850.0*pow(velocity[1], 2) * sin(tempJoint[1] - tempJoint[3]) - 5850.0*pow(velocity[3], 2)* sin(tempJoint[1] - tempJoint[3]) - 149526.0*velocity[1] * velocity[1] * sin(tempJoint[1]) - 8520.0*velocity[1] * velocity[3] * sin(tempJoint[3]) - 8520.0*velocity[1] * velocity[3] * sin(tempJoint[3]) - 11700.0*velocity[1] * velocity[1] * sin(tempJoint[1] - tempJoint[3]) + 11700.0*velocity[1] * velocity[3] * sin(tempJoint[1] - tempJoint[3]) + 11700.0*velocity[1] * velocity[3] * sin(tempJoint[1] - tempJoint[3]);
	temp[1] = 74763.0*pow(velocity[1], 2) * sin(tempJoint[1]) + 4260.0*pow(velocity[3], 2) * sin(tempJoint[3]) + 5850.0*pow(velocity[1], 2) * sin(tempJoint[1] - tempJoint[3]) - 8520.0*velocity[1] * velocity[3] * sin(tempJoint[3]) - 8520.0*velocity[1] * velocity[3] * sin(tempJoint[3]);
	temp[2] = 0;
	temp[3] = sin(tempJoint[3])*(5850.0*pow(velocity[1], 2)*cos(tempJoint[1]) + 30 * (velocity[1] + velocity[1])*(142.0*velocity[1] + 142.0*velocity[1])) - (5850.0*pow(velocity[1], 2) * cos(tempJoint[3])*sin(tempJoint[1]));

	/*double V[4] =
	{ 4260.0*pow(velocity[4], 2)* sin(joint[3]) - 74763.0*pow(velocity[2], 2)*sin(joint[1]) - 5850.0*pow(velocity[2], 2)*sin(joint[1] - joint[3]) - 5850.0*pow(velocity[4], 2)*sin(joint[1] - joint[3]) - 149526.0*velocity[1] * velocity[2] * sin(joint[1]) - 8520.0*velocity[1] * velocity[4] * sin(joint[3]) - 8520.0*velocity[2] * velocity[4] * sin(joint[3]) - 11700.0*velocity[1] * velocity[2] * sin(joint[1] - joint[3]) + 11700.0*velocity[1] * velocity[4] * sin(joint[1] - joint[3]) + 11700.0*velocity[2] * velocity[4] * sin(joint[1] - joint[3]),
	74763.0*pow(velocity[1], 2)*sin(joint[1]) + 4260.0*pow(velocity[4], 2) * sin(joint[3]) + 5850.0*pow(velocity[1], 2)* sin(joint[1] - joint[3]) - 8520.0*velocity[1] * velocity[4] * sin(joint[3]) - 8520.0*velocity[2] * velocity[4] * sin(joint[3]),
	0,
	sin(joint[3])*(5850.0*pow(velocity[1], 2)*cos(joint[1]) + 30 * (velocity[1] + velocity[2])*(142.0*velocity[1] + 142.0*velocity[2])) - (5850.0*pow(velocity[1], 2) * cos(joint[3])*sin(joint[1]))
	};
	*/

	for (int j = 0; j < 4; j++)
	{
		V[j] = temp[j];
	}
	return V;
}

/* FUNCTION: invMassMatrix
* This function calcualtes the INVERSE Mass Matrix from the given terms
* Input: joint values
* Output: a 4x1 array of the results
*/
double** DynamicSim::invMassMatrix(double* joint)
{ 
	double temp[4][4], tempJoint[4];
	tempJoint[0] = DEG2RAD(joint[0]);// = DEG2RAD(joint[0]);
	tempJoint[1] = DEG2RAD(joint[1]);// = DEG2RAD(joint[1])
	tempJoint[2] = joint[2];
	tempJoint[3] = DEG2RAD(joint[3]);// *PI / 180;

	//joint[0] = the1, joint[1] = the2, joint[3] = the4
	temp[0][0] = (0.00026298487836949375410913872452334*(5.0*pow(cos(tempJoint[3]), 2) - 27.0)) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0);
	temp[0][1] = (0.0000018520061857006602402052022853756*(5265.0*cos(tempJoint[1]) - 975.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[3]) - 710.0*pow(cos(tempJoint[3]), 2) + 3834.0)) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0);
	temp[0][2] = 0;
	temp[0][3] = -(0.000036114120621162874684001444564825*(639.0*cos(tempJoint[1] - 1.0*tempJoint[3]) - 270.0*cos(tempJoint[1]) + 50.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[3]) - 639.0*cos(tempJoint[1])*cos(tempJoint[3]))) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0);

	temp[1][0] = (0.0000018520061857006602402052022853756*(5265.0*cos(tempJoint[1]) - 975.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[3]) - 710.0*pow(cos(tempJoint[3]), 2) + 3834.0)) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0);
	temp[1][1] = (0.000000013042297082399015776092973840673*(276900.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[3]) - 1495260.0*cos(tempJoint[1]) + 190125.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 100820.0*pow(cos(tempJoint[3]), 2) - 1951353.0)) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0);
	temp[1][2] = 0;
	temp[1][3] = ((0.00000025432479310678080763381298989313*(90738.0*cos(tempJoint[1] - 1.0*tempJoint[3]) - 38340.0*cos(tempJoint[1]) - 170755.0*cos(tempJoint[3]) + 124605.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1]) + 7100.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[3]) - 90738.0*cos(tempJoint[1])*cos(tempJoint[3]) + 9750.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) - 72150.0)) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0));

	temp[2][0] = 0;
	temp[2][1] = 0;
	temp[2][2] = 0.37037037037037037037037037037037;
	temp[2][3] = 0,
		
	temp[3][0] = -(0.000036114120621162874684001444564825*(639.0*cos(tempJoint[1] - 1.0*tempJoint[3]) - 270.0*cos(tempJoint[1]) + 50.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[3]) - 639.0*cos(tempJoint[1])*cos(tempJoint[3]))) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0);
	temp[3][1] = (0.00000025432479310678080763381298989313*(90738.0*cos(tempJoint[1] - 1.0*tempJoint[3]) - 38340.0*cos(tempJoint[1]) - 170755.0*cos(tempJoint[3]) + 124605.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1]) + 7100.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[3]) - 90738.0*cos(tempJoint[1])*cos(tempJoint[3]) + 9750.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) - 72150.0)) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0);
	temp[3][2] = 0;
	temp[3][3] = (0.00000033062223103881504992395688686107*(191700.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1]) - 262700.0*cos(tempJoint[3]) + 7500.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 1224963.0*pow(cos(tempJoint[1]), 2) - 1734153.0)) / (135.0*pow(cos(tempJoint[1] - 1.0*tempJoint[3]), 2) + 729.0*pow(cos(tempJoint[1]), 2) + 185.0*pow(cos(tempJoint[3]), 2) - 270.0*cos(tempJoint[1] - 1.0*tempJoint[3])*cos(tempJoint[1])*cos(tempJoint[3]) - 999.0);


/*	{ (0.00026298487836949375410913872452334*(5.0*pow(cos(joint[3]),2) - 27.0)) / (135.0*pow(cos(joint[1] - 1.0*joint[3]),2) + 729.0*pow(cos(joint[1]),2) + 185.0*pow(cos(joint[3]),2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0),
		(0.0000018520061857006602402052022853756*(5265.0*cos(joint[1]) - 975.0*cos(joint[1] - 1.0*joint[3])*cos(joint[3]) - 710.0*pow(cos(joint[3]), 2) + 3834.0)) / (135.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 729.0*pow(cos(joint[1]), 2) + 185.0*pow(cos(joint[3]), 2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0),
		0,
		-(0.000036114120621162874684001444564825*(639.0*cos(joint[1] - 1.0*joint[3]) - 270.0*cos(joint[1]) + 50.0*cos(joint[1] - 1.0*joint[3])*cos(joint[3]) - 639.0*cos(joint[1])*cos(joint[3]))) / (135.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 729.0*pow(cos(joint[1]), 2) + 185.0*pow(cos(joint[3]), 2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0) 
		},

		{ (0.0000018520061857006602402052022853756*(5265.0*cos(joint[1]) - 975.0*cos(joint[1] - 1.0*joint[3])*cos(joint[3]) - 710.0*pow(cos(joint[3]), 2) + 3834.0)) / (135.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 729.0*pow(cos(joint[1]), 2) + 185.0*pow(cos(joint[3]), 2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0),
		(0.000000013042297082399015776092973840673*(276900.0*cos(joint[1] - 1.0*joint[3])*cos(joint[3]) - 1495260.0*cos(joint[1]) + 190125.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 100820.0*pow(cos(joint[3]), 2) - 1951353.0)) / (135.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 729.0*pow(cos(joint[1]), 2) + 185.0*pow(cos(joint[3]), 2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0),
		0,
		((0.00000025432479310678080763381298989313*(90738.0*cos(joint[1] - 1.0*joint[3]) - 38340.0*cos(joint[1]) - 170755.0*cos(joint[3]) + 124605.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1]) + 7100.0*cos(joint[1] - 1.0*joint[3])*cos(joint[3]) - 90738.0*cos(joint[1])*cos(joint[3]) + 9750.0*pow(cos(joint[1] - 1.0*joint[3]),2) - 72150.0)) / (135.0*pow(cos(joint[1] - 1.0*joint[3]),2) + 729.0*pow(cos(joint[1]),2) + 185.0*pow(cos(joint[3]),2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0))
		},

		{ 0,
		0,
		0.37037037037037037037037037037037,
		0 },

		{ -(0.000036114120621162874684001444564825*(639.0*cos(joint[1] - 1.0*joint[3]) - 270.0*cos(joint[1]) + 50.0*cos(joint[1] - 1.0*joint[3])*cos(joint[3]) - 639.0*cos(joint[1])*cos(joint[3]))) / (135.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 729.0*pow(cos(joint[1]), 2) + 185.0*pow(cos(joint[3]), 2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0),
		(0.00000025432479310678080763381298989313*(90738.0*cos(joint[1] - 1.0*joint[3]) - 38340.0*cos(joint[1]) - 170755.0*cos(joint[3]) + 124605.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1]) + 7100.0*cos(joint[1] - 1.0*joint[3])*cos(joint[3]) - 90738.0*cos(joint[1])*cos(joint[3]) + 9750.0*pow(cos(joint[1] - 1.0*joint[3]), 2) - 72150.0)) / (135.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 729.0*pow(cos(joint[1]), 2) + 185.0*pow(cos(joint[3]), 2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0),
		0,
		(0.00000033062223103881504992395688686107*(191700.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1]) - 262700.0*cos(joint[3]) + 7500.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 1224963.0*pow(cos(joint[1]), 2) - 1734153.0)) / (135.0*pow(cos(joint[1] - 1.0*joint[3]), 2) + 729.0*pow(cos(joint[1]), 2) + 185.0*pow(cos(joint[3]), 2) - 270.0*cos(joint[1] - 1.0*joint[3])*cos(joint[1])*cos(joint[3]) - 999.0) }
	};
*/
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			MInv[i][j] = temp[i][j];
		}
	}

	return MInv;
}


/* FUNCTION: massMatrix
* This function calcualtes the Mass Matrix from the given terms
* Input: joint values
* Output: a 4x1 array of the results

NOTE: This is NOT used in the dynamic symulator but the control system
*/
double** DynamicSim::massMatrix(double* joint){			//double** massMatrix(double* joint) { let's not use pointers
	
	double temp[4][4], tempJoint[4];
	tempJoint[0] = DEG2RAD(joint[0]);// = DEG2RAD(joint[0]);
	tempJoint[1] = DEG2RAD(joint[1]);// = DEG2RAD(joint[1]);
	tempJoint[2] = joint[2];
	tempJoint[3] = DEG2RAD(joint[3]);// = DEG2RAD(joint[3]);	joint[0] = DEG2RAD(joint[0]);// *PI / 180;
	//joint[1] = DEG2RAD(joint[1]); //joint[1] * PI / 180;
	//joint[3] = DEG2RAD(joint[3]); //joint[3] * PI / 180;
	//joint[0] = the1, joint[1] = the2, joint[3] = the4
	
	
	temp[0][0] = 11700.0*cos(tempJoint[1] - tempJoint[3]) + 149526.0*cos(tempJoint[1]) + 8520.0*cos(tempJoint[3]) + 196935.3;
	temp[0][1] = 5850.0*cos(tempJoint[1] - tempJoint[3]) + 74763.0*cos(tempJoint[1]) + 8520.0*cos(tempJoint[3]) + 56242.8;
	temp[0][2] = 0;
	temp[0][3] = -5850.0*cos(tempJoint[1] - tempJoint[3]) - 4260.0*cos(tempJoint[3]) - 1800.0;

	temp[1][0] = 5850.0*cos(tempJoint[1] - tempJoint[3]) + 74763.0*cos(tempJoint[1]) + 8520.0*cos(tempJoint[3]) + 56242.8;
	temp[1][1] = 8520.0*cos(tempJoint[3]) + 56242.8;
	temp[1][2] = 0;
	temp[1][3] = -4260.0*cos(tempJoint[3]) - 1800.0;

	temp[2][0] = 0;
	temp[2][1] = 0;
	temp[2][2] = 2.7;
	temp[2][3] = 0,

	temp[3][0] = -5850.0*cos(tempJoint[1] - tempJoint[3]) - 4260.0*cos(tempJoint[3]) - 1800.0;
	temp[3][1] = -4260.0*cos(tempJoint[3]) - 1800.0;
	temp[3][2] = 0;
	temp[3][3] = 1800;
	
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			M[i][j] = temp[i][j];
		}
	}

	return M;
	/*
	double	M[4][4] =
	{
		{ 11700.0*cos(joint[1] - joint[3]) + 149526.0*cos(joint[1]) + 8520.0*cos(joint[3]) + 196935.3,
		5850.0*cos(joint[1] - joint[3]) + 74763.0*cos(joint[1]) + 8520.0*cos(joint[3]) + 56242.8,
		0,
		-5850.0*cos(joint[1] - joint[3]) - 4260.0*cos(joint[3]) - 1800.0 },

		{ 5850.0*cos(joint[1] - joint[3]) + 74763.0*cos(joint[1]) + 8520.0*cos(joint[3]) + 56242.8,
		8520.0*cos(joint[3]) + 56242.8,
		0,
		-4260.0*cos(joint[3]) - 1800.0 },
		{ 0, 0, 2.7, 0 },
		{ -5850.0*cos(joint[1] - joint[3]) - 4260.0*cos(joint[3]) - 1800.0,
		-4260.0*cos(joint[3]) - 1800.0,
		0,
		1800.0 }
	};
	*/
}