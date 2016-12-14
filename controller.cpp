#include "controller.h"
using namespace std;

/* FUNCTION: Controller
*This function is the constructor for the controller class
*/

controller::controller(){

	
	torque = new double[4];
	torque_p = new double[4];
	sln = new double[4];
	betaSln = new double [4];

	torque_pSln = new double [4];
	kp_e = new double [4];
	kv_de  =new double [4];

	V = new double[4];
	M = new double*[4];
	for (int i = 0; i < 4; i++) {
		M[i] = new double[4];
	}

	alpha = new double*[4];
	for (int i = 0; i < 4; i++) {
		alpha[i] = new double[4];
	}


	//Global Constants
	kp=new double[4];	//orignally taken from textbook

	kp[0] = 300;//175;
	kp[1] = 300;//110;
	kp[2] = 300;//40;
	kp[3] = 200;//20;

}


/* FUNCTION: Torque
 This function takes the  desired and actual values and finds the torque
*/
double* controller::Torque(double* jointDesired, double* velocityDesired, double* accelDesired, double* jointActual, double*  velocityActual) {


	alpha = cmd.massMatrix(jointActual);
	beta = BetaMatrix(jointActual, velocityActual);
	torque_p = TorquePrime(jointDesired, velocityDesired, accelDesired, jointActual, velocityActual);
	/*for (int i = 0; i < 4; i++)
	{
		cout << torque_p[i] << setw(15);
	}
	cout << endl;
*/
	for (int i = 0; i < 4; i++) {
		sln[i] = alpha[i][0] * torque_p[0] + alpha[i][1] * torque_p[1] + alpha[i][2] * torque_p[2] + alpha[i][3] * torque_p[3];

	}

	for (int i = 0; i < 4; i++) {
		torque[i] = sln[i]/(1000*1000) + beta[i]/(1000*1000);
	}

	return torque;
}
/* FUNCTION: BetaMatrix
(see page 300 of text book)
*/
double * controller::BetaMatrix(double * jointActual, double *velocityActual) {
	

	double G[4] = { 0, 0, 2.7*g*jointActual[2]/1000, 0 };											//a 4x1 vector with a constant value, 2.7 comes from matlab
	//double F[4] = { b*jointActual[0], b*jointActual[1], b*jointActual[2], b*jointActual[3] };
	double F[4] = { b*velocityActual[0], b*velocityActual[1], b*velocityActual[2], b*velocityActual[3] };		//When multipled by velocity this is the empiracal frinction force

	V = cmd.Vmatrix(jointActual, velocityActual);

	for (int i = 0; i < 4; i++) {
		betaSln[i] = V[i] + G[i] + F[i];
	}

	return betaSln;
}

/* FUNCTION: TorquePrime
* This function completes the servo law equation by calling Kv_dError and Kp_Error
* Input: desired acceleration, Kv*(derivative of Error), Kp*( Error)
* Ouput: torque prime
*/
double* controller::TorquePrime(double * jointDesired, double * velocityDesired, double * accelDesired, double * jointActual,  double *  velocityActual) {

	for (int i = 0; i < 4; i++) {
		if (i == 2) {
			kp_e[i] = kp[i] * (jointDesired[i] - jointActual[i]); //Add the desired joint values to actual and multiplies the result by Kp
		}
		else {
			kp_e[i] = kp[i] * (jointDesired[i] * PI / 180 - jointActual[i] * PI / 180); //Add the desired joint values to actual and multiplies the result by Kp
		}
		
		kv = 2 * sqrt(kp[i]);
		kv_de[i] = kv * (velocityDesired[i] - velocityActual[i]); //Add the desired joint velocities to actual and multiplies the result by Kv
		
		torque_pSln[i] = accelDesired[i]+ kp_e[i] + kv_de[i]; //Final output
	}

	return torque_pSln;
}


/* FUNCTION: massMatrix
* This function calcualtes the Mass Matrix from the given terms
* Input: joint values
* Output: a 4x1 array of the results

NOTE: This is NOT used in the dynamic symulator but the control system
*/
double** controller::MASSMatrix(double* joint) {			//double** massMatrix(double* joint) { let's not use pointers

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