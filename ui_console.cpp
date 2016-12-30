#include "ui_console.h"

void ui_console::welcomeSplash()
{
		emulatorSplash();
}

void ui_console::emulatorSplash()
{
	cout << " " << endl;
	cout << "List of Commands, please select one by typeing corresponding command number" << endl;
	cout << "0: Quit Emulation" << endl;
	cout << "1: Initialize Robot Position, joints = (0,0,-150,0)" << endl;
	cout << "2: Reset to Start Position, joints = (90,0,-175,0)" << endl;
	cout << "3: Set Joint Values (WHERE function)" << endl;
	cout << "4: Move to X,Y,Z, Phi (SOLVE function)" << endl;
	cout << "5: Move Gripper" << endl;
	cout << "6: Get Current Joint Configuration" << endl;
	cout << "7: Get Current Cartesian Coordinates " << endl;
	cout << "8: Plan a Robot Trajectory" << endl;
	cout << "9: Run Dynamic Simualtion alone" << endl;
	cout << "10:Run a Trajectory with a Controller" << endl;
	cout << "Input: ";
	
	emul_select = -1;

	while (emul_select != 0 || emul_select != 1 || emul_select != 2 || emul_select != 2 || emul_select != 3 || emul_select != 4)
	{
		cin >> emul_select;
		cout << endl;
		switch (emul_select)
		{
		case 0: //done
			quitSplash();
			CloseMonitor();
			StopRobot();
			exit(0);
			break;
		case 1:
			cmd.initJoint();
			break;
		case 2:
			cmd.zeroPosition();
			break;
		case 3:
			cmd.moveJOINT();
			break;
		case 4:
			cmd.moveCart();
			break;
		case 5:
			cmd.moveGriper();
			break;
		case 6:
			cmd.currentJoints();			
			break;
		case 7:
			cmd.currentCartesian();
			break;
		case 8:	
			cmd.trajectoryPlan();
			break;	
		case 9:
			cmd.dynamicSim();
			break;
		case 10:
			cmd.controlledTrajectory();
			break;
		default:
			quitSplash();
			break;
		}
		emulatorSplash();
	}
}

void ui_console::quitSplash()
{
	cout << "System shutting down..." << endl;
	//necceasy reset command if any
	cout << "" << endl;
	cmd.stopRobot();
	system("pause");
}
