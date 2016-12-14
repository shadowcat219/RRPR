#pragma once
#define UI_CONSOLE

#include <stdio.h>
#include <iostream>
#include <string>
#include "robo_control.h"

using namespace std;

class ui_console 
{
private:
	int select_bit;
	int emul_select;
	int HW_select;
	char quit_bit;
	robo_control cmd;
public:
	void welcomeSplash();
	void emulatorSplash();
	void quitSplash();
};

