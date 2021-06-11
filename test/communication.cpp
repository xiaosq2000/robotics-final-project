
#include "communication.h"

int main()
{
	Communication communication;
	communication.SendInstruction("[1# System.Auto 1]");
	communication.SendInstruction("[2# System.Speed 10]");
	communication.SendInstruction("[2# Robot.JogMode 1,1,1]");
	communication.SendInstruction("[3# Robot.Frame 1,2]");
	communication.SendInstruction("[4# Move.Jog X,100]");
	communication.SendInstruction("[5# Move.Jog X,-100]");
	communication.~Communication();
	return 0;
}
