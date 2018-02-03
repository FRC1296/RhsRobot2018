/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands.
 */
#ifndef RHS_ROBOT_H
#define RHS_ROBOT_H

#include "RhsRobotBase.h"
#include "WPILib.h"
#include "Drivetrain.h"
#include "Claw.h"
#include "Elevator.h"
//#include "SendableChooser.h"

class RhsRobot : public RhsRobotBase
{
public:
	RhsRobot();
	virtual ~RhsRobot();

private:
	
	std::vector<ComponentBase *> ComponentSet;

	Joystick* pController_1;

	frc::SendableChooser<char> *pChooser;
	Drivetrain* pDrivetrain;
	Claw* pClaw;
	Elevator* pElevator;

	void Init();
	void OnStateChange();
	void Run();

	int iLoop;
};

#endif //RHS_ROBOT_H
