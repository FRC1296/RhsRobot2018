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
#include "Autonomous.h"
#include "Drivetrain.h"
#include "Claw.h"
#include "Elevator.h"
#include "ctre/Phoenix.h"

class RhsRobot : public RhsRobotBase
{
public:
	RhsRobot();
	virtual ~RhsRobot();

private:
	std::vector<ComponentBase *> ComponentSet;

	Joystick* pControllerDriver;
	Joystick* pControllerOperator;

	frc::SendableChooser<char> *pChooser;

	Autonomous *pAuto;
	Drivetrain* pDrivetrain;
	Claw* pClaw;
	Elevator* pElevator;

	string gameData = "XXX";
	string gameDataPrev = "XXX";
	char sStartLocation = 'X';
	char sStartLocationLast = 'X';

	void Init();
	void OnStateChange();
	void Run();
	void UpdateGameData();
	void UpdateSystemData();

	int iLoop;
};

#endif //RHS_ROBOT_H
