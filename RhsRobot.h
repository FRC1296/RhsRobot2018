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
#include "Arm.h"
#include "Climber.h"
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
	Compressor* pCompressor;
	PowerDistributionPanel* pPDP;
	Timer *pSpeedTimer;

	frc::SendableChooser<char> *pChooser;

	Autonomous *pAuto;
	Drivetrain* pDrivetrain;
	Claw* pClaw;
	Elevator* pElevator;
	Arm* pArm;
	Climber* pClimber;

	string gameData = "XXX";
	string gameDataPrev = "XXX";
	char sStartLocation = 'X';
	char sStartLocationLast = 'X';
	bool bLimitSpeedWhileElevatorIsUp;

	float fLeftTrigger;
	float fRightTrigger;
	float fHeightPercent;
	float fDrivetrainSpeed;
	float fExhaustLimit;

	RobotMessage lastMessage;
	RobotMessage maxMessage;

	void Init();
	void OnStateChange();
	void Run();
	void UpdateGameData();
	void UpdateSystemData();
	void MonitorPDB();

	int iLoop;
};

#endif //RHS_ROBOT_H
