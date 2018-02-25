
#include "Component.h"
#include "ComponentBase.h"
#include "RobotParams.h"
#include "WPILib.h"
#include "Elevator.h"

//Robot


/* servo notes
 *
 * when the elevator goes up the encoder ticks DECREASE for sure
 *
 * joysticks give negative numbers when you press them up, remember they were originally
 * designed as flight controls
 *
 * positive motor motion makes the elevator go up for sure
 *
 * so we need to SetSensorPhase(true) to reverse the direction sense of the encoder ticks
 *
 * we do NOT need to call SetInverted(true)
 *
 * so why was the servo going nuts?  a bug!!  one must call ConfigPeakCurrentDuration
 * if you are using ConfigPeakCurrentLimit!
 *
 *
 */


Elevator::Elevator()
: ComponentBase(ELEVATOR_TASKNAME, ELEVATOR_QUEUE, ELEVATOR_PRIORITY)
{
	//TODO: add member objects
	pElevatorMotorLeft = new TalonSRX(CAN_ELEVATOR_TALON_LEFT);
	pElevatorMotorRight = new TalonSRX(CAN_ELEVATOR_TALON_RIGHT);
	pElevatorMotorLeft->SetNeutralMode(NeutralMode::Brake);
	pElevatorMotorRight->SetNeutralMode(NeutralMode::Brake);
	pElevatorMotorLeft->Set(ControlMode::PercentOutput,0);
	pElevatorMotorRight->Set(ControlMode::PercentOutput,0);
	pElevatorMotorLeft->ConfigPeakCurrentLimit(30,1000);
	pElevatorMotorRight->ConfigPeakCurrentLimit(30,1000);
	pElevatorMotorLeft->ConfigContinuousCurrentLimit(30,1000);
	pElevatorMotorRight->ConfigContinuousCurrentLimit(30,1000);
	pElevatorMotorLeft->ConfigPeakCurrentDuration(0.0,10);
	pElevatorMotorRight->ConfigPeakCurrentDuration(0.0,10);
	pElevatorMotorLeft->EnableCurrentLimit(true);
	pElevatorMotorRight->EnableCurrentLimit(true);
	pElevatorMotorLeft->SetInverted(false);
	pElevatorMotorRight->SetInverted(false);
	pElevatorMotorLeft->ConfigPeakOutputForward(0.75, 10);
	pElevatorMotorLeft->ConfigPeakOutputReverse(-.25,10);
	pElevatorMotorRight->ConfigPeakOutputForward(0.75, 10);
	pElevatorMotorRight->ConfigPeakOutputReverse(-.25,10);

	pElevatorMotorLeft->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,10);
	pElevatorMotorLeft->SetSensorPhase(true);
	pElevatorMotorLeft->Config_kF(0,0.0,10);
	pElevatorMotorLeft->Config_kP(0,0.2,10);
	pElevatorMotorLeft->Config_kI(0,0.0,10);
	pElevatorMotorLeft->Config_kD(0,0.0,10);
	pElevatorMotorLeft->SelectProfileSlot(0,0);
	pElevatorMotorLeft->ConfigAllowableClosedloopError(0, 200, 10);

	Wait(1.0);
	pElevatorMotorRight->Set(ControlMode::Follower,CAN_ELEVATOR_TALON_LEFT);
	pElevatorMotorLeft->Set(ControlMode::Position, pElevatorMotorLeft->GetSelectedSensorPosition(0));
	//pElevatorMotorLeft->Set(ControlMode::PercentOutput, 0.0);

	iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(0);
	iStartPos = iCurrPos;
	fCurVoltage = 0;
	fMotorSpeed = 0;
	fMaxSpeed = 0;
    iMoveDelta = 0;  // for now

	pEleTimeout = new Timer();

	SmartDashboard::PutNumber("Elevator Speed RPM",((fMaxSpeed*600)/4096.0));
	SmartDashboard::PutNumber("Elevator Position in Ticks",iCurrPos);
	SmartDashboard::PutNumber("Elevator Position in Rotations",iCurrPos/4096.0);

	SmartDashboard::PutNumber("Elevator Init Position",iStartPos);
	SmartDashboard::PutNumber("Elevator Switch Position",iStartPos + iFloorToSwitch);
	SmartDashboard::PutNumber("Elevator Scale Position",iStartPos + iFloorToScale);

	pTask = new std::thread(&Elevator::StartTask, this, ELEVATOR_TASKNAME, ELEVATOR_PRIORITY);
	wpi_assert(pTask);
};

Elevator::~Elevator()
{
	//TODO delete member objects

	delete(pTask);
};

void Elevator::OnStateChange()
{
};

void Elevator::Run()
{
	iCurrPos = pElevatorMotorLeft->GetSelectedSensorPosition(0);

	SmartDashboard::PutNumber("Elevator Speed RPM",((fMaxSpeed*600)/4096.0));
	SmartDashboard::PutNumber("Elevator Position in Ticks",iCurrPos);
	SmartDashboard::PutNumber("Elevator Position in Rotations",iCurrPos/4096.0);

	SmartDashboard::PutNumber("Elevator Init Position",iStartPos);
	SmartDashboard::PutNumber("Elevator Switch Position",iStartPos + iFloorToSwitch);
	SmartDashboard::PutNumber("Elevator Scale Position",iStartPos + iFloorToScale);
	SmartDashboard::PutNumber("Difference",(iStartPos + iFloorToScale) - iCurrPos);

	switch(localMessage.command)			//Reads the message command
	{
	case COMMAND_COMPONENT_TEST:
		break;

	case COMMAND_ELEVATOR_MOVE:
		fMotorSpeed = localMessage.params.elevator.fSpeed;
		//pElevatorMotorLeft->Set(ControlMode::PercentOutput, -fMotorSpeed);

		if (fMotorSpeed > 0.2)
		{
		    iMoveDelta -= 100;
		    //if(iLoop % 25 == 0)
		    //	printf("Joystick %0.2f speed %0.2f pos %d\n", fMotorSpeed, -fMotorSpeed, iCurrPos);
		}
		else if(fMotorSpeed < -0.2)
		{
		    iMoveDelta += 100;
		    //if(iLoop % 25 == 0)
		    //	printf("Joystick %0.2f speed %0.2f pos %d\n", fMotorSpeed, -fMotorSpeed, iCurrPos);
		}
		break;

	case COMMAND_ELEVATOR_NOBUTTON:
		iMoveDelta = 0;
		pElevatorMotorLeft->Set(ControlMode::Position, iStartPos + iMoveDelta);
		SmartDashboard::PutString("Button", "No Buttons Pushed");
		pEleTimeout->Reset();
		pEleTimeout->Start();
		break;

	case COMMAND_ELEVATOR_FLOOR:
		pElevatorMotorLeft->Set(ControlMode::Position, iStartPos + iMoveDelta);
		SmartDashboard::PutString("Button", "A Button Pushed");
		pEleTimeout->Reset();
		pEleTimeout->Start();
		break;

	case COMMAND_ELEVATOR_SWITCH:
		if( (iStartPos + iFloorToSwitch + iMoveDelta) > iFloorToMax)
		{
			pElevatorMotorLeft->Set(ControlMode::Position, iFloorToMax);
		}
		else
		{
			pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToSwitch + iMoveDelta);
		}

		SmartDashboard::PutString("Button", "B Button Pushed");
		pEleTimeout->Reset();
		pEleTimeout->Start();
		break;

	case COMMAND_ELEVATOR_SCALE_MID:
		if( (iStartPos + iFloorToScale + iMoveDelta) > iFloorToMax)
		{
			pElevatorMotorLeft->Set(ControlMode::Position, iFloorToMax);
		}
		else
		{
			pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToScale + iMoveDelta);
		}

		SmartDashboard::PutString("Button", "Y Button Pushed");
		pEleTimeout->Reset();
		pEleTimeout->Start();
		break;

	case COMMAND_ELEVATOR_CLIMB:
	//	pElevatorMotorLeft->Set(ControlMode::Position,iStartPos + iFloorToClimb);
		break;

	default:
		break;
	}

	// implement timeout

	SmartDashboard::PutNumber("Closed Loop Error", pElevatorMotorLeft->GetClosedLoopError(0));

	if((pElevatorMotorLeft->GetClosedLoopError(0) < 200) && (pElevatorMotorLeft->GetClosedLoopError(0) > -200))
	{
		// the servo has closed
		pEleTimeout->Reset();
		pEleTimeout->Stop();
	}
	else
	{
		if(pEleTimeout->Get() > 3)
		{
			// use the current position which should stop the servo
			pElevatorMotorLeft->Set(ControlMode::Position, iCurrPos);
		}
	}

};

