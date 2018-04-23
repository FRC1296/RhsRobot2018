
#include "Component.h"
#include "ComponentBase.h"
#include "RobotParams.h"
#include "WPILib.h"
#include "Arm.h"

//Robot


/* servo notes
 *
 * when the arm goes up the encoder ticks INCREASE for sure
 *
 * joysticks give negative numbers when you press them up, remember they were originally
 * designed as flight controls
 *
 * positive motor motion makes the arm go down for sure
 *
 * so we need to SetSensorPhase(false) to keep the direction sense of the encoder ticks
 *
 * we do NOT need to call SetInverted(true)
 *
 */


Arm::Arm()
: ComponentBase(ARM_TASKNAME, ARM_QUEUE, ARM_PRIORITY)
{
	//TODO: add member objects
	pArmMotor = new TalonSRX(CAN_ARM_TALON);
	pArmMotor->SetNeutralMode(NeutralMode::Brake);
	pArmMotor->Set(ControlMode::PercentOutput,0);
	pArmMotor->ConfigPeakCurrentLimit(30,1000);
	pArmMotor->ConfigContinuousCurrentLimit(30,1000);
	pArmMotor->ConfigPeakCurrentDuration(0.0,10);
	pArmMotor->EnableCurrentLimit(true);
	pArmMotor->SetInverted(true);
	pArmMotor->ConfigPeakOutputForward(0.65, 10);
	pArmMotor->ConfigPeakOutputReverse(-0.65,10);

	pArmMotor->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,10);
	pArmMotor->SetSensorPhase(false);
	pArmMotor->Config_kF(0,0.0,10);
	pArmMotor->Config_kP(0,0.325,10);
	pArmMotor->Config_kI(0,0.0001,10);
	pArmMotor->Config_kD(0,0.0,10);
	pArmMotor->SelectProfileSlot(0,0);
	pArmMotor->ConfigAllowableClosedloopError(0, 20, 10);
	pArmMotor->ConfigMaxIntegralAccumulator(1, 1000, 10);

	pBumperSwitch = new DigitalInput(ARM_BUMPER_SWITCH_SLOT);

	Wait(1.0);
	//	pArmMotor->Set(ControlMode::Position, pArmMotor->GetSelectedSensorPosition(0));
	//pArmMotor->Set(ControlMode::Velocity, 0.0);

	iCurrPos = pArmMotor->GetSelectedSensorPosition(0);
	iStartPos = iCurrPos;
	iFloorPos = iStartPos - iStartToOpen;
	iStowPos = iStartPos - iStartToStow;
	fCurVoltage = 0;
	fMotorSpeed = 0;
	fMaxSpeed = 0;
	iMoveDelta = 0;  // for now
	bTogglePressed = false;

	pArmTimeout = new Timer();

	pClawSolenoid = new Solenoid(CAN_PCM,1);
	pClawSolenoid->Set(false);

	SmartDashboard::PutNumber("Arm Speed RPM",((fMaxSpeed*600)/4096.0));
	SmartDashboard::PutNumber("Arm Position in Ticks",iCurrPos);
	SmartDashboard::PutNumber("Arm Position in Rotations",iCurrPos/4096.0);

	SmartDashboard::PutNumber("Arm Init Position",iStartPos);
	SmartDashboard::PutNumber("First Arm Open Position",iStartPos - iStartToOpen);
	SmartDashboard::PutNumber("Arm Shoot Position",iStartPos - iStartToShoot);

	//pArmMotor->Set(ControlMode::Position, iStartPos + iStartToShoot);

	bClawOpen = false;

	pTask = new std::thread(&Arm::StartTask, this, ARM_TASKNAME, ARM_PRIORITY);
	wpi_assert(pTask);
};

Arm::~Arm()
{
	delete(pArmTimeout);
	delete(pArmMotor);
	delete(pTask);
};

void Arm::OnStateChange()
{
};

void Arm::Run()
{
	iCurrPos = pArmMotor->GetSelectedSensorPosition(0);

	SmartDashboard::PutNumber("New Arm Floor Position",iFloorPos);

	SmartDashboard::PutNumber("Arm Speed RPM",((fMaxSpeed*600)/4096.0));
	SmartDashboard::PutNumber("Arm Position in Ticks",iCurrPos);
	SmartDashboard::PutNumber("Arm Position in Rotations",iCurrPos/4096.0);
	SmartDashboard::PutBoolean("Arm Bumper Switches",!(pBumperSwitch->Get()));
	iMoveDelta = 0;
	SmartDashboard::PutNumber("Arm Init Position",iStartPos);
	SmartDashboard::PutNumber("Arm Open Position",iStartPos - iStartToOpen);
	SmartDashboard::PutNumber("Arm Shoot Position",iStartPos - iStartToShoot);

		if (!(pBumperSwitch->Get()) && pArmMotor->GetSelectedSensorPosition(0) < iStowPos - 1024)
	{
		// If the bumper switch is activated and the arm has not reached or passed its initial floor position
		iFloorPos = pArmMotor->GetSelectedSensorPosition(0);
		pArmMotor->Set(ControlMode::Position,iFloorPos);
	}

	if (!(localMessage.command == COMMAND_CLAW_TOGGLE))
	{
		bTogglePressed = false;
	}

/*	if (pArmMotor->GetSelectedSensorPosition(0) < iFloorPos - 1024) {
		pArmMotor->Set(ControlMode::Position,iStowPos);
	} */

	switch(localMessage.command)			//Reads the message command
	{
	case COMMAND_COMPONENT_TEST:
		break;

	case COMMAND_ARM_MOVE:
		/*		fMotorSpeed = localMessage.params.arm.fArmSpeed;

		if (fMotorSpeed > 0.2)
		{
			iMoveDelta -= 10;
		}
		else if(fMotorSpeed < -0.2)
		{
			iMoveDelta += 10;
		} */
		break;

	case COMMAND_ARM_OPEN:
		/*		if( (iStartPos + iStartToOpen + iMoveDelta) > iStartToMax)
		{
			pArmMotor->Set(ControlMode::Position, iStartPos - iStartToMax);
		}
		else
		{ */
		//pArmMotor->Set(ControlMode::Position,iStartPos - iStartToOpen);// - iMoveDelta);
		//		}

/*		if (localMessage.params.arm.bFloorPos)
		{
			iFloorPos = ZeroArm();
		}
		else
		{ */
			pArmMotor->Set(ControlMode::Position,iFloorPos);
	//	}
		pArmTimeout->Reset();
		pArmTimeout->Start();
		break;

	case COMMAND_ARM_SHOOT:
		/*		if( (iStartPos - iStartToShoot - iMoveDelta) < iStartPos - iStartToMax)
		{
			pArmMotor->Set(ControlMode::Position,iStartPos - iStartToMax);
		}
		else
		{ */
		pArmMotor->Set(ControlMode::Position,iStowPos);// - iMoveDelta);
		//		}

		pArmTimeout->Reset();
		pArmTimeout->Start();
		break;

	case COMMAND_ARM_STOW:
		/*		if( (iStartPos - iStartToStow - iMoveDelta) < iStartPos - iStartToMax)
		{
			pArmMotor->Set(ControlMode::Position,iStartPos - iStartToMax);
		}
		else
		{ */
		pArmMotor->Set(ControlMode::Position,iStowPos);// - iMoveDelta);
		//		}

		pArmTimeout->Reset();
		pArmTimeout->Start();
		break;

	case COMMAND_ARM_FLOOR:
		/*		if( (iStartPos - iStartToOpen - iMoveDelta) < iStartPos - iStartToOpen)
		{
			pArmMotor->Set(ControlMode::Position, iStartPos -iStartToOpen);
		}
		else
		{ */
		//pArmMotor->Set(ControlMode::Position,iStartPos - iStartToOpen);// - iMoveDelta);
		//		}
/*		if (localMessage.params.arm.bFloorPos)
		{
			iFloorPos = ZeroArm();
		}
		else
		{ */
			pArmMotor->Set(ControlMode::Position,iFloorPos);
//		}
				pArmTimeout->Reset();
				pArmTimeout->Start();
		break;

	case COMMAND_CLAW_PINCH:
		pClawSolenoid->Set(false);
		break;

	case COMMAND_CLAW_RELEASE:
		pClawSolenoid->Set(true);
		break;

	case COMMAND_CLAW_TOGGLE:
		/*	if (!bTogglePressed && !bClawOpen)
		{
			bTogglePressed = true;
			pClawSolenoid->Set(true);
			bClawOpen = true;
		}
		else if (!bTogglePressed)
		{
			bTogglePressed = true;
			pClawSolenoid->Set(false);
			bClawOpen = false;
		}*/
		break;

	default:
		break;
	}

	// implement timeout

	SmartDashboard::PutNumber("Arm Closed Loop Error", pArmMotor->GetClosedLoopError(0));

	if((pArmMotor->GetClosedLoopError(0) < 50) && (pArmMotor->GetClosedLoopError(0) > -50))
	{
		// the servo has closed
		pArmTimeout->Reset();
		pArmTimeout->Stop();
	}
	else
	{
		if(pArmTimeout->Get() > 1.75)
		{
			// use the current position which should stop the servo
			pArmMotor->Set(ControlMode::Velocity, 0.0);
		}
	}

};

int Arm::ZeroArm() {
	pArmMotor->Set(ControlMode::Position,iStartPos - iStartToOpen);
	pArmTimeout->Reset();
	pArmTimeout->Start();
	while (pArmTimeout->Get() < 2.0 && pArmMotor->GetSelectedSensorPosition(0) < (iStartPos - iStartToOpen) + 512)
	{
		;
	}
	while (pArmTimeout->Get() < 2.0 && !pBumperSwitch->Get()) {
		pArmMotor->Set(ControlMode::PercentOutput,0.1);
	}
	pArmTimeout->Stop();
	pArmTimeout->Reset();
	return pArmMotor->GetSelectedSensorPosition(0);
}




