/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands sent to the subsystems
 * that implement behaviors for each part for the robot.
 */

#include "ComponentBase.h"
#include "RhsRobot.h"
#include "RobotParams.h"
#include "WPILib.h"

// The constructor sets the pointer to our objects to NULL.  We use pointers so we
// can control when the objects are instantiated.  It looks kinda old school but is
// standard practice in embedded systems.

RhsRobot::RhsRobot() {

	// set new object pointers to NULL here
	pControllerDriver = NULL;
	pControllerOperator = NULL;
	pChooser = NULL;

	pAuto = NULL;
	pDrivetrain = NULL;
	pClaw = NULL;
	pElevator = NULL;

	iLoop = 0;            // a helpful little loop counter
}

// This will never get used but we make it functional anyways.  We iterate through our objects
// (with messaging) then delete the system objects.

RhsRobot::~RhsRobot() {
	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();

	for(; nextComponent != ComponentSet.end(); ++nextComponent)
	{
		delete (*nextComponent);
	}

	// delete other system objects here (but not our message-based objects)

	delete pControllerDriver;
	delete pControllerOperator;
}

void RhsRobot::Init() {
	/* 
	 * Set all pointers to null and then allocate memory and construct objects
	 * EXAMPLE:	drivetrain = NULL; (in constructor)
	 * 			drivetrain = new Drivetrain(); (in RhsRobot::Init())
	 */

	pChooser = new frc::SendableChooser<char>();
	pChooser->AddObject("Left",'L');
	pChooser->AddObject("Center",'C');
	pChooser->AddObject("Right",'R');
	pChooser->AddDefault("Simple", 'X');
	SmartDashboard::PutData("Autonomous mode chooser", pChooser);

	pControllerDriver = new Joystick(0);
	pControllerOperator = new Joystick(1);
	//pCompressor = new Compressor(CAN_PCM);
	//pCompressor->SetClosedLoopControl(true);

	pDrivetrain = new Drivetrain();
	pClaw = new Claw();
	pAuto = new Autonomous();
	pElevator = new Elevator();

	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();

	if(pDrivetrain)
	{
		nextComponent = ComponentSet.insert(nextComponent, pDrivetrain);
	}

	if(pClaw)
	{
		nextComponent = ComponentSet.insert(nextComponent, pClaw);
	}

	if(pElevator)
	{
		nextComponent = ComponentSet.insert(nextComponent, pElevator);
	}

	if(pAuto)
	{
		nextComponent = ComponentSet.insert(nextComponent, pAuto);
	}

	// instantiate our other objects here
}


// this method iterates through all our objects (in our message infrastructure) and sends
// them a message, it is used mostly for telling every object the robot state has changed

void RhsRobot::OnStateChange() {
	std::vector<ComponentBase *>::iterator nextComponent;

	for(nextComponent = ComponentSet.begin();
			nextComponent != ComponentSet.end(); ++nextComponent)
	{
		(*nextComponent)->SendMessage(&robotMessage);
	}
}

// this method is where the magic happens.  It is called every time we get a new message from th driver station

void RhsRobot::Run() {
	/* Poll for control data and send messages to each subsystem. Surround blocks with if(component) so entire components can be disabled
	 * by commenting out their construction.
	 * EXAMPLE: if(pDrivetrain)
	 * 			{ 
	 * 				// Check joysticks and send messages
					robotMessage.command = COMMAND_GEARINTAKE_HOLD;
					robotMessage.params.gear.GearHold = 1.0;
					pDrivetrail->SendMessage(&robotMessage);
	 * 			}
	 */

	fLeftTrigger = pControllerDriver->GetRawAxis(L310_TRIGGER_LEFT);
	fRightTrigger = pControllerDriver->GetRawAxis(L310_TRIGGER_RIGHT);

	SmartDashboard::PutNumber("Left Trigger",fLeftTrigger);
	SmartDashboard::PutNumber("Right Trigger",fRightTrigger);

	if((iLoop++ % 50) == 0)  // once every second or so
	{
		UpdateSystemData();
	}

	UpdateGameData();

	if(pAuto)
	{
		if(GetCurrentRobotState() == ROBOT_STATE_AUTONOMOUS)
		{
			// all messages to components will come from the autonomous task

			return;
		}
	}

	if(pDrivetrain)
	{
#if 0
		if(WAVE_DASH)
		{
			robotMessage.command  = COMMAND_DRIVETRAIN_WAVE;
			SmartDashboard::PutString("Mode","WAVE DASH");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else if(PIDGEY_ROTATE_LEFT90)
		{
			robotMessage.params.turn.fAngle = 90;
			robotMessage.command = COMMAND_DRIVETRAIN_GPTURN;
			SmartDashboard::PutString("cmd","Left 90 PID Called");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else if(PIDGEY_ROTATE_RIGHT90)
		{
			robotMessage.params.turn.fAngle = -90;
			robotMessage.command = COMMAND_DRIVETRAIN_GPTURN;
			SmartDashboard::PutString("cmd","Right 90 PID Called");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else if(PIDGEY_ROTATE_180)
		{
			robotMessage.params.turn.fAngle = 180;
			robotMessage.command = COMMAND_DRIVETRAIN_GPTURN;
			SmartDashboard::PutString("cmd","180 PID Called");
			pDrivetrain->SendMessage(&robotMessage);
		}

		if (DRIVETRAIN_BOXFILTER)
		{
			robotMessage.params.turn.fAngle = 90;
			robotMessage.command = COMMAND_DRIVETRAIN_BOXFILTER;
			SmartDashboard::PutString("cmd","Box Filter Called");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else if (DRIVETRAIN_MTURN)
		{
			robotMessage.params.turn.fAngle = 90;
			robotMessage.command = COMMAND_DRIVETRAIN_MTURN;
			SmartDashboard::PutString("cmd","Measured Turn Called");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else if(PIDGEY_ROTATE_GPTURN)
		{
			robotMessage.params.turn.fAngle = 90;
			robotMessage.command = COMMAND_DRIVETRAIN_GPTURN;
			SmartDashboard::PutString("cmd","PID Turn Called");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else if(DRIVETRAIN_MMOVE)
		{
			robotMessage.params.mmove.fDistance = 36*2;
			robotMessage.params.mmove.fTime = 3;
			robotMessage.command = COMMAND_DRIVETRAIN_MMOVE;
			SmartDashboard::PutString("cmd","PID Move Called");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command  = COMMAND_DRIVETRAIN_RUN_ARCADE;
			robotMessage.params.adrive.left = TANK_DRIVE_LEFT;
			robotMessage.params.adrive.right = CHEEZY_DRIVE_WHEEL;
			//robotMessage.params.adrive.left = (ARCADE_DRIVE_LEFT * ARCADE_DRIVE_LEFT * ARCADE_DRIVE_LEFT);
			//robotMessage.params.adrive.right = (ARCADE_DRIVE_RIGHT * ARCADE_DRIVE_RIGHT * ARCADE_DRIVE_RIGHT);
			pDrivetrain->SendMessage(&robotMessage);
		}
#endif
		robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_CHEESY;
		 			robotMessage.params.cheesyDrive.wheel = CHEESY_DRIVE_WHEEL;
		 			robotMessage.params.cheesyDrive.throttle = CHEESY_DRIVE_THROTTLE;
		 			robotMessage.params.cheesyDrive.bQuickturn = CHEESY_DRIVE_QUICKTURN;
		 			pDrivetrain->SendMessage(&robotMessage);

		// delete after we link in cheesy libraries

		robotMessage.command  = COMMAND_DRIVETRAIN_RUN_ARCADE;
					robotMessage.params.adrive.left = (ARCADE_DRIVE_LEFT * ARCADE_DRIVE_LEFT * ARCADE_DRIVE_LEFT);
					robotMessage.params.adrive.right = (ARCADE_DRIVE_RIGHT * ARCADE_DRIVE_RIGHT * ARCADE_DRIVE_RIGHT);
					pDrivetrain->SendMessage(&robotMessage);

	}

	if(pClaw)
	{
		if(CLAW_INHALE > .1)
		{
			robotMessage.command = COMMAND_CLAW_INHALE;
			robotMessage.params.claw.fClawSpeed = CLAW_INHALE;
			pClaw->SendMessage(&robotMessage);
		}
		else if(CLAW_EXHALE > .1)
		{
			robotMessage.command = COMMAND_CLAW_EXHALE;
			robotMessage.params.claw.fClawSpeed = CLAW_EXHALE;
			pClaw->SendMessage(&robotMessage);
		}
		else if(CLAW_PINCH)
		{
			robotMessage.command = COMMAND_CLAW_PINCH;
			pClaw->SendMessage(&robotMessage);
		}
		else if(CLAW_RELEASE)
		{
			robotMessage.command = COMMAND_CLAW_RELEASE;
			pClaw->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command = COMMAND_CLAW_STOP;
			robotMessage.params.claw.fClawSpeed = 0.0;
			pClaw->SendMessage(&robotMessage);
		}
	}

	if(pElevator)
	{
		SmartDashboard::PutNumber("Raw Elevator Axis",ELEVATOR);
		if (ELEVATOR_SWITCH)
		{
			robotMessage.command = COMMAND_ELEVATOR_SWITCH;
			pElevator->SendMessage(&robotMessage);
		}
		else if (ELEVATOR_FLOOR)
		{
			robotMessage.command = COMMAND_ELEVATOR_FLOOR;
			pElevator->SendMessage(&robotMessage);
		}
		else if (ELEVATOR_SCALE)
		{
			robotMessage.command = COMMAND_ELEVATOR_SCALE;
			pElevator->SendMessage(&robotMessage);
		}
		else if (ELEVATOR > .2 || ELEVATOR < -.2)
		{
			robotMessage.command = COMMAND_ELEVATOR_MOVE;
			robotMessage.params.elevator.fSpeed = ELEVATOR;
			pElevator->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command = COMMAND_ELEVATOR_MOVE;
			robotMessage.params.elevator.fSpeed = 0;
			pElevator->SendMessage(&robotMessage);
		}
	}
}

void RhsRobot::UpdateGameData(void)
{
	// if the starting position has changed or we get new

	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	if(gameData.length() == 0)
	{
		return;
	}

	sStartLocation = (char) pChooser->GetSelected();

	if((gameData != gameDataPrev) || (sStartLocation != sStartLocationLast))
	{
		robotMessage.command = COMMAND_SYSTEM_GAMEDATA;

		if(gameData[0] == 'L')
		{
			robotMessage.params.gamedata.eSwitchSide = GAMEPIECESIDE_LEFT;
			SmartDashboard::PutBoolean("Switch Left", true);
			SmartDashboard::PutBoolean("Switch Right", false);
		}
		else if (gameData[0] == 'R')
		{
			robotMessage.params.gamedata.eSwitchSide = GAMEPIECESIDE_RIGHT;
			SmartDashboard::PutBoolean("Switch Left", false);
			SmartDashboard::PutBoolean("Switch Right", true);
		}
		else
		{
			robotMessage.params.gamedata.eSwitchSide = GAMEPIECESIDE_LEFT;
			SmartDashboard::PutBoolean("Switch Left", true);
			SmartDashboard::PutBoolean("Switch Right", false);
		}

		if(gameData[1] == 'L')
		{
			robotMessage.params.gamedata.eScaleSide = GAMEPIECESIDE_LEFT;
			SmartDashboard::PutBoolean("Scale Left", true);
			SmartDashboard::PutBoolean("Scale Right", false);
		}
		else if(gameData[1] == 'R')
		{
			robotMessage.params.gamedata.eScaleSide = GAMEPIECESIDE_RIGHT;
			SmartDashboard::PutBoolean("Scale Left", false);
			SmartDashboard::PutBoolean("Scale Right", true);
		}
		else
		{
			robotMessage.params.gamedata.eScaleSide = GAMEPIECESIDE_LEFT;
			SmartDashboard::PutBoolean("Scale Left", true);
			SmartDashboard::PutBoolean("Scale Right", false);
		}

		if(gameData[2] == 'L')
		{
			robotMessage.params.gamedata.eOpponentSwitchSide = GAMEPIECESIDE_LEFT;
			SmartDashboard::PutBoolean("Opponent Left", true);
			SmartDashboard::PutBoolean("Opponent Right", false);
		}
		else if(gameData[2] == 'R')
		{
			robotMessage.params.gamedata.eOpponentSwitchSide = GAMEPIECESIDE_RIGHT;
			SmartDashboard::PutBoolean("Opponent Left", false);
			SmartDashboard::PutBoolean("Opponent Right", true);
		}
		else
		{
			robotMessage.params.gamedata.eOpponentSwitchSide = GAMEPIECESIDE_LEFT;
			SmartDashboard::PutBoolean("Opponent Left", true);
			SmartDashboard::PutBoolean("Opponent Right", false);
		}

		if(sStartLocation == 'L')
		{
			robotMessage.params.gamedata.eStartingPosition = GAMEPIECESTART_LEFT;
			SmartDashboard::PutBoolean("Start Left", true);
			SmartDashboard::PutBoolean("Start Middle", false);
			SmartDashboard::PutBoolean("Start Right", false);
		}
		else if(sStartLocation == 'C')
		{
			robotMessage.params.gamedata.eStartingPosition = GAMEPIECESTART_CENTER;
			SmartDashboard::PutBoolean("Start Left", false);
			SmartDashboard::PutBoolean("Start Middle", true);
			SmartDashboard::PutBoolean("Start Right", false);
		}
		else if(sStartLocation == 'R')
		{
			robotMessage.params.gamedata.eStartingPosition = GAMEPIECESTART_RIGHT;
			SmartDashboard::PutBoolean("Start Left", false);
			SmartDashboard::PutBoolean("Start Middle", false);
			SmartDashboard::PutBoolean("Start Right", true);
		}
		else
		{
			robotMessage.params.gamedata.eStartingPosition = GAMEPIECESTART_SIMPLE;
		}

		// the game data has changed so tell everyone who is interested

		if(pAuto)
		{
			pAuto->SendMessage(&robotMessage);
		}

		if(pDrivetrain)
		{
			pDrivetrain->SendMessage(&robotMessage);
		}

		gameDataPrev = gameData;
		sStartLocationLast = sStartLocation;
		SmartDashboard::PutData("Autonomous mode chooser", pChooser);
	}
}

void RhsRobot::UpdateSystemData(void)
{
	// send system health data to interested subsystems

	robotMessage.command = COMMAND_SYSTEM_CONSTANTS;
	robotMessage.params.system.fBattery = frc::DriverStation::GetInstance().GetBatteryVoltage();

	if(pDrivetrain)
	{
		pDrivetrain->SendMessage(&robotMessage);
	}

	SmartDashboard::PutNumber("Match Time", frc::DriverStation::GetInstance().GetMatchTime());
}


HAL_Bool HAL_Initialize(int32_t timeout, int32_t mode);


START_ROBOT_CLASS(RhsRobot) // why is this giving an error Mr. B :(

