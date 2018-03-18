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
	pControllerDriver = NULL;
	pControllerOperator = NULL;
	pChooser = NULL;

	pAuto = NULL;
	pDrivetrain = NULL;
	pClaw = NULL;
	pElevator = NULL;
	pArm = NULL;
	pClimber = NULL;
	pPuncher = NULL;

	fHeightPercent = 0.0;
	fDrivetrainSpeed = 1.0;
	fExhaustLimit = 1.0;

	bLimitSpeedWhileElevatorIsUp = false;

	memset( &lastMessage, 0, sizeof(lastMessage));
	memset( &maxMessage, 0, sizeof(maxMessage));
	memset( &robotMessage, 0, sizeof(robotMessage));

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
	SmartDashboard::PutData("Autonomous Mode Chooser", pChooser);

	pSpeedTimer = new Timer();

	pControllerDriver = new Joystick(0);
	pControllerOperator = new Joystick(1);
	pCompressor = new Compressor(CAN_PCM);
	pCompressor->SetClosedLoopControl(true);
	pPDP = new PowerDistributionPanel(CAN_PDB);

	pDrivetrain = new Drivetrain();
	pClaw = new Claw();
	pElevator = new Elevator();
	pArm = new Arm();
	pAuto = new Autonomous();
	pClimber = new Climber();
	pPuncher = new Puncher();


	//camera = CameraServer::GetInstance()->StartAutomaticCapture();
	//camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 15);

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

	if(pArm)
	{
		nextComponent = ComponentSet.insert(nextComponent, pArm);
	}

	if(pAuto)
	{
		nextComponent = ComponentSet.insert(nextComponent, pAuto);
	}
	if(pClimber)
	{
		nextComponent = ComponentSet.insert(nextComponent, pClimber);
	}
	if(pPuncher)
	{
		nextComponent = ComponentSet.insert(nextComponent, pPuncher);
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

	fHeightPercent = pElevator->PercentHeight();
	fDrivetrainSpeed = (float)(1.0 / ((2*fHeightPercent) + 1));
	SmartDashboard::PutNumber("Elevator Switch2Scale Percent",fHeightPercent);

	SmartDashboard::PutNumber("Left Trigger",fLeftTrigger);
	SmartDashboard::PutNumber("Right Trigger",fRightTrigger);

	if((iLoop % 50) == 0)   // once every second or so
	{
		UpdateSystemData();
	}

	if((iLoop++ % 5) == 0)  // ten times per second or so
	{
		MonitorPDB();
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
			robotMessage.params.adrive.left = (ARCADE_DRIVE_LEFT * ARCADE_DRIVE_LEFT * ARCADE_DRIVE_LEFT);
			robotMessage.params.adrive.right = (ARCADE_DRIVE_RIGHT * ARCADE_DRIVE_RIGHT * ARCADE_DRIVE_RIGHT);
			pDrivetrain->SendMessage(&robotMessage);
		}
#endif

		/*		if(bLimitSpeedWhileElevatorIsUp)
		{
			robotMessage.params.cheesyDrive.wheel = CHEESY_DRIVE_WHEEL / 2.25;
			robotMessage.params.cheesyDrive.throttle = CHEESY_DRIVE_THROTTLE / 2.5;
			robotMessage.params.cheesyDrive.bQuickturn = CHEESY_DRIVE_QUICKTURN;
		}
		else
		{
			robotMessage.params.cheesyDrive.wheel = CHEESY_DRIVE_WHEEL / 1.75;
			robotMessage.params.cheesyDrive.throttle = CHEESY_DRIVE_THROTTLE / 1.0;
			robotMessage.params.cheesyDrive.bQuickturn = CHEESY_DRIVE_QUICKTURN;
		} */

		// Testing for new PID values
/*		if(PIDGEY_ROTATE_GPTURN)
		{
			robotMessage.params.turn.fAngle = 90;
			robotMessage.command = COMMAND_DRIVETRAIN_GPTURN;
			SmartDashboard::PutString("cmd","PID Turn Called");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else if(DRIVETRAIN_MMOVE)
		{
			robotMessage.params.mmove.fDistance = -1*(36*2);
			robotMessage.params.mmove.fTime = 3;
			robotMessage.command = COMMAND_DRIVETRAIN_MMOVE;
			SmartDashboard::PutString("cmd","PID Move Called");
			pDrivetrain->SendMessage(&robotMessage);
		}
		else
		{*/
		robotMessage.params.cheesyDrive.wheel = CHEESY_DRIVE_WHEEL / 1.75;
		robotMessage.params.cheesyDrive.throttle = (CHEESY_DRIVE_THROTTLE * fDrivetrainSpeed);
		robotMessage.params.cheesyDrive.bQuickturn = CHEESY_DRIVE_QUICKTURN;

		robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_CHEESY;
		pDrivetrain->SendMessage(&robotMessage);
		//		}
		// delete after we link in cheesy libraries

		//robotMessage.command  = COMMAND_DRIVETRAIN_RUN_ARCADE;
		//			robotMessage.params.adrive.left = (ARCADE_DRIVE_LEFT * ARCADE_DRIVE_LEFT * ARCADE_DRIVE_LEFT);
		//			robotMessage.params.adrive.right = (ARCADE_DRIVE_RIGHT * ARCADE_DRIVE_RIGHT * ARCADE_DRIVE_RIGHT);
		//			pDrivetrain->SendMessage(&robotMessage);
	}

	if(pClaw)
	{
		if(CLAW_INHALE > .1)// Intake cube
		{
			robotMessage.command = COMMAND_CLAW_INHALE;
			robotMessage.params.claw.fClawSpeed = CLAW_INHALE;
			pClaw->SendMessage(&robotMessage);
		}
		else if(CLAW_EXHALE > .1)// Exhaust cube
		{
			robotMessage.command = COMMAND_CLAW_EXHALE;
			robotMessage.params.claw.fClawSpeed = (CLAW_EXHALE * fExhaustLimit);
			pClaw->SendMessage(&robotMessage);
		}
		/*
		else if (CLAW_EXHALE >= 0.1 && CLAW_EXHALE < 0.7) {
			robotMessage.command = COMMAND_CLAW_EXHALE;
			robotMessage.params.claw.fClawSpeed = 0.3;
			pClaw->SendMessage(&robotMessage);
		}
		else if (CLAW_EXHALE >= 0.7) {
			robotMessage.command = COMMAND_CLAW_EXHALE;
			robotMessage.params.claw.fClawSpeed = CLAW_EXHALE;
			pClaw->SendMessage(&robotMessage);
		}
		 */
		else// Slowly rotate intake in to hold cube
		{
			robotMessage.command = COMMAND_CLAW_STOP;
			robotMessage.params.claw.fClawSpeed = 0.0;
			pClaw->SendMessage(&robotMessage);
		}
	}

	if(pArm)
	{
		if(CLAW_PINCH)
		{
			robotMessage.command = COMMAND_CLAW_PINCH;
			pArm->SendMessage(&robotMessage);
		}
		else if(CLAW_RELEASE)
		{
			robotMessage.command = COMMAND_CLAW_RELEASE;
			pArm->SendMessage(&robotMessage);
		}
/*		if(CLAW_TOGGLE)
		{
			robotMessage.command = COMMAND_CLAW_TOGGLE;
			pArm->SendMessage(&robotMessage);
		} */
		if(CLAW_MOVE > .5)
		{
			robotMessage.command = COMMAND_ARM_FLOOR;
			pArm->SendMessage(&robotMessage);
		}
		if(CLAW_MOVE <-.5)
		{
			robotMessage.command = COMMAND_ARM_SHOOT;
			pArm->SendMessage(&robotMessage);
		}
	}

	if(pElevator)
	{
		SmartDashboard::PutNumber("Raw Elevator Axis",ELEVATOR_DELTA);

		if (ELEVATOR_SWITCH)
		{
			robotMessage.command = COMMAND_ELEVATOR_SWITCH;
			pElevator->SendMessage(&robotMessage);
//			robotMessage.command = COMMAND_ARM_FLOOR;
//			pArm->SendMessage(&robotMessage);
			bLimitSpeedWhileElevatorIsUp = false;
			fExhaustLimit = .75;
		}
		else if (ELEVATOR_SCALE)
		{
			robotMessage.command = COMMAND_ELEVATOR_SCALE;
			pElevator->SendMessage(&robotMessage);
//			robotMessage.command = COMMAND_ARM_FLOOR;
//			pArm->SendMessage(&robotMessage);
			bLimitSpeedWhileElevatorIsUp = true;
			pSpeedTimer->Reset();
			pSpeedTimer->Stop();
			fExhaustLimit = .6;
		}
		else if(ELEVATOR_FLOOR)
		{
			robotMessage.command = COMMAND_ELEVATOR_FLOOR;
			pElevator->SendMessage(&robotMessage);
			pSpeedTimer->Reset();
			pSpeedTimer->Start();
			fExhaustLimit = 1.0;
		}
		else
		{
			robotMessage.command = COMMAND_ELEVATOR_NOBUTTON;
			pElevator->SendMessage(&robotMessage);
			pSpeedTimer->Reset();
			pSpeedTimer->Start();
			fExhaustLimit = 1.0;
		}

		if ((ELEVATOR_DELTA > .2) || (ELEVATOR_DELTA < -.2))
		{
			robotMessage.command = COMMAND_ELEVATOR_MOVE;
			robotMessage.params.elevator.fSpeed = ELEVATOR_DELTA;
			pElevator->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command = COMMAND_ELEVATOR_MOVE;
			robotMessage.params.elevator.fSpeed = 0;
			pElevator->SendMessage(&robotMessage);
		}
	}

	if (pClimber)
	{
		if (CLIMBER_UP)
		{
			robotMessage.command = COMMAND_CLIMB_UP;
			robotMessage.params.climb.fClimbSpeed = CLIMBER_SPEED;
			pClimber->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command = COMMAND_CLIMB_STOP;
			robotMessage.params.climb.fClimbSpeed = 0.0;
			pClimber->SendMessage(&robotMessage);
		}
	}

	if (pSpeedTimer->Get() >= 2.0)
	{
		pSpeedTimer->Stop();
		bLimitSpeedWhileElevatorIsUp = false;
	}
	/* If the timer no longer works for some reason
	 * if (pElevator->LimitSpeed())
	 * {
	 * 		bLimitSpeedWhileElevatorIsUp = true;
	 * }
	 * else
	 * { // Perhaps do the timer here instead of changing it back but with a shorter timeout?
	 *   // Alternatively we could do some sort of ramping here
	 * 		bLimitSpeedWhileElevatorIsUp = false;
	 * }
	 */
}

void RhsRobot::MonitorPDB(void)
{
	robotMessage.command = COMMAND_SYSTEM_PDBDATA;
	robotMessage.params.pdb.lclaw = pPDP->GetCurrent(PDB_CLAW_CHANNEL_ONE);
	robotMessage.params.pdb.rclaw = pPDP->GetCurrent(PDB_CLAW_CHANNEL_TWO);

	if(pClaw)
	{
		robotMessage.params.pdb.lclaw = pPDP->GetCurrent(PDB_CLAW_CHANNEL_ONE);
		robotMessage.params.pdb.rclaw = pPDP->GetCurrent(PDB_CLAW_CHANNEL_TWO);
		pClaw->SendMessage(&robotMessage);
	}

	if(pDrivetrain)
	{
		robotMessage.params.pdb.ldrive1 = pPDP->GetCurrent(PDB_DRIVE_CHANNEL_L1);
		robotMessage.params.pdb.ldrive2 = pPDP->GetCurrent(PDB_DRIVE_CHANNEL_L2);
		robotMessage.params.pdb.ldrive3 = pPDP->GetCurrent(PDB_DRIVE_CHANNEL_L3);
		robotMessage.params.pdb.rdrive1 = pPDP->GetCurrent(PDB_DRIVE_CHANNEL_R1);
		robotMessage.params.pdb.rdrive2 = pPDP->GetCurrent(PDB_DRIVE_CHANNEL_R2);
		robotMessage.params.pdb.rdrive3 = pPDP->GetCurrent(PDB_DRIVE_CHANNEL_R3);
		pDrivetrain->SendMessage(&robotMessage);
	}

	lastMessage = robotMessage;

	if(lastMessage.params.pdb.lclaw > maxMessage.params.pdb.lclaw)
	{
		maxMessage.params.pdb.lclaw = lastMessage.params.pdb.lclaw;
	}

	if(lastMessage.params.pdb.rclaw > maxMessage.params.pdb.rclaw)
	{
		maxMessage.params.pdb.rclaw = lastMessage.params.pdb.rclaw;
	}

	if(lastMessage.params.pdb.ldrive1 > maxMessage.params.pdb.ldrive1)
	{
		maxMessage.params.pdb.ldrive1 = lastMessage.params.pdb.ldrive1;
	}

	if(lastMessage.params.pdb.ldrive2 > maxMessage.params.pdb.ldrive2)
	{
		maxMessage.params.pdb.ldrive2 = lastMessage.params.pdb.ldrive2;
	}

	if(lastMessage.params.pdb.ldrive3 > maxMessage.params.pdb.ldrive3)
	{
		maxMessage.params.pdb.ldrive3 = lastMessage.params.pdb.ldrive3;
	}

	if(lastMessage.params.pdb.rdrive1 > maxMessage.params.pdb.rdrive1)
	{
		maxMessage.params.pdb.rdrive1 = lastMessage.params.pdb.rdrive1;
	}

	if(lastMessage.params.pdb.rdrive2 > maxMessage.params.pdb.rdrive2)
	{
		maxMessage.params.pdb.rdrive2 = lastMessage.params.pdb.rdrive2;
	}

	if(lastMessage.params.pdb.rdrive3 > maxMessage.params.pdb.rdrive3)
	{
		maxMessage.params.pdb.rdrive3 = lastMessage.params.pdb.rdrive3;
	}

	SmartDashboard::PutNumber("Claw Motor Left Max Amps", maxMessage.params.pdb.lclaw);
	SmartDashboard::PutNumber("Claw Motor Right Max Amps", maxMessage.params.pdb.rclaw);
	SmartDashboard::PutNumber("Drive Motor Left #1 Max Amps", maxMessage.params.pdb.ldrive1);
	SmartDashboard::PutNumber("Drive Motor Left #2 Max Amps", maxMessage.params.pdb.ldrive2);
	SmartDashboard::PutNumber("Drive Motor Left #3 Max Amps", maxMessage.params.pdb.ldrive3);
	SmartDashboard::PutNumber("Drive Motor Right #1 Max Amps", maxMessage.params.pdb.rdrive1);
	SmartDashboard::PutNumber("Drive Motor Right #2 Max Amps", maxMessage.params.pdb.rdrive2);
	SmartDashboard::PutNumber("Drive Motor Right #3 Max Amps", maxMessage.params.pdb.rdrive3);

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

