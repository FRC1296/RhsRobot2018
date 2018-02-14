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
	pController_1 = NULL;
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
}

void RhsRobot::Init() {
	/* 
	 * Set all pointers to null and then allocate memory and construct objects
	 * EXAMPLE:	drivetrain = NULL; (in constructor)
	 * 			drivetrain = new Drivetrain(); (in RhsRobot::Init())
	 */

	pController_1 = new Joystick(0);
	pDrivetrain = new Drivetrain();
	pClaw = new Claw();
	//SmartDashboard::PutData('M')
	pChooser = new frc::SendableChooser<char>();
	pChooser->AddDefault("Middle", 'M');
	pChooser->AddObject("Right",'R');
	pChooser->AddObject("Left",'L');

	SmartDashboard::PutData("Autonomous mode chooser", pChooser);

	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();
	/*
 if()
	{
		nextComponent = ComponentSet.insert(nextComponent, );
	}

	 */
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

	char sStart_Location = (char) pChooser->GetSelected();

	char c[2];
	c[0] = sStart_Location;
	c[1] = 0;

	SmartDashboard::PutString("Field Starting Position",c);
	std::__cxx11::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	if(gameData[0] == 'L')
	{
		//Put left auto code here
		SmartDashboard::PutString("Switch","Left");
	} else {
		//Put right auto code here
		SmartDashboard::PutString("Switch","Right");
	}

/*	if(WAVE_DASH)
	{
		pUpMotor->Set(ControlMode::PercentOutput,.75);

	}
	else
	{
		pUpMotor->Set(ControlMode::PercentOutput,0);
	}
*/

	if(pDrivetrain)
	{
		/*if(WAVE_DASH)
		{
			/*robotMessage.command  = COMMAND_DRIVETRAIN_WAVE;
			SmartDashboard::PutString("Mode","WAVE DASH");
			pDrivetrain->SendMessage(&robotMessage);*/
		/*
		}
		else*/ /* if(PIDGEY_ROTATE_LEFT90)
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
		*/
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
			robotMessage.params.mmove.fDistance = 30;
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
		}*/

	}

	if(pClaw)
	{

	}

	if((iLoop++ % 50) == 0)  // once every second or so
	{
		// send system health data to interested subsystems

		robotMessage.command = COMMAND_SYSTEM_CONSTANTS;
		robotMessage.params.system.fBattery = DriverStation::GetInstance().GetBatteryVoltage();
	}
}


HAL_Bool HAL_Initialize(int32_t timeout, int32_t mode);


START_ROBOT_CLASS(RhsRobot) // why is this giving an error Mr. B :(

