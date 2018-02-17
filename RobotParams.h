/** \file
 *  Defines task parameters, hardware assignments and controller button/axis assignment.
 *
 * This header contains basic parameters for the robot. All parameters must be constants with internal
 * linkage, otherwise the One Definition Rule will be violated.
 */

// TODO: please go over these items with a knowledgeable mentor and check to see what we need/don't need
#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

//Robot
#include "JoystickLayouts.h"			//For joystick layouts
#include "ctre/Phoenix.h"

//Robot Params
const char* const ROBOT_NAME =		"RhsRobot2018";	//Formal name
const char* const ROBOT_NICKNAME =  "Unknown";		//Nickname
const char* const ROBOT_VERSION =	"0.5";			//Version

//Utility Functions - Define commonly used operations here
#define ABLIMIT(a,b)		if(a > b) a = b; else if(a < -b) a = -b;
#define TRUNC_THOU(a)		((int)(1000 * a)) * .001
#define TRUNC_HUND(a)		((int)(100 * a)) * .01
#define PRINTAUTOERROR		printf("Early Death! %s %i \n", __FILE__, __LINE__);

//Task Params - Defines component task priorites relative to the default priority.
//EXAMPLE: const int DRIVETRAIN_PRIORITY = DEFAULT_PRIORITY -2;
const int DEFAULT_PRIORITY      = 50;
const int COMPONENT_PRIORITY 	= DEFAULT_PRIORITY;
const int DRIVETRAIN_PRIORITY 	= DEFAULT_PRIORITY;
const int PIXI_PRIORITY 	    = DEFAULT_PRIORITY;
const int AUTONOMOUS_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTOEXEC_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTOPARSER_PRIORITY 	= DEFAULT_PRIORITY;
const int CLAW_PRIORITY 		= DEFAULT_PRIORITY;
const int ELEVATOR_PRIORITY		= DEFAULT_PRIORITY;
const int CLIMBER_PRIORITY		= DEFAULT_PRIORITY;

//Task Names - Used when you view the task list but used by the operating system
//EXAMPLE: const char* DRIVETRAIN_TASKNAME = "tDrive";
const char* const COMPONENT_TASKNAME	= "tComponent";
const char* const DRIVETRAIN_TASKNAME	= "tDrive";
const char* const AUTONOMOUS_TASKNAME	= "tAuto";
const char* const AUTOEXEC_TASKNAME		= "tAutoEx";
const char* const AUTOPARSER_TASKNAME	= "tParse";
const char* const CLAW_TASKNAME 		= "tClaw";
const char* const ELEVATOR_TASKNAME		= "tElevator";
const char* const CLIMBER_TASKNAME		= "tClimber";

//TODO change these variables throughout the code to PIPE or whatever instead  of QUEUE
//Queue Names - Used when you want to open the message queue for any task
//NOTE: 2015 - we use pipes instead of queues
//EXAMPLE: const char* DRIVETRAIN_TASKNAME = "tDrive";

const char* const COMPONENT_QUEUE 	= "/tmp/qComp";
const char* const DRIVETRAIN_QUEUE 	= "/tmp/qDrive";
const char* const AUTONOMOUS_QUEUE 	= "/tmp/qAuto";
const char* const AUTOPARSER_QUEUE 	= "/tmp/qParse";
const char* const CLAW_QUEUE 		= "/tmp/qClaw";
const char* const ELEVATOR_QUEUE	= "/tmp/qElevator";
const char* const CLIMBER_QUEUE		= "/tmp/qClimber";

//PWM Channels - Assigns names to PWM ports 1-10 on the Roborio
//EXAMPLE: const int PWM_DRIVETRAIN_FRONT_LEFT_MOTOR = 1;

const int PWM_DRIVETRAIN_LEFT_MOTOR = 0;
const int PWM_DRIVETRAIN_RIGHT_MOTOR = 1;

//CAN IDs - Assigns names to the various CAN IDs
//EXAMPLE: const int CAN_PDB = 0;
/** \page motorID Motor Controller IDs
 * \verbatim
0 - PDB
1 - left drive motor
2 - right drive motor
Add more as needed.
 \endverbatim
 */

// TODO: Delegate TalonSRX numbers

const int CAN_PCM = 17;
const int CAN_PIGEON = 18;
const int CAN_PDB = 19;

// Had to change these, lowkey salty at mechanical ~ Jiff
const int CAN_DRIVETRAIN_TALON_LEFT = 4;
const int CAN_DRIVETRAIN_VICTOR_LEFT1 = 5;
const int CAN_DRIVETRAIN_VICTOR_LEFT2 = 6;
const int CAN_DRIVETRAIN_TALON_RIGHT = 1;
const int CAN_DRIVETRAIN_VICTOR_RIGHT1 = 2;
const int CAN_DRIVETRAIN_VICTOR_RIGHT2 = 3;

const int CAN_CLIMBER_TALON = 1; // Arbitrary Numbers until delegated
const int CAN_CLIMBER_VICTOR = -1; // Arbitrary Numbers until delegated
const int CAN_CLAW_VICTOR_LEFT = -1; // Arbitrary Numbers until delegated
const int CAN_CLAW_VICTOR_RIGHT = -1; // See above
const int CAN_ELEVATOR_TALON_LEFT = -1; // See above
const int CAN_ELEVATOR_TALON_RIGHT = -1; // See above


//Relay Channels - Assigns names to Relay ports 1-8 on the roboRio
//EXAMPLE: const int RLY_COMPRESSOR = 1;

//Digital I/O - Assigns names to Digital I/O ports 1-14 on the roboRio
//EXAMPLE: const int DIO_DRIVETRAIN_BEAM_BREAK = 0;


//Solenoid - Assigns names to Solenoid ports 1-8 on the roboRio
//EXAMPLE: const int SOL_DRIVETRAIN_SOLENOID_SHIFT_IN = 1;

//I2C - Assigns names to I2C ports 1-2 on the Roborio
//EXAMPLE: const int IO2C_AUTO_ACCEL = 1;

//Analog I/O - Assigns names to Analog I/O ports 1-8 on the roboRio
//EXAMPLE: const int AIO_BATTERY = 8;


//Relay I/O - Assigns names to Realy I/O ports 1-8 on the roboRio
//EXAMPLE: const int RELAY_LED = 0;


//Joystick Input Device Counts - used by the listener to watch buttons and axis
const int JOYSTICK_BUTTON_COUNT = 10;
const int JOYSTICK_AXIS_COUNT = 5;

//POV IDs - Assign names to the 9 POV positions: -1 to 7
//EXAMPLE: const int POV_STILL = -1;
const int POV_STILL = -1;

//Primary Controller Mapping - Assigns action to buttons or axes on the first joystick
#undef	USE_X3D_FOR_CONTROLLER_1
#undef	USE_XBOX_FOR_CONTROLLER_1
#define	USE_L310_FOR_CONTROLLER_1

//Secondary Controller Mapping - Assigns action to buttons or axes on the second joystick
#undef	USE_X3D_FOR_CONTROLLER_2
#undef 	USE_XBOX_FOR_CONTROLLER_2
#define USE_L310_FOR_CONTROLLER_2

#ifdef USE_XBOX_FOR_CONTROLLER_1
#endif
/** \page joysticks Joystick Layouts
 * \verbatim
 	 +++++ Controller 1 +++++
  	A Button					Toggle noodle fan
  	B Button					~~
  	X Button					Hold Cube clicker at bottom to remove totes
  	Y Button					Release Cube clicker from hold
  	Start Button				Start Cube autocycle
  	Back Button					Stop Cube autocycle
  	Left Bumper					Run Conveyor forward
  	Right Bumper				Run Conveyor backwards - to claw
  	Left Thumbstick Button		Close CanLifter claw
  	Right Thumbstick Button		Open CanLifter claw
  	Left Thumbstick				Left tank, Arcade
  	Right Thumbstick			Right tank
  	D-pad						~~
  	Left Trigger				Lower CanLifter
  	RightTrigger				Raise CanLifter

 	 +++++ Controller 2 +++++
  	A Button					~~
  	B Button					~~
  	X Button					Hold Cube clicker at bottom to remove totes
  	Y Button					Release Cube clicker from hold
  	Start Button				Start Cube autocycle
  	Back Button					Stop Cube autocycle
  	Left Bumper					~~
  	Right Bumper				~~
 	Left Thumbstick Button		~~
  	Right Thumbstick Button		~~
  	Left Thumbstick				~~
  	Right Thumbstick			Raise/lower Cube clicker
  	D-pad						~~
  	Left Trigger				~~
  	RightTrigger				~~
 \endverbatim
 */
#ifdef USE_L310_FOR_CONTROLLER_1

#define TANK_DRIVE_LEFT				(pControllerDriver->GetRawAxis(L310_THUMBSTICK_LEFT_Y))
#define TANK_DRIVE_RIGHT			(-pControllerDriver->GetRawAxis(L310_THUMBSTICK_RIGHT_Y))
#define CHEEZY_DRIVE_WHEEL			(pControllerDriver->GetRawAxis(L310_THUMBSTICK_RIGHT_X))
#define CHEEZY_DRIVE_THROTTLE		(-pControllerDriver->GetRawAxis(L310_THUMBSTICK_LEFT_Y))
#define CHEEZY_DRIVE_SPIN		    (-pControllerDriver->GetRawAxis(L310_TRIGGER_LEFT) + pControllerDriver->GetRawAxis(L310_TRIGGER_RIGHT))
#define CHEEZY_DRIVE_QUICKTURN		(pControllerDriver->GetRawButton(L310_BUTTON_BUMPER_LEFT))

#define ARCADE_DRIVE_LEFT			((pControllerDriver->GetRawAxis(L310_THUMBSTICK_LEFT_Y)) + (-1*(pControllerDriver->GetRawAxis(L310_THUMBSTICK_RIGHT_X))))
#define ARCADE_DRIVE_RIGHT			((pControllerDriver->GetRawAxis(L310_THUMBSTICK_LEFT_Y)) - (-1*(pControllerDriver->GetRawAxis(L310_THUMBSTICK_RIGHT_X))))

//#define WAVE_DASH					(pControllerDriver->GetRawButton(L310_BUTTON_A))

#define PIDGEY_ROTATE_LEFT90		(pControllerDriver->GetRawButton(L310_BUTTON_X))
#define PIDGEY_ROTATE_RIGHT90		(pControllerDriver->GetRawButton(L310_BUTTON_B))
#define PIDGEY_ROTATE_180			(pControllerDriver->GetRawButton(L310_BUTTON_Y))

#define DRIVETRAIN_BOXFILTER		(pControllerDriver->GetRawButton(L310_BUTTON_X))
#define PIDGEY_ROTATE_GPTURN		(pControllerDriver->GetRawButton(L310_BUTTON_B))
#define DRIVETRAIN_MTURN			(pControllerDriver->GetRawButton(L310_BUTTON_A))

#define DRIVETRAIN_MMOVE			(pControllerDriver->GetRawButton(L310_BUTTON_Y))
#define CLIMBER_PULL_UP				(pControllerDriver->GetRawButton(L310_BUTTON_BUMPER_LEFT))

// TODO: Add Component Commands

/********** Drive Train Constants:**********/
#define WHEEL_DIA					4.0			//Wheel Diameter in inches
#define PI							3.14159
#define ROBOT_WIDTH					26.0		// Width of the drivetrain in inches
#define DRIVETRAIN_CONST_KP			(1.0/120.0) // Constant P value for PID loops
#define DRIVETRAIN_CONST_KI			(1.0/1000.0) // Constant I value for PID loops
#define DRIVETRAIN_CONST_KD			(1.0/40.0)  // Constant D value for PID loops
#define MAX_TURN_SPEED				32604		// Max Turning speed in ticks per 100 milliseconds
#define UPDATE_RATE					.02			// Update loop rate for drive train
#define MAX_STRAIGHT_SPEED			5000.0		// Max Straight speed in Ticks per Second
#define TURN_TTM					0.4			// Time to maximum turning speed
#define STRAIGHT_TTM				0.4			// Time to maximum straight speed
#define FILTER_ONE_LENGTH			20			// Length of first filter
#define FILTER_TWO_LENGTH			10			// Length of second filter
#define MAX_SPEED_PID				.75			// Max speed allowed in autonomous commands

#define ACCEPT_RANGE_TICKS			2048		//Acceptable Range for "finished" PID loop in ticks
#define ACCEPT_RANGE_DEGR			2			//Acceptable Range for "finished" PID loop in degrees
#define ACCEPT_RANGE_KI				20			//Acceptable Range for adding Integral Value

#define ACCEPT_RANGE_MOVE			512			//Acceptable Range for "finished" PID loop for moving straight

#endif // USE_L310_FOR_CONTROLLER_1

#ifdef USE_X3D_FOR_CONTROLLER_2
#endif // USE_X3D_FOR_CONTROLLER_2

#ifdef USE_XBOX_FOR_CONTROLLER_2
#endif // USE_XBOX_FOR_CONTROLLER_2

#ifdef USE_L310_FOR_CONTROLLER_2
#endif // USE_L310_FOR_CONTROLLER_2

#endif //ROBOT_PARAMS_H
