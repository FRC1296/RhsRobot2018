/** \file
 *  Messages used for intertask communications
 */

/** Defines the messages we pass from task to task.
 *
 * The RobotMessage struct is a data structure used to pass information to the
 * robot's components. It is composed of a command that indicates the action to
 * be carried out and a union of params that contain additional data.
 */

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

/**
 \msc
 arcgradient = 8;
 robot [label="Main\nRobot"],
 auto [label="Autonomous"],
 check [label="Check\nList"],
 drive [label="Drive\nTrain"],
 test [label="Component\nExample"];
 robot=>* [label="SYSTEM_MSGTIMEOUT"];
 robot=>* [label="SYSTEM_OK"];
 robot=>* [label="SYSTEM_ERROR"];
 robot=>* [label="STATE_DISABLED"];
 robot=>* [label="STATE_AUTONOMOUS"];
 robot=>* [label="STATE_TELEOPERATED"];
 robot=>* [label="STATE_TEST"];
 robot=>* [label="STATE_UNKNOWN"];
 robot=>auto [label="RUN"];
 robot=>check [label="RUN"];
 robot=>drive [label="STOP"];
 robot=>drive [label="DRIVE_TANK"];
 robot=>drive [label="DRIVE_ARCADE"];
 auto=>drive [label="DRIVE_STRAIGHT"];
 auto=>drive [label="TURN"];
 drive=>auto [label="AUTONOMOUS_RESPONSE_OK"]
 drive=>auto [label="AUTONOMOUS_RESPONSE_ERROR"]

 robot=>test[label="TEST"];
 \endmsc

 */

enum MessageCommand {
	COMMAND_UNKNOWN,					//!< COMMAND_UNKNOWN
	COMMAND_SYSTEM_MSGTIMEOUT,			//!< COMMAND_SYSTEM_MSGTIMEOUT
	COMMAND_SYSTEM_OK,					//!< COMMAND_SYSTEM_OK
	COMMAND_SYSTEM_ERROR,				//!< COMMAND_SYSTEM_ERROR
	COMMAND_SYSTEM_CONSTANTS,
	COMMAND_SYSTEM_GAMEDATA,
	COMMAND_SYSTEM_PDBDATA,

	COMMAND_ROBOT_STATE_DISABLED,		//!< Tells all components that the robot is disabled
	COMMAND_ROBOT_STATE_AUTONOMOUS,		//!< Tells all components that the robot is in auto
	COMMAND_ROBOT_STATE_TELEOPERATED,	//!< Tells all components that the robot is in teleop
	COMMAND_ROBOT_STATE_TEST,			//!< Tells all components that the robot is in test
	COMMAND_ROBOT_STATE_UNKNOWN,		//!< Tells all components that the robot's state is unknown

	COMMAND_AUTONOMOUS_RUN,				//!< Tells Autonomous to run
	COMMAND_AUTONOMOUS_COMPLETE,		//!< Tells all components that Autonomous is done running the script
	COMMAND_AUTONOMOUS_RESPONSE_OK,		//!< Tells Autonomous that a command finished running successfully
	COMMAND_AUTONOMOUS_RESPONSE_ERROR,	//!< Tells Autonomous that a command had a error while running
	COMMAND_CHECKLIST_RUN,				//!< Tells CheckList to run

	COMMAND_DRIVETRAIN_RUN,
	COMMAND_DRIVETRAIN_RUN_ARCADE,		//!< Run drive train using arcade controls
	COMMAND_DRIVETRAIN_RUN_TANK,
	COMMAND_DRIVETRAIN_MMOVE, 			//!< Moves a specified distance
	COMMAND_DRIVETRAIN_MTURN,			//!< Drivetrain test for Turning with just encoders
	COMMAND_DRIVETRAIN_WAVE,			//!< Drivetrain test for sin wave for THE CLAW
	COMMAND_DRIVETRAIN_DRIVE_CHEESY,
	COMMAND_DRIVETRAIN_AUTOTURN,
	COMMAND_DRIVETRAIN_AUTOMOVE,

	COMMAND_DRIVETRAIN_GPTURN,			//!< Drivetrain test for Gyro rotation with a PID loop
	COMMAND_DRIVETRAIN_BOXFILTER,		//!< Drivetrain test for Boxfilter turning

	COMMAND_ELEVATOR_MOVE,				//!< "Manual" elevator control
	COMMAND_ELEVATOR_FLOOR,				//!< Elevator floor position
	COMMAND_ELEVATOR_SWITCH,			//!< Elevator switch position
	COMMAND_ELEVATOR_SCALE_LOW,			//!< Elevator scale position (tipped in our favor)
	COMMAND_ELEVATOR_SCALE_MID,			//!< Elevator scale position (balanced)
	COMMAND_ELEVATOR_SCALE_HIGH,		//!< Elevator scale position (tipped not in our favor)
	COMMAND_ELEVATOR_CLIMB,				//!< Prepare Elevator for climbing
	COMMAND_ELEVATOR_NOBUTTON,

	COMMAND_CLAW_INHALE,				//!< Grab the block
	COMMAND_CLAW_EXHALE,				//!< Spit out block
	COMMAND_CLAW_STOP,					//!< Stop rollers now!
	COMMAND_CLAW_PINCH,					//!< Pinch the block
	COMMAND_CLAW_RELEASE,				//!< Release the block

	COMMAND_ARM_MOVE,
	COMMAND_ARM_OPEN,
	COMMAND_ARM_SHOOT,
	COMMAND_ARM_STOW,

	//add new component messages here

	COMMAND_COMPONENT_TEST,				//!< COMMAND_COMPONENT_TEST
	COMMAND_LAST                        //!< COMMAND_LAST
};

struct MoveParams {
	float fLeft;
	float fRight;
};

struct MeasuredMoveParams {
	float fSpeed;
	float fDistance;
	float fTime;
};

struct ProximityMoveParams {
	float fSpeed;
	float fDistance;
	float fTime;
};

struct TurnParams {
	float fAngle;
	float fTimeout;
};

///Used to deliver joystick readings to Drivetrain
struct TankDriveParams {
	float left;
	float right;
};

struct ArcadeDriveParams {
	float left;
	float right;
};


struct CheesyDriveParams {
	float wheel;
	float throttle;
	bool bQuickturn;
};

struct ClawParams {
	float fClawSpeed;
};

struct ArmParams {
	float fArmSpeed;
};

struct ElevatorParams {
	float fDistance;
	float fTime;
	float fSpeed;
};

struct SystemParams {
	float fBattery;
};

struct PdbParams {
	unsigned char ldrive1;
	unsigned char ldrive2;
	unsigned char ldrive3;
	unsigned char rdrive1;
	unsigned char rdrive2;
	unsigned char rdrive3;
	unsigned char lclaw;
	unsigned char rclaw;
};


enum GamePieceSides
{
	GAMEPIECESIDE_LEFT,
	GAMEPIECESIDE_RIGHT,
	GAMEPIECESIDE_LAST
};

enum GamePieceStart
{
	GAMEPIECESTART_LEFT,
	GAMEPIECESTART_CENTER,
	GAMEPIECESTART_RIGHT,
	GAMEPIECESTART_SIMPLE,
	GAMEPIECESTART_LAST
};

struct GameDataParams {
	GamePieceSides eSwitchSide;
	GamePieceSides eScaleSide;
	GamePieceSides eOpponentSwitchSide;
	GamePieceStart eStartingPosition;
};

///Used to deliver autonomous values to Drivetrain

struct AutonomousParams {
	unsigned uMode;
	unsigned uDelay;
	///how long a function can run, maximum
	float timeout;
	///how long until a function performs
	float timein;

	///used by drivetrain for straight driving
	float driveSpeed;
	float driveDistance;
	float turnAngle;
	float driveTime;
};

///Contains all the parameter structures contained in a message
union MessageParams {
	MoveParams move;
	MeasuredMoveParams mmove;
	ProximityMoveParams pmove;
	TurnParams turn;
	SystemParams system;
	ArcadeDriveParams adrive;
	CheesyDriveParams cheesyDrive;
	TankDriveParams tdrive;
	GameDataParams gamedata;
	ClawParams claw;
	ArmParams arm;
	ElevatorParams elevator;
	PdbParams pdb;
};

///A structure containing a command, a set of parameters, and a reply id, sent between components
struct RobotMessage {
	MessageCommand command;
	const char* replyQ;
	MessageParams params;
};

#endif //ROBOT_MESSAGE_H
