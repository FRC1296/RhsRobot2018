/** \file
 * Base class from which we derive our main robot class.
 *
 * The RhsRobotBase class is an extension to RobotBase and provides basic robot functionality.
 */

#ifndef RHS_ROBOT_BASE_H
#define RHS_ROBOT_BASE_H

#include "RobotMessage.h"
#include <unistd.h>

//Robot
#include "WPILib.h"
#include "RobotBase.h"


typedef enum eRobotOpMode
{
	ROBOT_STATE_DISABLED,
	ROBOT_STATE_AUTONOMOUS,
	ROBOT_STATE_TELEOPERATED,
	ROBOT_STATE_TEST,
	ROBOT_STATE_UNKNOWN
} RobotOpMode;

class RhsRobotBase : public RobotBase
{
public:
	RhsRobotBase();
	virtual ~RhsRobotBase();

	RobotOpMode GetCurrentRobotState();
	RobotOpMode GetPreviousRobotState();
	bool HasStateChanged();
	void StartCompetition();	//Robot's main function

	int GetLoop();				//Returns the loop number

protected:
	RobotMessage robotMessage;			//Message to be written and sent to components

	virtual void Init() = 0;			//Abstract function: initializes the robot
	virtual void OnStateChange() = 0;	//Abstract function: handles state changes
	virtual void Run() = 0;				//Abstract function: robot logic

private:
	RobotOpMode currentRobotState;
	RobotOpMode previousRobotState;

	int loop;			//Loop counter

};

#endif //RHS_ROBOT_BASE_H
