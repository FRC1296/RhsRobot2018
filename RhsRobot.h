/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands.
 */
#ifndef RHS_ROBOT_H
#define RHS_ROBOT_H

#include <RhsRobotBase.h>
#include "WPILib.h"

class RhsRobot : public RhsRobotBase
{
public:
	RhsRobot();
	virtual ~RhsRobot();

private:
	
	void Init();
	void OnStateChange();
	void Run();

	int iLoop;
};

#endif //RHS_ROBOT_H
