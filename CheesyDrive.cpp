/** \file
 * Example of subsystem task behavior.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 * The task receives messages from the main robot class and implements behaviors
 * for a given subsystem.
 */

#include "CheesyDrive.h"


void CheezyInit1296(void)
{

}

void CheezyIterate1296(
    const DrivetrainGoal *goal,
    const DrivetrainPosition *position,
    DrivetrainOutput *output,
    DrivetrainStatus *status)
{

}

CheesyLoop::CheesyLoop()
{
	bOutputEnabled = false;
	bEnableServo = false;
	CheezyInit1296();  // initialize the cheezy drive code base

	pTask = new std::thread(&CheesyLoop::StartTask, this, CHEESY_TASKNAME, CHEESY_PRIORITY);
}

CheesyLoop::~CheesyLoop()
{
	delete pTask;
}

void CheesyLoop::Run(void)
{
	 while(true)
	 {
		 Wait(0.005);

		if(bOutputEnabled)
		{
		    std::lock_guard<wpi::mutex> lock(sync);
			 CheezyIterate1296(&currentGoal,
					 &currentPosition,
					 &currentOutput,
					 &currentStatus);
		}
		else
		{
		    std::lock_guard<wpi::mutex> lock(sync);
			 CheezyIterate1296(&currentGoal,
					 &currentPosition,
					 NULL,
					 &currentStatus);
		}
	 }
}

void CheesyLoop::Update(const DrivetrainGoal &goal,
    const DrivetrainPosition &position,
    DrivetrainOutput &output,
    DrivetrainStatus &status,
	bool bEnabled)
{
    std::lock_guard<wpi::mutex> lock(sync);

	bOutputEnabled = bEnabled;
	currentGoal = goal;
	currentPosition = position;
	output = currentOutput;
	status = currentStatus;
}






