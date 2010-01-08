/** @file PlannerSupervisor.h Header file the PlannerSupervisor class.
 *
 * Version information : 1.0
 *
 * Date 04/05/2009
 *
 */
/*
 * Copyright (C) 2009 Sebastien Gay, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email: sebastien.gay@epfl.ch
 * website: www.robotcub.org
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2
 * or any later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 */

#ifndef PLANNERSUPERVISOR__H
#define PLANNERSUPERVISOR__H

#include <yarp/os/all.h>
using namespace yarp::os;

#include "GlobalStuff.h"
#include "Gui.h"

#define PORT_NAME "/supervisor/in"
#define PLANNER_PORT "/CrawlPlanner/supervisor/out"
#define MODULE_PERIOD 0.0

/**
 * The main supervisor module class.  
 * This class handles the communication between the CrawlPlanner module (@see CrawlPlanner) and the Gui (@see Gui).
 */
class PlannerSupervisor :
    public Thread
{
public:
	/**
	 * Constructor of the PlannerSupervisor class.
	 * Opens ports and connects to the planner.
	 */
    PlannerSupervisor(Glib::Dispatcher &myDispatcher, Glib::Mutex &myLockMutex, queue<Vision *> &myVisionM_PIpe, queue<dvec2> &myPotentialVectorM_PIpe);
    
	/**
	 * Destructor of the PlannerSupervisor class.
	 * Closes portsr.
	 */
	virtual ~PlannerSupervisor(void);
	
	/**
	 * Main thread loop.
	 * Gets the potential field from the planner and sends it to the Gui (@see Gui)
	 */
    virtual void run();

protected:
    BufferedPort<Bottle> myPort;
    Glib::Dispatcher &dispatcher;  //the signal to send data to the GUI process.
    Glib::Mutex &lockMutex; //mutex to protect the movementsM_PIpe shared queue.
    queue<Vision *> &visionPipe;//shared queue to transmit the vision from this thread to the GUI.
    queue<dvec2> &potentialVectorPipe;//shared queue to transmit the vision rotated to fit the body frame from this thread to the GUI.
};

#endif //PLANNERSUPERVISOR__H