#pragma once
#include <yarp/os/all.h>
using namespace yarp::os;

#include "GlobalStuff.h"
#include "Gui.h"

#define PORT_NAME "/supervisor/in"
//#define TRACKER_PORT "/tracker"
#define PLANNER_PORT "/CrawlPlanner/supervisor/out"
#define MODULE_PERIOD 0.0

class PlannerSupervisor :
    public Thread
{
public:
    PlannerSupervisor(Glib::Dispatcher &myDispatcher, Glib::Mutex &myLockMutex, queue<Vision *> &myVisionM_PIpe, queue<dvec2> &myPotentialVectorM_PIpe);
    virtual ~PlannerSupervisor(void);
    virtual void run();

protected:
    BufferedPort<Bottle> myPort;
    Glib::Dispatcher &dispatcher;  //the signal to send data to the GUI process.
    Glib::Mutex &lockMutex; //mutex to protect the movementsM_PIpe shared queue.
    queue<Vision *> &visionPipe;//shared queue to transmit the vision from this thread to the GUI.
    queue<dvec2> &potentialVectorPipe;//shared queue to transmit the vision rotated to fit the body frame from this thread to the GUI.
};