#include "PlannerSupervisor.h"
#include <gtkmm.h>
#include "Gui.h"
#include <iostream>
#include <string>
using namespace std;
#include "Patch.h"

PlannerSupervisor::PlannerSupervisor(Glib::Dispatcher &myDispatcher, Glib::Mutex &myLockMutex, queue<Vision *> &myVisionPipe, queue<dvec2> &myPotentialVectorPipe)
    : dispatcher(myDispatcher), lockMutex(myLockMutex), visionPipe(myVisionPipe), potentialVectorPipe(myPotentialVectorPipe)
{
    myPort.open(PORT_NAME);
    cout << "Waiting for connection with CrawlPlanner ..." << endl;
    while(!Network::connect(PLANNER_PORT, PORT_NAME))
    {
        Time::delay(1);
    }
}

PlannerSupervisor::~PlannerSupervisor(void)
{
	myPort.close();
}


void PlannerSupervisor::run(void)
{
    while (!isStopping()) 
    {
        Vision *vision = new Vision;
        Bottle *readBottle = myPort.read();
        
        Bottle *potentialVectorBottle = readBottle->get(0).asList();
        dvec2 potentialVector(potentialVectorBottle->get(0).asDouble(), potentialVectorBottle->get(1).asDouble());

        for(int i=1; i < readBottle->size() ; ++i)
        {
            Bottle *potentialBottle = readBottle->get(i).asList();

            double x = potentialBottle->get(0).asDouble();
            double y = potentialBottle->get(1).asDouble();
            double radius = potentialBottle->get(2).asDouble();
            double value = potentialBottle->get(3).asInt();
            Potential *potential = new Potential(x, y, radius, value);
            vision->push_back(potential);
        }

        //sending to Gui process.
        {
            Glib::Mutex::Lock lock(lockMutex);
            visionPipe.push(vision);
            potentialVectorPipe.push(potentialVector);
        }
        dispatcher.emit();
    }
}