#include "Gui.h"
#include <iostream>
using namespace std;
Gui::Gui(Glib::Dispatcher &myDispatcher, Glib::Mutex &myLockMutex, queue<Vision *> &myVisionPipe, queue<dvec2> &myPotentialVectorPipe) 
    : dispatcher(myDispatcher), lockMutex(myLockMutex), visionPipe(myVisionPipe), potentialVectorPipe(myPotentialVectorPipe)//: btnQuit("Quit")
{
    set_title(WINDOW_TITLE);
    set_default_size(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
    
    worldDrawingArea = new WorldDrawingArea(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
    add(*worldDrawingArea);
    worldDrawingArea->show();
    connection = dispatcher.connect(sigc::mem_fun(*this, &Gui::OnReceiveStuff)); 
}

Gui::~Gui(void)
{
}

void Gui::OnReceiveStuff(void)
{
    Vision *vision;
    dvec2 potentialVector;
    //Vision *bodyVision;
    {
        Glib::Mutex::Lock lock(lockMutex);
        vision = visionPipe.front();
        visionPipe.pop();
        potentialVector = potentialVectorPipe.front();
        potentialVectorPipe.pop();
        //bodyVision = bodyFrameVisionPipe.front();
        //bodyFrameVisionPipe.pop();
    }
#if _DEBUG
	//cout << "Received displacement : (" << movement.translation.x << "," << movement.translation.y << ")  angle : " << movement.rotationAngle << endl;
#endif

    //for(unsigned int i=0; i<bodyVision->size(); ++i)
    //{
    //    vision->push_back((*bodyVision)[i]);
    //}

	worldDrawingArea->SetVision(vision);
    worldDrawingArea->SetPotentialVector(potentialVector);
	worldDrawingArea->queue_draw();
}
