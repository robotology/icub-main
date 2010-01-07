#pragma once

#include <gtkmm/button.h>
#include <gtkmm/window.h>
#include "WorldDrawingArea.h"
#include <queue>
using namespace std;
#include "GlobalStuff.h"


#define WINDOW_TITLE "ICub World"
#define DEFAULT_WINDOW_WIDTH 480
#define DEFAULT_WINDOW_HEIGHT 360

class Gui : public Gtk::Window
{
public:
    Gui(Glib::Dispatcher &myDispatcher, Glib::Mutex &myLockMutex, queue<Vision *> &myVisionPipe, queue<dvec2> &myPotentialVectorPipe);
    virtual ~Gui(void);

protected:
//Signal handlers:
void OnReceiveStuff(void);

Glib::Dispatcher &dispatcher;
sigc::connection connection;
Glib::Mutex &lockMutex;
queue<Vision *> &visionPipe;
queue<dvec2> &potentialVectorPipe;

//Member widgets:
//Gtk::Button btnQuit;
WorldDrawingArea *worldDrawingArea;
};
