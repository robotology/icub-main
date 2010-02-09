/** @file Gui.h Header file the Gui class.
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

#ifndef GUI__H
#define GUI__H

#include <gtkmm/button.h>
#include <gtkmm/window.h>
#include "WorldDrawingArea.h"
#include <queue>
using namespace std;
#include "GlobalStuff.h"


#define WINDOW_TITLE "ICub World"
#define DEFAULT_WINDOW_WIDTH 480
#define DEFAULT_WINDOW_HEIGHT 360


/**
 * A Gtkmm Gui.
 * Displays a single window with the potential field drawn in it.
 */
class Gui : public Gtk::Window
{
public:
/**
 * Constructor of the Gui class
 * Initializes the Window and the drawing area.
 */
Gui(Glib::Dispatcher &myDispatcher, Glib::Mutex &myLockMutex, queue<Vision *> &myVisionPipe, queue<Vector> &myPotentialVectorPipe);
    virtual ~Gui(void);

protected:
/**
 * Callback function to handle received data from the PlannerSupervisor thread (@see PlannerSupervisor)
 * Gets the potential field from the PlannerSupervisor thread through a pipe and draws it on the window.
 */
void OnReceiveStuff(void);

private:
Glib::Dispatcher &dispatcher;
sigc::connection connection;
Glib::Mutex &lockMutex;
queue<Vision *> &visionPipe;
queue<Vector> &potentialVectorPipe;

//Member widgets:
//Gtk::Button btnQuit;
WorldDrawingArea *worldDrawingArea;
};

#endif //GUI__H
