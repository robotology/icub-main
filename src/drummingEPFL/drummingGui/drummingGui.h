/** @file DrummingGui.h Header file the DrummingGui class.
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

#ifndef DRUMMING_GUI__H
#define DRUMMING_GUI__H

#include <gtkmm.h>

#include <gtkmm/window.h>

#include <gtkmm/button.h>
#include <gtkmm/box.h>
#include <gtkmm/label.h>

#include <string>

#include "drummingControl.h"

static const std::string WINDOW_TITLE = "Drumming GUI";
static const int DEFAULT_WINDOW_WIDTH = 480;
static const int DEFAULT_WINDOW_HEIGHT = 360;
static const std::string GUI_FILE = "drummingGui.glade";
static const std::string PARTITION_FOLDER = "part";

/**
 * A Gtkmm Gui.
 * 
 */
class DrummingGui : public Gtk::Window
{
public:
/**
 * Constructor of the DrummingGui class
 * Initializes the Window
 */
    DrummingGui();
    virtual ~DrummingGui(void);

protected:


private:
	//Gtk builder
	Glib::RefPtr<Gtk::Builder> builder;

	//widgets
	Gtk::VBox *mainBox;
	Gtk::TreeView *tvwScores;


private:
	Glib::RefPtr<Gtk::Builder> LoadFromFile();
	void UpdatePartitionFolder(void);

	

    //the main code to control the drumming;
    DrummingControl drummingControl;

	//signal handlers
	void OnBtnQuitClicked(void);
	void OnBtnStopCustomClicked(void);
	void OnBtnPlayCustomClicked(void);
	void OnBtnLeftArm1Clicked(void);
	void OnBtnLeftArm2Clicked(void);
	void OnBtnLeftArmHoldClicked(void);
	void OnBtnRightArm1Clicked(void);
	void OnBtnRightArm2Clicked(void);
	void OnBtnRightArmHoldClicked(void);
};

#endif //DRUMMING_GUI__H
