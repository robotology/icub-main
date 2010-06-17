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

static const double M_PI = 3.14159265;

/**
 * A Gtkmm Gui.
 * 
 */
class DrummingGui : public Gtk::Window
{

private:
    enum Parts {LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG, HEAD};

    //the main class to control the drumming;
    DrummingControl drummingControl;

	//Gtk builder
	Glib::RefPtr<Gtk::Builder> builder;

	//widgets
	Gtk::VBox *mainBox;
	Gtk::TreeView *tvwScores;
    Gtk::HScale *hscTempo;
    Gtk::HScale *hscCoordArms;
    Gtk::HScale *hscCoordLegs;
    Gtk::HScale *hscCoordArmsLegs;

public:
/**
 * Constructor of the DrummingGui class
 * Initializes the Window
 */
    DrummingGui();
    virtual ~DrummingGui(void);


private:
	Glib::RefPtr<Gtk::Builder> LoadFromFile();
	void UpdatePartitionFolder(void);
	
    void GetAndConnectWidgets(void);

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
    void OnBtnLeftLegClicked(void);
	void OnBtnLeftLegHoldClicked(void);
    void OnBtnRightLegClicked(void);
	void OnBtnRightLegHoldClicked(void);
    void OnBtnHead1Clicked(void);
    void OnBtnHead2Clicked(void);
    void OnBtnHead3Clicked(void);
    void OnBtnHead4Clicked(void);
	void OnBtnHeadHoldClicked(void);
    void OnBtnHeadScanClicked(void);
    void OnHscTempoValueChanged(void);
    void OnHscCoordArmsValueChanged(void);
    void OnHscCoordLegsValueChanged(void);
    void OnHscCoordArmsLegsValueChanged(void);
    void OnBtnPlayScoreClicked(void);
    void OnBtnStopScoreClicked(void);
};

#endif //DRUMMING_GUI__H
