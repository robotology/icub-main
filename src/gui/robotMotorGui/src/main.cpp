// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2007 Francesco Nori (iron@liralab.it)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *  
 */

/**
 *
 * @ingroup icub_tools
 * @ingroup icub_guis
 * \defgroup icub_robotMotorGui robotMotorGui
 *
 * A simple graphical interface for moving all
 * the joints of the iCub robot with sliders. 
 * Uses remote control boards. 
 *
 * \image html robotMotorGui.jpg
 * \image latex robotMotorGui.eps "A window of robotMotorGui running on Linux" width=15cm
 *
 * \section intro_sec Description
 *
 * This GUI can be used for the following pouposes:
 *
 * - continuosly reading the position of the ALL the robot joints
 * - running/idling single joints
 * - running ALL the robot joints
 * - position command of single joints
 * - changing the velocity of the position commands
 * - performing sequences of position commands with the ALL robot joints
 * - calibrating single joints
 * - checking if the robot is in position ("@")
 *
 * \section parameters_sec Parameters
 * 
 * \code
 * --name: name of the robot (used to form port names)
 * --parts: a list of parts to be added.
 * \endcode
 * Example:
 * \code
 * robotMotorGui --name icub --parts (head torso left_arm right_arm left_leg right_leg)
 * \endcode
 * 
 * These parameters can be specified in a single file, passed with the
 * --from option.
 * Example:
 * \code
 * robotMotorGui --from robotMotorGui.ini
 * \endcode
 *
 * By default robotMotorGui starts using the file robotMotorGui.ini
 * in $ICUB_ROOT/app/default.
 *
 * An home position can be optionally defined in the supplied file.
 * This home position is specified using a group [part_zero] (e.g. [head_zero])
 * containing the home position and velocity  that will be commanded when the home
 * button is pressed:
 * \code
 * [head_zero]
 * PositionZero      0.0        0.0      0.0       0.0        0.0        0.0
 * VelocityZero     10.0       10.0     10.0      10.0       10.0       10.0 
 * \endcode
 *
 * A set of calibration parameters can be optionally defined in the
 * supplied file. These calibration parameters follow the same standard
 * followed by the \ref icub_iCubInterface and can be specified within
 * the group [part_calib] (e.g. [head_calib]):
 * \code
 * [head_calib]
 * CalibrationType     0          0        0         0          0          0
 * Calibration1    500.0	  1000.0    900.0     300.0     1333.0     1333.0
 * Calibration2     20.0	    20.0     20.0     -20.0        5.0        5.0
 * Calibration3      0.0	     0.0      0.0       0.0        0.0        0.0
 * \endcode
 *
 * \section portsa_sec Ports Accessed
 * For each part initalized (e.g. right_leg):
 * - /icub/right_leg/rpc:i 
 * - /icub/right_leg/command:i
 * - /icub/gui/right_leg/state:i
 *
 * \section portsc_sec Ports Created
 * For each part initalized (e.g. right_leg):
 * - /icub/gui/right_leg/rpc:o
 * - /icub/gui/right_leg/command:o
 * - /icub/right_leg/state:o
 * 
 * \section conf_file_sec Configuration Files
 *
 * Passed with the paremter --from, configure the layout of the gui.
 * \code
 * name icub
 * parts (head torso right_arm left_arm)
 * \endcode
 *
 * Creates a gui. Connects automatically to:
 * 
 * \code
 * /icub/head/*
 * /icub/torso/*
 * /icub/right_arm/*
 * ...
 * \endcode
 *
 * \section tested_os_sec Tested OS
 * Linux and Windows.
 *
 * \author Francesco Nori
 *
 *Copyright (C) 2008 RobotCub Consortium
 *
 *CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 *This file can be edited at src/gui/robotMotorGui/src/main.cpp.
 **/

///////////YARP//////////
#include <yarp/os/impl/NameClient.h>

///////canGui/////////////
//#include "include/robotMotorGui.h"
//#include "include/partMover.h"

#include "include/allPartsWindow.h"
#include <string>

using namespace yarp::os::impl;


///////Initializations////
GtkWidget *robotNameBox   = NULL;

char *partsName[];
int *ENA[];
int NUMBER_OF_ACTIVATED_PARTS = 0;
int NUMBER_OF_AVAILABLE_PARTS = 0;
int PART;

ResourceFinder *finder;
////////////////////////

static void destroy_main (GtkWindow *window,	gpointer   user_data)
{
    gtk_widget_destroy (GTK_WIDGET(window));
    gtk_main_quit ();
}

GtkWidget *buttonGoAll;
GtkWidget *buttonSeqAll;
GtkWidget *buttonSeqAllTime;
GtkWidget *buttonSeqAllSave;
GtkWidget *buttonSeqAllLoad;
GtkWidget *buttonSeqAllCycle;
GtkWidget *buttonSeqAllCycleTime;
GtkWidget *buttonSeqAllStop;
GtkWidget *buttonSeqAllStopTime;
GtkWidget *buttonRunAllParts;
GtkWidget *buttonHomeAllParts;

//*********************************************************************************
// This callback switches among tabs
void notebook_change (GtkNotebook *nb, GtkNotebookPage *nbp,	gint current_enabled, partMover** currentPartMover)
{
    gint i;

    //skip if the "all" tab has been called
    if (current_enabled!=NUMBER_OF_ACTIVATED_PARTS)
        {
            for (i = 0; i < NUMBER_OF_ACTIVATED_PARTS; i++)
                {
                    //fprintf(stderr, "disabling update for part %d \n", i);
                    currentPartMover[i]->disable_entry_update(currentPartMover[i]);
                }
            //fprintf(stderr, "enabling update for part %d \n", current_enabled);
            currentPartMover[current_enabled]->enable_entry_update(currentPartMover[current_enabled]);
        }

    return;
}


//*********************************************************************************
// This function is main window after selection of the part to be controlled

static void myMain2(GtkButton *button,	int *position)
{
    std::string portPartName;
    std::string robotName;
    std::string portLocalName;

    portPartName.resize(1024);
    robotName.resize(1024);
    portLocalName.resize(1024);

    GtkWidget *main_vbox1		 = NULL;
    GtkWidget *main_vbox2		 = NULL;
    GtkWidget *main_vbox3		 = NULL;
    GtkWidget *main_vbox4		 = NULL;
    GtkWidget *main_vbox5		 = NULL;
    Property options;

    yarp::os::Network::init();

    //retrieve robot name
    sprintf(&robotName[0], "%s", gtk_entry_get_text((GtkEntry *)(robotNameBox)));
  
    gtk_widget_destroy (window);
    window = NULL;
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	
    //Creation of the notebook
    GtkWidget*  nb1 = gtk_notebook_new();
    gtk_container_add (GTK_CONTAINER (window), nb1);
    g_signal_connect (window, "destroy",G_CALLBACK (destroy_main), &window);

    char legsLabel[]= "Legs";
    GtkWidget *label;
    int n;

    PolyDriver *partsdd[MAX_NUMBER_ACTIVATED];
    partMover *partMoverList[MAX_NUMBER_ACTIVATED];
    partMover *currentPartMover;

    for (n = 0; n < NUMBER_OF_AVAILABLE_PARTS; n++)
        {
            if(*ENA[n] == 1)
                {
                    label = gtk_label_new(partsName[n]);
                    main_vbox4 = gtk_vbox_new (FALSE, 0);
                    gint note4 = gtk_notebook_append_page((GtkNotebook*) nb1, main_vbox4, label);

                    sprintf(&portPartName[0], "/%s/%s", robotName.c_str(), partsName[n]);

                    //checking eixstence of the port
                    int ind = 0;
                    sprintf(&portLocalName[0], "/%s/gui%d/%s/rpc:o", robotName.c_str(), ind, partsName[n]);
                    NameClient &nic=NameClient::getNameClient();
                    fprintf(stderr, "Checking the existence of: %s \n", portLocalName.c_str());
                    Address adr=nic.queryName(portLocalName.c_str());

                    //Contact c = yarp::os::Network::queryName(portLocalName.c_str());
                    fprintf(stderr, "ADDRESS is: %s \n", adr.toString().c_str());
                    while(adr.isValid())
                        {   
                            ind++;
                            sprintf(&portLocalName[0], "/%s/gui%d/%s/rpc:o", robotName.c_str(), ind, partsName[n]);
                            adr=nic.queryName(portLocalName.c_str());
                        }

                    sprintf(&portLocalName[0], "/%s/gui%d/%s", robotName.c_str(), ind, partsName[n]);
                    options.put("local", portLocalName.c_str());	//local port names
                    options.put("device", "remote_controlboard");
                    options.put("remote", portPartName.c_str());
                    options.put("carrier", "udp");

                    partsdd[n] = new PolyDriver(options);
                    currentPartMover = new partMover(main_vbox4, partsdd[n], partsName[n], finder);
                    if(!(currentPartMover->interfaceError)) 
                        {
                            partMoverList[NUMBER_OF_ACTIVATED_PARTS] = currentPartMover;
                            NUMBER_OF_ACTIVATED_PARTS++;
                        }
                }
        }


    if (NUMBER_OF_ACTIVATED_PARTS > 0)
        {
            g_signal_connect (nb1, "switch-page",G_CALLBACK(notebook_change), partMoverList);

            main_vbox5 = gtk_fixed_new ();
            label = gtk_label_new("all");
            gint note4 = gtk_notebook_append_page((GtkNotebook*) nb1, main_vbox5, label);
	
            //Frame
            GtkWidget *frame1 = gtk_frame_new ("Global commands");
            gtk_widget_set_size_request 	(frame1, 250, 550);
            gtk_fixed_put (GTK_FIXED (main_vbox5), frame1, 10, 10);

            //Button 1 in the panel
            buttonGoAll = gtk_button_new_with_mnemonic ("Go ALL!");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonGoAll, 30, 50);
            g_signal_connect (buttonGoAll, "clicked", G_CALLBACK (go_all_click), partMoverList);
            gtk_widget_set_size_request(buttonGoAll, 190, 30);

	
            //Button 2 in the panel
            buttonSeqAll = gtk_button_new_with_mnemonic ("Run ALL Sequence");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonSeqAll, 30, 100);
            g_signal_connect (buttonSeqAll, "clicked",G_CALLBACK(sequence_all_click), partMoverList);
            gtk_widget_set_size_request     (buttonSeqAll, 190, 30);

            //Button 2time in the panel
            buttonSeqAllTime = gtk_button_new_with_mnemonic ("Run ALL Sequence (time)");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonSeqAllTime, 30, 150);
            g_signal_connect (buttonSeqAllTime, "clicked",G_CALLBACK(sequence_all_click_time), partMoverList);
            gtk_widget_set_size_request     (buttonSeqAllTime, 190, 30);	
	
            //Button 3 in the panel
            buttonSeqAllSave = gtk_button_new_with_mnemonic ("Save ALL Sequence");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonSeqAllSave, 30, 200);
            g_signal_connect (buttonSeqAllSave, "clicked", G_CALLBACK(sequence_all_save), partMoverList);
            gtk_widget_set_size_request     (buttonSeqAllSave, 190, 30);	
	
            //Button 4 in the panel
            buttonSeqAllLoad = gtk_button_new_with_mnemonic ("Load ALL Sequence");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonSeqAllLoad, 30, 250);
            g_signal_connect (buttonSeqAllLoad, "clicked", G_CALLBACK (sequence_all_load), partMoverList);
            gtk_widget_set_size_request     (buttonSeqAllLoad, 190, 30);

	
            //Button 5 in the panel
            buttonSeqAllCycle = gtk_button_new_with_mnemonic ("Cycle ALL Sequence");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonSeqAllCycle, 30, 300);
            g_signal_connect (buttonSeqAllCycle, "clicked", G_CALLBACK (sequence_all_cycle), partMoverList);
            gtk_widget_set_size_request     (buttonSeqAllCycle, 190, 30);

            //Button 5time in the panel
            buttonSeqAllCycleTime = gtk_button_new_with_mnemonic ("Cycle ALL Sequence (time)");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonSeqAllCycleTime, 30, 350);
            g_signal_connect (buttonSeqAllCycleTime, "clicked", G_CALLBACK (sequence_all_cycle_time), partMoverList);
            gtk_widget_set_size_request     (buttonSeqAllCycleTime, 190, 30);

            //Button 6 in the panel
            buttonSeqAllStop = gtk_button_new_with_mnemonic ("Stop ALL Sequence");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonSeqAllStop, 30, 400);
            g_signal_connect (buttonSeqAllStop, "clicked", G_CALLBACK (sequence_all_stop),  partMoverList);
            gtk_widget_set_size_request     (buttonSeqAllStop, 190, 30);
	
            //Button 6time in the panel
            //buttonSeqAllStopTime = gtk_button_new_with_mnemonic ("Stop ALL Sequence (time)");
            //gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonSeqAllStopTime, 120, 450);
            //g_signal_connect (buttonSeqAllStopTime, "clicked", G_CALLBACK (sequence_all_stop_time),  partMoverList);
            //gtk_widget_set_size_request     (buttonSeqAllStopTime, 190, 30);

            //Button 6time in the panel
            buttonRunAllParts = gtk_button_new_with_mnemonic ("Run ALL Parts");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonRunAllParts, 30, 450);
            g_signal_connect (buttonRunAllParts, "clicked", G_CALLBACK (run_all_parts),  partMoverList);
            gtk_widget_set_size_request     (buttonRunAllParts, 190, 30);

            //Button 7 in the panel
            buttonHomeAllParts = gtk_button_new_with_mnemonic ("Home ALL Parts");
            gtk_fixed_put	(GTK_FIXED(main_vbox5), buttonHomeAllParts, 30, 500);
            g_signal_connect (buttonHomeAllParts, "clicked", G_CALLBACK (home_all_parts),  partMoverList);
            gtk_widget_set_size_request     (buttonHomeAllParts, 190, 30);

            // finish & show
            //	connected_status();
            gtk_window_set_default_size (GTK_WINDOW (window), 480, 250);
            gtk_window_set_resizable (GTK_WINDOW (window), true);
        }
		
    if (!GTK_WIDGET_VISIBLE (window))
        gtk_widget_show_all (window);
    else
        {
            gtk_widget_destroy (window);
            window = NULL;
        }

    gtk_main ();		
    fprintf(stderr, "Closing the partMovers. Number of acitvated parts was %d. \n", NUMBER_OF_ACTIVATED_PARTS);
    for (int i = 0; i < NUMBER_OF_ACTIVATED_PARTS; i++)
        {
            fprintf(stderr, "Closing part number %d \n", i);
            partMoverList[i]->releaseDriver();
            fprintf(stderr, "Deleting part number %d \n", i);
            delete partMoverList[i];
            Time::delay(1);
        }
    fprintf(stderr, "Closing the main GUI \n");
    Network::fini();
    return;
	
}


//*********************************************************************************
static GtkTreeModel * create_net_model (void)
{
    gint i = 0;
    GtkListStore *store;
    GtkTreeIter iter;
	
    // create list store
    store = gtk_list_store_new (1, G_TYPE_STRING);
	
    // add data to the list store
    gtk_list_store_append (store, &iter);
    gtk_list_store_set (store, &iter, 0, "Right Arm",-1);
    gtk_list_store_append (store, &iter);
    gtk_list_store_set (store, &iter, 0, "Left Arm",-1);  
    gtk_list_store_append (store, &iter);
    gtk_list_store_set (store, &iter, 0, "Head Waist",-1);  
    gtk_list_store_append (store, &iter);
    gtk_list_store_set (store, &iter, 0, "Legs",-1);  
	
    return GTK_TREE_MODEL (store);
}

//*********************************************************************************
static void combo_net_changed (GtkComboBox *box,	gpointer   user_data)
{
    PART = gtk_combo_box_get_active (box);
}

static void check_pressed(GtkWidget *box,	gpointer   user_data)
{
    int *pENA = (int *) user_data;
    if (*pENA == 0)
        *pENA = 1;         //part selected for use
    else
        *pENA = 0;         //part deselected for use
    //fprintf(stderr, "%d \n", *pENA);
    return;
}


//*********************************************************************************
// Entry point for the GTK application
int myMain( int   argc, char *argv[] )
{
    int n=0;
    GtkWidget *button1;
    GtkWidget *inv1     = NULL;
    GtkWidget *top_hbox = NULL;
    GtkWidget *main_vbox= NULL;
    Property p, q;
    finder = new ResourceFinder;
    gtk_init (&argc, &argv);
    //////////////////////////////////////////////////////////////////////
    //create the main window, and sets the callback destroy_main() to quit
    //the application when the main window is closed
    //////////////////////////////////////////////////////////////////////
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	
    gtk_window_set_title (GTK_WINDOW (window), "CanControl GUI");
    g_signal_connect (window, "destroy",G_CALLBACK (destroy_main), &window);
	
    gtk_container_set_border_width (GTK_CONTAINER (window), 8);
	
    //Creation of main_vbox the container for every other widget
    main_vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (window), main_vbox);
	
    //creation of the top_hbox
    top_hbox = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (top_hbox), 10);
    gtk_container_add (GTK_CONTAINER (main_vbox), top_hbox);
	
    inv1 = gtk_fixed_new ();
    gtk_container_add (GTK_CONTAINER (top_hbox), inv1);
  
    //retrieve information for the list of parts
    finder->setVerbose();
    finder->setDefaultConfigFile("robotMotorGui.ini");
    finder->setDefault("name", "icub");
    finder->configure("ICUB_ROOT",argc,argv);
    //fprintf(stderr, "Retrieved finder: %p \n", finder);

    std::string robotName=finder->find("name").asString().c_str();
    Bottle *pParts=finder->find("parts").asList();
    if (pParts!=0)
        NUMBER_OF_AVAILABLE_PARTS=pParts->size();
    else
        NUMBER_OF_AVAILABLE_PARTS=0;

    if (NUMBER_OF_AVAILABLE_PARTS > MAX_NUMBER_ACTIVATED)
        {
            fprintf(stderr, "The number of parts exceeds the maximum! \n");
            return 0;
        }
    if (NUMBER_OF_AVAILABLE_PARTS<=0)
        {
            fprintf(stderr, "Invalid number of parts, check config file \n");
            return 0;
        }

    for(n=0; n < MAX_NUMBER_ACTIVATED; n++)
        {
            //ENA = 0: part available
            //ENA = -1: part unavailable
            //ENA = 1: part used
            ENA[n] = new int;
            *ENA[n] = -1;
        } 

    //Check 1 in the panel
    for(n=0;n<NUMBER_OF_AVAILABLE_PARTS;n++)
        {
            partsName[n] = new char[80];
            *ENA[n] = 1;
            //fprintf(stderr, "Getting part %d \n", n);
            strcpy(partsName[n], pParts->get(n).asString().c_str());
            //fprintf(stderr, "%s \n", partsName[n]);
            GtkWidget *check= gtk_check_button_new_with_mnemonic (partsName[n]);
            gtk_fixed_put	(GTK_FIXED(inv1), check, 100*n, 0);
            gtk_widget_set_size_request     (check, 80, 50);
            gtk_toggle_button_set_active((GtkToggleButton*) check, true);
            g_signal_connect (check, "clicked", G_CALLBACK (check_pressed),ENA[n]);
        }

    //Robot name
    robotNameBox = gtk_entry_new ();
    gtk_entry_set_editable(GTK_ENTRY(robotNameBox),true);
    //fprintf(stderr, "%s", (const char*) xtmp);
    gtk_entry_set_text(GTK_ENTRY(robotNameBox), robotName.c_str());
    gtk_fixed_put	(GTK_FIXED(inv1), robotNameBox, 100*NUMBER_OF_AVAILABLE_PARTS, 10);
    //g_signal_connect (renderer, "edited", G_CALLBACK (edited_timing), currentClassData);
    gtk_widget_set_size_request     (robotNameBox, 60, 30);
    //gtk_container_add (GTK_CONTAINER (top_hbox), robotNameBox);

    //Button 1 in the panel
    button1 = gtk_button_new_with_mnemonic ("Select Parts and Click");
    //gtk_container_add (GTK_CONTAINER (top_hbox ), button1);
    gtk_fixed_put	(GTK_FIXED(inv1), button1, 100*(NUMBER_OF_AVAILABLE_PARTS +1), 10);
    g_signal_connect (button1, "clicked", G_CALLBACK (myMain2),NULL);
    gtk_widget_set_size_request     (button1, 180, 30);

    gtk_window_set_default_size (GTK_WINDOW (window), 60, 30);
    gtk_window_set_resizable (GTK_WINDOW (window), true);
	
    if (!GTK_WIDGET_VISIBLE (window))
        gtk_widget_show_all (window);
    else
        {
            gtk_widget_destroy (window);
            window = NULL;
        }
	
    gtk_main ();
    fprintf(stderr, "Deleting the finder");
    delete finder;
    fprintf(stderr, "...done!\n");
    return 0;
}

int main(int argc, char* argv[])
{
    return myMain(argc, argv);
}
