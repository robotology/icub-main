// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
\defgroup icub_controlBasisGui controlBasisGui
@ingroup icub_guis

A viewer for running ControlBasis API (CBAPI) control laws.
 
Copyright (C) 2010 RobotCub Consortium 
 
Author: Stephen Hart
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.
 
\section intro_sec Description 

This gui will connect to all running control basis resources that are running on the 
network and allow a user to configure control laws with them.  When a user adds a 
controller to the current control law, it will be added with a lower priority then 
previously added controllers.  The final multi-objective prioritized output will be
computed using nullspace projection.  

The user is allowed to set a gain for each controller.

Allowing "virtual" effectors will enable the user to choose resources such as the 
Cartesian Position of a robot end-effector.  This is "virtual" because the actual 
device that is moved is a Configuration resource that accepts commands by first being
transformed through an available Jacobian.  The possible virtual effectors are defined
by the possible Jacobian operations that act on the actual (moveable) Configuration
resources.  If virtual effectors are not allowed, only the Configuration resources will
be displayed.

Each controller computes its control signal as follows:

\code 
d_effector = - gain*Potential(sensor)*Jacobian^#,
\endcode

where "#" is the generalized (Moore-Penrose) pseudoinverse.  If the "Use Jacobian Transpose"
button is selected, the transpose is used in place of the pseudoinverse.  
 
The GUI will look like this
\image html controlBasisGUI.jpg

\section example_sec Example 
Try with the following:
 
\code
on terminal 1: iCub_SIM

on terminal 2: iCubControlBasisResources  // this starts up the resources indicated in $ICUB_ROOT/app/controlBasis/conf/iCubSimControlBasisResources.ini 

on terminal 3: controlBasisGUI
\endcode
 
This file can be edited at 
\in src/controlBasis/app/controlBasisGUI.cpp.


**/

#ifndef _CBAPI_WINDOW__H_
#define _CBAPI_WINDOW__H_

#include "CBAPIHelper.h"
#include "CBAPIUtil.h"
#include "ControlDataThread.h"

#include "ControlBasisPotentialFunction.h"
#include "ControlBasisJacobian.h"

namespace CB {
  
    class CBAPIWindow : public Gtk::Window {

    public:
        CBAPIWindow();
        virtual ~CBAPIWindow();
        
        void refreshResourceList();
        void loadPotentialFunctions();
        void loadPotentialFunctionsFromFile();
        void loadJacobians();

    protected:
        
        void on_notebook_switch_page(GtkNotebookPage *page, guint page_num);

        void on_sensor_selection();
        void on_reference_selection();
        void on_potential_function_selection();
        void on_effector_selection();
       
        void on_virtual_effector_toggle();
        void on_use_jacobian_transpose();

        // main widgets
        Gtk::VBox cbapiVBox;
        Gtk::Notebook cbapiNotebook;

        // widgets for the control tab
        Gtk::Table controlTabTable;
        Gtk::Table controlResourcesTable;
        Gtk::Table controlDefinitionTable;
        Gtk::Table optionsTable;

        Gtk::Frame controlResourcesFrame;
        Gtk::Frame controlDefinitionFrame;
        Gtk::Frame controlOutputFrame;
    
        Gtk::Button addControllerButton;
        Gtk::Button clearControllerButton;
        Gtk::Button runControllerButton;
        Gtk::Button stopControllerButton;
        Gtk::Button refreshButton;

        Gtk::CheckButton allowVirtualEffectorsBox;
        Gtk::CheckButton useJacobianTransposeBox;

        Gtk::Entry gainEntry;
        Gtk::Label gainLabel;

        ResourceList sensorList;
        ResourceList referenceList;
        ResourceList potentialFunctionList;
        ResourceList effectorList;
        
        CBAPITextWindow controlDefinitionText;
        CBAPITextWindow controlOutputText;
        
        // storage information for building control laws
        std::vector<PotentialFunctionInfo *> pfInfo;
        std::vector<JacobianInfo> jacInfo;
        std::vector<ResourceInfo> sensorInfo;
        std::vector<ResourceInfo> effectorInfo;
        
        void on_add_button_clicked();
        void on_clear_button_clicked();
        void on_run_button_clicked();
        void on_stop_button_clicked();
        void on_refresh_button_clicked();
        void on_control_thread_update(ControlDataThread *dThread);
    
        // the main CBAPI class that we pass GUI events too...
        CBAPIHelper cbapi;
        ControlDataThread *dataThread;
        
        // tree pointers to handle menu selection callbacks
        Glib::RefPtr<Gtk::TreeSelection> sensorTreeSelection;
        Glib::RefPtr<Gtk::TreeSelection> referenceTreeSelection;
        Glib::RefPtr<Gtk::TreeSelection> pfTreeSelection;
        Glib::RefPtr<Gtk::TreeSelection> effectorTreeSelection;
        
        bool controlLawRunning;
        bool showVirtualEffectors;
        bool useJacobianTranspose;

  };

}



#endif
