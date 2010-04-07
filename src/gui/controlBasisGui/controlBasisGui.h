// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
\defgroup icub_controlBasisGui controlBasisGui
@ingroup icub_guis

A viewer for running ControlBasis API (CBAPI) control laws.
 
Copyright (C) 2010 RobotCub Consortium 
 
Author: Stephen Hart
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.
 
\section intro_sec Description 

This GUI will connect to all running control basis resources that are running on the 
network and allow a user to configure control laws with them.  When a user adds a 
controller to the current control law, it will be added with a lower priority then 
previously added controllers.  The final multi-objective prioritized output will be
computed using nullspace projection.  

The user is allowed to set a gain for each controller.

Allowing "virtual" effectors will enable the user to choose resources such as the 
Cartesian Position of a robot end-effector as the thing to move.  These effectors are "virtual" because the actual 
device that is moved is a Configuration resource transformed through an available Jacobian.  The possible virtual effectors are defined
by the possible Jacobian operations that act on the actual (moveable) Configuration
resources.  If virtual effectors are not allowed, only the Configuration resources will
be displayed.

Each controller computes its control signal as follows:

\code 
d_effector = - gain*Potential(sensor)*Jacobian^#,
\endcode

where "#" is the generalized (Moore-Penrose) pseudoinverse of the Jacobian:
\code
J = d_Potential(sensor) / d_effector.
\endcode
If the "use Jacobian Transpose" button is selected, the transpose is used in place of the pseudoinverse.  

If the "use PD-Control" button is selected, the control loop will compute errors using both proportional 
and derivative components.  If it is not, the controller is only proportional.
 
The GUI will look like this
\image html controlBasisGUI.jpg


A high-level diagram of how controllers are combined is shown in the figure below:

\image html cbapi.png

Each controller in the multi-objective law can require a Jacobian (if the task-space is not 
Configuration-space), and a set of k sensory resources (Note that the control basis GUI application
currently only allows the user to specify two sensors at this time: a "current" and a "reference").  In general, a particular
potential function could require more, but this has not been implemented yet in the GUI, as none of the current
PFs do in fact require more then either one or two sensors.  

A note on what's going on "beneath the hood."  When the GUI creates a control law of n primitives, it creates one RunnableControlLaw object
and n Controller objects.  The Controllers automatically create the PotentialFunction and Jacobian classes as necessay (if 
the effector is "virtual," then the RunnableControlLaw actually creates the Jacobian).  These classes connect over YARP
ports to the Control Basis resources that must be running on the network.  These resourcees, as necessary, communicate with the low-level
devices that read from and write to the hardware. For example, consider a Cartesian controller that moves a YARP motor device to a 
reference position.  A schematic of this controller is shown below:

\image html cartesian_controller.png

In this example, the YARPConfigurationVariables object talks to a motor device with the name <device_name> (in this example, this is an iCub device).  It accepts commands 
via a YARP port (/data:i). It provides the current values of the device joints and the DH-Parameters over the ports /data:o and 
/params:o, respectively.  The EndEffectorCartesianPosition resource consumes these values and computes the forward kinematics, thus providing
the "current" signal to the potential function; in this case a simple squared-error quadratic that measures the error between the current
and the reference value provided by a CartesianPositionReference object.  When the Controller computes this error, it needs to send a signal back
to the device to move it and reduce the error.  However, because the error is in Cartesian-space, it needs to first be transformed into Configuration-space using an
instance of the ManipulatorPositionJacobian.  This Jacobian also needs the current position and the DH-Parameters from the device, and
so connects to the appropriate ports.

A second controller example can be seen below.  This controller moves an iCub manipulator to optimize the Measure of Manipulability metric. This metric  keeps the manipulator
Jacobian well conditioned (and thus away from undesirable singularities).  This controller operates in Configuration-space and thus does not need a Jacobian.  Moreover, the 
potential is evaluated from only the current value of the robot's joints and the DH-parameters (and therefore does not need a second reference sensor).  Note that if this 
controller were to run as a 2nd priorty controller to the Cartesian Controller shown in the above example, then it would connect to the same RunnableControlLaw and YARPConfigurationVariables
objects.

\image html manipulability_controller.png

\section example_sec Example 
Try with the following:
 
\code

// start the iCub simulator
on terminal 1: iCub_SIM

// start the control basis modules for the iCub (simulator)
on terminal 2: iCubControlBasisResources  // this starts up the resources indicated in $ICUB_ROOT/app/controlBasis/conf/iCubSimControlBasisResources.ini. 
// Edit this file to change which resources are started. 

// optional
on terminal 2a: referenceGui // starts up iCub reference modules

// start the GUI
on terminal 3: controlBasisGui
\endcode
 
This file can be edited at 
\in src/controlBasis/app/controlBasisGui.cpp.

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
        void on_use_pd_control();

        // main widgets
        Gtk::VBox cbapiVBox;
        Gtk::Notebook cbapiNotebook;

        // widgets for the control tab
        Gtk::Table cbapiTable;
        Gtk::Table controlResourcesTable;

        Gtk::Table controlTabTable;
        Gtk::Table controlDefinitionTable;
        Gtk::Table optionsTable;

        // widgets for the sequence tab
        Gtk::Table sequenceTabTable;
        Gtk::Table sequenceControllersTable;
        Gtk::Frame sequenceControllersFrame;
        Gtk::Frame sequenceOutputFrame;

        Gtk::Frame controlResourcesFrame;
        Gtk::Frame controlDefinitionFrame;
        Gtk::Frame controlOutputFrame;
    
        Gtk::Button addControllerButton;
        Gtk::Button clearControllerButton;
        Gtk::Button runControllerButton;
        Gtk::Button stopControllerButton;
        Gtk::Button refreshButton;

        Gtk::Button addControllerToSequenceButton;
        Gtk::Button addSecondaryControllerToSequenceButton;
        Gtk::Button clearSequenceButton;
        Gtk::Button runSequenceButton;
        Gtk::Button stopSequenceButton;
        Gtk::Button fwdSequenceButton;
        Gtk::Button bkSequenceButton;

        Gtk::CheckButton allowVirtualEffectorsBox;
        Gtk::CheckButton useJacobianTransposeBox;
        Gtk::CheckButton usePDControlBox;

        Gtk::Entry controllerGainEntry;
        Gtk::Label controllerGainLabel;

        Gtk::Entry sequenceGainEntry;
        Gtk::Label sequenceGainLabel;

        ResourceList sensorList;
        ResourceList referenceList;
        ResourceList potentialFunctionList;
        ResourceList effectorList;
        
        CBAPITextWindow controlDefinitionText;
        CBAPITextWindow controlOutputText;

        CBAPITextWindow sequenceControllersText;
        CBAPITextWindow sequenceOutputText;
        
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

        void on_add_to_sequence_button_clicked();
        void on_add_secondary_to_sequence_button_clicked();
        void on_clear_sequence_button_clicked();
        void on_run_sequence_button_clicked();
        void on_stop_sequence_button_clicked();
        void on_bk_sequence_button_clicked();
        void on_fwd_sequence_button_clicked();

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
        bool sequenceRunning;
        bool showVirtualEffectors;
        bool useJacobianTranspose;
        bool usePDControl;
  };

}



#endif
