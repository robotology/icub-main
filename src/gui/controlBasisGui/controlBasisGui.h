// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
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
