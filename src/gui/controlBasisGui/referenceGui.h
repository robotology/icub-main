// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _REFERENCE_WINDOW__H_
#define _REFERENCE_WINDOW__H_

#include <gtkmm.h>
#include <yarp/sig/Vector.h>

#include <iCubConfigurationReference.h>
#include <CartesianPositionReference.h>
#include <HeadingFovea.h>

namespace CB {
  
    class ReferenceWindow : public Gtk::Window {

    public:

        ReferenceWindow();
        virtual ~ReferenceWindow();

        void startResources();
        void stopResources();

    protected:

        const static int MAX_VALS = 10;
               
        // main widgets
        Gtk::VBox referenceVBox;
        Gtk::Notebook referenceNotebook;

        // table widgets for each reference
        Gtk::Table posTable;
        Gtk::Table armTable;
        Gtk::Table legTable;
        Gtk::Table fullArmTable;
        Gtk::Table headTable;
        Gtk::Table handTable;
        Gtk::Table torsoTable;
        Gtk::Table eyeTable;

        Gtk::Frame posFrame;
        Gtk::Frame armFrame;
        Gtk::Frame legFrame;
        Gtk::Frame fullArmFrame;
        Gtk::Frame headFrame;
        Gtk::Frame handFrame;
        Gtk::Frame torsoFrame;
        Gtk::Frame eyeFrame;

        Gtk::Table appTable;
        
        Gtk::Button resetButton;
        Gtk::Button setButton;

        Gtk::Entry posEntry[MAX_VALS];
        Gtk::Label posLabel[MAX_VALS];

        Gtk::Entry armEntry[MAX_VALS];
        Gtk::Label armLabel[MAX_VALS];

        Gtk::Entry legEntry[MAX_VALS];
        Gtk::Label legLabel[MAX_VALS];

        Gtk::Entry handEntry[MAX_VALS];
        Gtk::Label handLabel[MAX_VALS];

        Gtk::Entry headEntry[MAX_VALS];
        Gtk::Label headLabel[MAX_VALS];

        Gtk::Entry torsoEntry[MAX_VALS];
        Gtk::Label torsoLabel[MAX_VALS];

        Gtk::Entry fullArmEntry[MAX_VALS];
        Gtk::Label fullArmLabel[MAX_VALS];

        Gtk::Entry eyePanTiltEntry[MAX_VALS];
        Gtk::Label eyePanTiltLabel[MAX_VALS];

        Gtk::Entry eyePanTiltVergeEntry[MAX_VALS];
        Gtk::Label eyePanTiltVergeLabel[MAX_VALS];

        void on_set_button_clicked();
        void on_reset_button_clicked();
        void on_notebook_switch_page(GtkNotebookPage *page, guint page_num);     

        // reference services
        iCubConfigurationReference *iCubArmRef;
        iCubConfigurationReference *iCubFullArmRef;
        iCubConfigurationReference *iCubLegRef;
        iCubConfigurationReference *iCubHandRef;
        iCubConfigurationReference *iCubHeadRef;
        iCubConfigurationReference *iCubTorsoRef;
        iCubConfigurationReference *iCubEyeRef;
        iCubConfigurationReference *iCubEyePanTiltRef;
        iCubConfigurationReference *iCubEyePanTiltVergeRef;
        CartesianPositionReference *iCubPositionRef;
        HeadingFovea *fovea;

        yarp::sig::Vector armRef;
        yarp::sig::Vector fullArmRef;
        yarp::sig::Vector legRef;
        yarp::sig::Vector headRef;
        yarp::sig::Vector handRef;
        yarp::sig::Vector torsoRef;
        yarp::sig::Vector posRef;
        yarp::sig::Vector eyePanTiltRef;
        yarp::sig::Vector eyePanTiltVergeRef;

        bool resourcesStarted;

  };

}



#endif
