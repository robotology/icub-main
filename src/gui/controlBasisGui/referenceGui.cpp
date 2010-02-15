// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "referenceGui.h"

#include <iostream>
#include <ace/ACE.h>

using namespace CB;
using namespace std;

static void close_handler(int);

ReferenceWindow::ReferenceWindow() :
    appTable(13,4,true),
    posTable(11,3,true),
    armTable(11,3,true),
    fullArmTable(11,3,true),
    legTable(11,3,true),
    handTable(11,3,true),
    headTable(11,3,true),
    torsoTable(11,3,true),
    setButton("Set", true),
    resetButton("Reset", true)
{

    int k = 0;
    string str;
    int max_entry_length=10;

    set_title("Reference GUI");
    set_size_request(400,500);
    set_border_width(10);
    //    set_icon_from_file("app/cb.png");

    setButton.set_border_width(4);
    resetButton.set_border_width(4);

    setButton.signal_clicked().connect( sigc::mem_fun(*this, &ReferenceWindow::on_set_button_clicked) );
    resetButton.signal_clicked().connect( sigc::mem_fun(*this, &ReferenceWindow::on_reset_button_clicked) );

    add(referenceVBox);
    referenceVBox.pack_start(appTable);
    referenceNotebook.set_border_width(5);

    appTable.attach(referenceNotebook,0,4,0,13);
    appTable.attach(resetButton,1,2,13,14);
    appTable.attach(setButton,2,3,13,14);

    posFrame.set_label("Cartesian Position Reference");
    armFrame.set_label("iCub Arm Reference");
    legFrame.set_label("iCub Leg Reference");
    fullArmFrame.set_label("iCub Torso+Arm Reference");
    handFrame.set_label("iCub Hand Reference");
    headFrame.set_label("iCub Head Reference");
    torsoFrame.set_label("iCub Torso Reference");

    posFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    armFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    legFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    fullArmFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    handFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    headFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    torsoFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);

    posFrame.set_border_width(15);
    armFrame.set_border_width(15);
    legFrame.set_border_width(15);
    fullArmFrame.set_border_width(15);
    handFrame.set_border_width(15);
    headFrame.set_border_width(15);
    torsoFrame.set_border_width(15);

    // position reference
    posLabel[0].set_text("x:");
    posLabel[1].set_text("y:");
    posLabel[2].set_text("z:");

    for(int i=0; i<3; i++) {
        posEntry[i].set_text("0.0");
        posEntry[i].set_max_length(max_entry_length);
        posTable.attach(posLabel[i],0,1,i,i+1);
        posTable.attach(posEntry[i],1,2,i,i+1);
    }

    // arm reference
    for(int i=0; i<7; i++) {
        stringstream out;
        out << i;
        str = "j" + out.str() + ":";
        armLabel[i].set_text(str);
        armEntry[i].set_text("0.0");
        armEntry[i].set_max_length(max_entry_length);
        armTable.attach(armLabel[i],0,1,i,i+1);
        armTable.attach(armEntry[i],1,2,i,i+1);
    }

    // leg reference
    for(int i=0; i<6; i++) {
        stringstream out;
        out << i;
        str = "j" + out.str() + ":";
        legLabel[i].set_text(str);
        legEntry[i].set_text("0.0");
        legEntry[i].set_max_length(max_entry_length);
        legTable.attach(legLabel[i],0,1,i,i+1);
        legTable.attach(legEntry[i],1,2,i,i+1);
    }

    // hand reference
    for(int i=0; i<9; i++) {
        stringstream out;
        out << i;
        str = "j" + out.str() + ":";
        handLabel[i].set_text(str);
        handEntry[i].set_text("0.0");
        handEntry[i].set_max_length(max_entry_length);
        handTable.attach(handLabel[i],0,1,i,i+1);
        handTable.attach(handEntry[i],1,2,i,i+1);
    }

    // head reference
    for(int i=0; i<3; i++) {
        stringstream out;
        out << i;
        str = "j" + out.str() + ":";
        headLabel[i].set_text(str);
        headEntry[i].set_text("0.0");
        headEntry[i].set_max_length(max_entry_length);
        headTable.attach(headLabel[i],0,1,i,i+1);
        headTable.attach(headEntry[i],1,2,i,i+1);
    }

    // torso reference
    for(int i=0; i<3; i++) {
        stringstream out;
        out << i;
        str = "j" + out.str() + ":";
        torsoLabel[i].set_text(str);
        torsoEntry[i].set_text("0.0");
        torsoEntry[i].set_max_length(max_entry_length);
        torsoTable.attach(torsoLabel[i],0,1,i,i+1);
        torsoTable.attach(torsoEntry[i],1,2,i,i+1);
    }

    // fullArm reference
    for(int i=0; i<10; i++) {
        stringstream out;
        out << i;
        str = "j" + out.str() + ":";
        fullArmLabel[i].set_text(str);
        fullArmEntry[i].set_text("0.0");
        fullArmEntry[i].set_max_length(max_entry_length);
        fullArmTable.attach(fullArmLabel[i],0,1,i,i+1);
        fullArmTable.attach(fullArmEntry[i],1,2,i,i+1);
    }

    posFrame.add(posTable);
    armFrame.add(armTable);
    legFrame.add(legTable);
    fullArmFrame.add(fullArmTable);
    headFrame.add(headTable);
    handFrame.add(handTable);
    torsoFrame.add(torsoTable);

    referenceNotebook.append_page(posFrame, "Position");
    referenceNotebook.append_page(armFrame, "Arm");
    referenceNotebook.append_page(legFrame, "Leg");
    referenceNotebook.append_page(fullArmFrame, "Full Arm");
    referenceNotebook.append_page(handFrame, "Hand");
    referenceNotebook.append_page(headFrame, "Head");
    referenceNotebook.append_page(torsoFrame, "Torso");

    referenceNotebook.signal_switch_page().connect(sigc::mem_fun(*this, &ReferenceWindow::on_notebook_switch_page) );

    show_all_children();

    startResources();

}

ReferenceWindow::~ReferenceWindow() { 

    stopResources();

    delete iCubArmRef;
    delete iCubFullArmRef;
    delete iCubLegRef;
    delete iCubHandRef;
    delete iCubHeadRef;
    delete iCubTorsoRef;
    delete iCubPositionRef;

}

void ReferenceWindow::on_notebook_switch_page(GtkNotebookPage* /* page */, guint page_num) {
}

void ReferenceWindow::on_set_button_clicked() { 
    cout << "Set" << endl; 

    int currentPageNum = referenceNotebook.get_current_page();

    switch(currentPageNum) {
    case 0:
        for(int i=0; i<3; i++) posRef[i] = atof(posEntry[i].get_text().c_str());       
        cout << "pos: (" << posRef[0] << ", " << posRef[1] << ", " << posRef[2] << ")" << endl;
        iCubPositionRef->setVals(posRef);
        break;
    case 1:
        for(int i=0; i<7; i++) armRef[i] = atof(armEntry[i].get_text().c_str());       
        iCubArmRef->setVals(armRef);
        break;
    case 2:
        for(int i=0; i<6; i++) legRef[i] = atof(legEntry[i].get_text().c_str());       
        iCubLegRef->setVals(legRef);
        break;
    case 3:
        for(int i=0; i<10; i++) fullArmRef[i] = atof(fullArmEntry[i].get_text().c_str());       
        iCubFullArmRef->setVals(fullArmRef);
        break;
    case 4:
        for(int i=0; i<9; i++) handRef[i] = atof(handEntry[i].get_text().c_str());       
        iCubHandRef->setVals(handRef);
        break;
    case 5:
        for(int i=0; i<3; i++) headRef[i] = atof(headEntry[i].get_text().c_str());       
        iCubHeadRef->setVals(headRef);
        break;
    case 6:
        for(int i=0; i<3; i++) torsoRef[i] = atof(torsoEntry[i].get_text().c_str());       
        iCubTorsoRef->setVals(torsoRef);
        break;
    }

}

void ReferenceWindow::on_reset_button_clicked() { 

    int currentPageNum = referenceNotebook.get_current_page();

    switch(currentPageNum) {
    case 0:
        posRef.zero();
        for(int i=0; i<3; i++) {
            posEntry[i].set_text("0.0");
        }
        break;
    case 1:
        armRef.zero();
        for(int i=0; i<7; i++) {
            armEntry[i].set_text("0.0");
        }
        break;
    case 2:
        legRef.zero();
        for(int i=0; i<6; i++) {
            legEntry[i].set_text("0.0");
        }
        break;
    case 3:
        fullArmRef.zero();
        for(int i=0; i<10; i++) {
            fullArmEntry[i].set_text("0.0");
        }
        break;
    case 4:
        handRef.zero();
        for(int i=0; i<9; i++) {
            handEntry[i].set_text("0.0");
        }
        break;
    case 5:
        headRef.zero();
        for(int i=0; i<3; i++) {
            headEntry[i].set_text("0.0");
        }
        break;
    case 6:
        torsoRef.zero();
        for(int i=0; i<3; i++) {
            torsoEntry[i].set_text("0.0");
        }
        break;
    }

}

void ReferenceWindow::startResources() {

    armRef.resize(7);
    armRef.zero();
    legRef.resize(6);
    legRef.zero();
    handRef.resize(9);
    handRef.zero();
    headRef.resize(3);
    headRef.zero();
    torsoRef.resize(3);
    torsoRef.zero();
    posRef.resize(3);
    posRef.zero();

    // start up references
    cout << endl << "creating iCub arm reference..." << endl << endl;
    iCubArmRef = new iCubConfigurationReference("/icub/arm", 7);  
    iCubArmRef->setVals(armRef);
    iCubArmRef->setUpdateDelay(0.25);
    iCubArmRef->startResource();

    iCubFullArmRef = new iCubConfigurationReference("/icub/fullarm", 10);  
    iCubFullArmRef->setVals(fullArmRef);
    iCubFullArmRef->setUpdateDelay(0.25);
    iCubFullArmRef->startResource();
    
    iCubLegRef = new iCubConfigurationReference("/icub/leg", 6);  
    iCubLegRef->setVals(armRef);
    iCubLegRef->setUpdateDelay(0.25);
    iCubLegRef->startResource();

    iCubHeadRef = new iCubConfigurationReference("/icub/head", 3);  
    iCubHeadRef->setVals(headRef);
    iCubHeadRef->setUpdateDelay(0.25);
    iCubHeadRef->startResource();

    iCubHandRef = new iCubConfigurationReference("/icub/hand", 9);  
    iCubHandRef->setVals(handRef);
    iCubHandRef->setUpdateDelay(0.25);
    iCubHandRef->startResource();

    iCubTorsoRef = new iCubConfigurationReference("/icub/torso", 3);  
    iCubTorsoRef->setVals(torsoRef);
    iCubTorsoRef->setUpdateDelay(0.25);
    iCubTorsoRef->startResource();

    iCubPositionRef = new CartesianPositionReference("/icub/pos");
    iCubPositionRef->setVals(posRef);
    iCubPositionRef->setUpdateDelay(0.2);
    iCubPositionRef->startResource();

    resourcesStarted = true;

}

void ReferenceWindow::stopResources() {

    cout << "stopping resources..." << endl;
    if(resourcesStarted) {
        iCubArmRef->stopResource();
        iCubFullArmRef->stopResource();
        iCubLegRef->stopResource();
        iCubHandRef->stopResource();
        iCubHeadRef->stopResource();
        iCubTorsoRef->stopResource();
        iCubPositionRef->stopResource();
        resourcesStarted = false;
    }
}

int main(int argc, char *argv[]) {

    Gtk::Main kit(argc,argv);
    ReferenceWindow mainWindow;

    ACE_OS::signal(SIGINT, (ACE_SignalHandler) close_handler);
    ACE_OS::signal(SIGTERM, (ACE_SignalHandler) close_handler);

    Gtk::Main::run(mainWindow); 
    cout << "Success!!" << endl << endl;
    return 1;

}


static void close_handler(int) {
    std::cout << "Shutting down..." << std::endl;
    Gtk::Main::quit();
}


