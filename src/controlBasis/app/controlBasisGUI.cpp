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

\delta \effector = gain*potential*Jacobian^#,

where "#" is the generalized (Moore-Penrose) pseudoinverse.  If the "Use Jacobian Transpose"
button is selected, the transpose is used in place of the pseudoinverse.  
 
The GUI will look like this
\image html controlBasisGUI.jpg

\section example_sec Example 
Try with the following:
 
\code
on terminal 1: iCub_SIM

on terminal 2: src/controlBasis/tests/startResources
- type "start" and hit enter

on terminal 3: controlBasisGUI

\author Stephen Hart
 
This file can be edited at 
\in src/controlBasis/app/controlBasisGUI.cpp.
*/
#include "controlBasisGUI.h"

#include <yarp/String.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/impl/NameClient.h>
#include <yarp/os/impl/NameConfig.h>

using namespace CB;
using namespace std;
using namespace yarp::os;
using namespace yarp::os::impl;

CBAPIWindow::CBAPIWindow() :
    controlTabTable(2,2,true),
    controlResourcesTable(6,9,true),
    controlDefinitionTable(8,12,true),
    optionsTable(2,1,true),
    sensorList("sensors"),
    referenceList("reference"),
    potentialFunctionList("potential functions"),
    effectorList("effectors"),
    controlDefinitionText("blue","white"),
    controlOutputText("green","black"),
    addControllerButton(Gtk::Stock::ADD),
    clearControllerButton(Gtk::Stock::CLEAR),
    runControllerButton("Run", true),
    //runControllerButton(Gtk::Stock::PLAY),
    stopControllerButton(Gtk::Stock::STOP),
    refreshButton(Gtk::Stock::REFRESH),
    allowVirtualEffectorsBox("allow virtual effectors"),
    useJacobianTransposeBox("use Jacobian Transpose"),
    gainLabel("gain:")
{

    controlLawRunning = false;
    showVirtualEffectors = false;
    useJacobianTranspose = true;
    useJacobianTransposeBox.set_active(useJacobianTranspose);
    cbapi.useTranspose(useJacobianTranspose);

    set_title("Control Basis API GUI");
    set_size_request(1200,750);
    set_border_width(10);
    set_icon_from_file("app/cb.png");

    addControllerButton.set_border_width(4);
    clearControllerButton.set_border_width(4);
    runControllerButton.set_border_width(4);
    stopControllerButton.set_border_width(4);
    refreshButton.set_border_width(2);

    addControllerButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                &CBAPIWindow::on_add_button_clicked) );
    clearControllerButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                  &CBAPIWindow::on_clear_button_clicked) );
    runControllerButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                &CBAPIWindow::on_run_button_clicked) );
    stopControllerButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_stop_button_clicked) );
    refreshButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                          &CBAPIWindow::on_refresh_button_clicked) ); 

    allowVirtualEffectorsBox.signal_clicked().connect( sigc::mem_fun(*this,
                                                                  &CBAPIWindow::on_virtual_effector_toggle) );
    useJacobianTransposeBox.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_use_jacobian_transpose) );

    add(cbapiVBox);
    cbapiVBox.pack_start(cbapiNotebook);
    cbapiNotebook.set_border_width(5);
    controlTabTable.set_border_width(5);
    controlDefinitionTable.set_border_width(5);
    
    controlResourcesFrame.set_label("Controller Resources");
    controlResourcesFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    controlTabTable.attach(controlResourcesFrame,0,2,0,1);

    sensorList.set_border_width(3);
    referenceList.set_border_width(3);
    potentialFunctionList.set_border_width(3);
    effectorList.set_border_width(3);

    controlResourcesTable.attach(sensorList,0,3,0,3);
    controlResourcesTable.attach(referenceList,0,3,3,6);
    controlResourcesTable.attach(potentialFunctionList,3,6,0,6);
    controlResourcesTable.attach(effectorList,6,9,0,5);
    controlResourcesTable.attach(refreshButton,6,7,5,6);

    optionsTable.attach(allowVirtualEffectorsBox,0,1,0,1);
    optionsTable.attach(useJacobianTransposeBox,0,1,1,2);
    controlResourcesTable.attach(optionsTable,7,9,5,6);

    gainEntry.set_max_length(5);
    gainEntry.set_text("1");
    
    //  controlResourcesTable.set_col_spacings(1);
    //controlResourcesTable.set_row_spacings(2);
    controlResourcesFrame.add(controlResourcesTable);
    
    controlDefinitionFrame.set_label("Controller Definition");
    controlDefinitionFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    controlDefinitionText.set_border_width(5);
    
    controlDefinitionTable.attach(controlDefinitionText,0,8,0,8);
    controlDefinitionTable.attach(addControllerButton,8,10,0,1); 
    controlDefinitionTable.attach(runControllerButton,8,10,1,2);
    controlDefinitionTable.attach(stopControllerButton,8,10,2,3);
    controlDefinitionTable.attach(clearControllerButton,8,10,3,4);

    controlDefinitionTable.attach(gainLabel,10,11,0,1); 
    controlDefinitionTable.attach(gainEntry,11,12,0,1); 

    controlDefinitionFrame.add(controlDefinitionTable);
    controlTabTable.attach(controlDefinitionFrame,0,1,1,2);

    controlOutputFrame.set_label("Controller Output");
    controlOutputFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    controlOutputText.set_border_width(5);
    controlOutputFrame.add(controlOutputText);
    controlTabTable.attach(controlOutputFrame,1,2,1,2);

    cbapiNotebook.append_page(controlTabTable, "Control");
    cbapiNotebook.signal_switch_page().connect(sigc::mem_fun(*this, &CBAPIWindow::on_notebook_switch_page) );

    show_all_children();
    
    loadPotentialFunctions();
    loadJacobians();
    refreshResourceList();

    sensorTreeSelection = sensorList.getTreeSelection();
    referenceTreeSelection = referenceList.getTreeSelection();
    pfTreeSelection = potentialFunctionList.getTreeSelection();
    effectorTreeSelection = effectorList.getTreeSelection();
    
    sensorTreeSelection->signal_changed().connect(sigc::mem_fun(*this, &CBAPIWindow::on_sensor_selection));
    referenceTreeSelection->signal_changed().connect(sigc::mem_fun(*this, &CBAPIWindow::on_reference_selection));
    pfTreeSelection->signal_changed().connect(sigc::mem_fun(*this, &CBAPIWindow::on_potential_function_selection));
    effectorTreeSelection->signal_changed().connect(sigc::mem_fun(*this, &CBAPIWindow::on_effector_selection));

    dataThread = new ControlDataThread();
    dataThread->control_thread_update_finished().connect(sigc::bind<1>(sigc::mem_fun(*this, &CBAPIWindow::on_control_thread_update), dataThread));

}

CBAPIWindow::~CBAPIWindow() { }


void CBAPIWindow::on_control_thread_update(ControlDataThread *dThread) {
    string str = dataThread->getOutputString();
    controlOutputText.append_text(str);
}

void CBAPIWindow::on_notebook_switch_page(GtkNotebookPage* /* page */, guint page_num) {
    std::cout << "CBAPI Switching to tab " << page_num << std::endl;
}

void CBAPIWindow::on_sensor_selection() {
    std::cout << "got sensor selection" << endl;
    string sen = sensorList.getSelected();
    if(sen=="") return;
}

void CBAPIWindow::on_reference_selection() {
    std::cout << "got reference selection" << endl;
    string ref = referenceList.getSelected();
    if(ref=="") return;
}

void CBAPIWindow::on_potential_function_selection() {
    std::cout << "got pf selection" << endl;
    string pf = potentialFunctionList.getSelected();
    if(pf=="") return;

    refreshResourceList();
    int id;
    for(id=0; id<pfInfo.size(); id++) {
        if(pfInfo[id].name == pf) break;
    }

    printf("got PF: %s, space: %s, ref=%d\n", pfInfo[id].name.c_str(), pfInfo[id].space.c_str(), (int)(pfInfo[id].hasReference));

    referenceList.clear();
    sensorList.clear();

    printf("sensor 1 info size: %d\n", sensorInfo.size());
    for(int i=0; i<sensorInfo.size(); i) {
        printf("testing sensor %s %s\n", sensorInfo[i].space.c_str(), sensorInfo[i].name.c_str());
        if(sensorInfo[i].space != pfInfo[id].space) {
            sensorInfo.erase(sensorInfo.begin()+i);
        } else {
            i++;
        }
    }
    printf("sensor 2 info size: %d\n", sensorInfo.size());
    
    string str;
    for(int i=0; i<sensorInfo.size(); i++) {
        str = sensorInfo[i].space + sensorInfo[i].name;
        printf("adding sensor: %s\n", str.c_str());
        sensorList.addResource(str);
        if(pfInfo[id].hasReference) {
            referenceList.addResource(str);
        }
    }
    
}

void CBAPIWindow::on_effector_selection() {
    cout << "got effector selection" << endl;
    string eff = effectorList.getSelected();
    if(eff=="") return;
}

void CBAPIWindow::on_virtual_effector_toggle() {
    cout << "got virtual effector toggle" << endl;
    if(allowVirtualEffectorsBox.get_active()) {
        showVirtualEffectors = true;
    } else {
        showVirtualEffectors = false;
    }
    refreshResourceList();
}

void CBAPIWindow::on_use_jacobian_transpose() {
    cout << "got use transpose toggle" << endl;
    if(useJacobianTransposeBox.get_active()) {
        useJacobianTranspose = true;
    } else {
        useJacobianTranspose = false;
    }
    cbapi.useTranspose(useJacobianTranspose);
}

void CBAPIWindow::refreshResourceList() {
  
    cout << "RefreshResourceList()" << endl;
    string serviceName;
    string simpleServiceName;
    int start,stop;  
    string resourceName, resourceSpace;
    ResourceInfo resInfo;
    
    NameClient& nic = NameClient::getNameClient();
    NameConfig nc;
    String name = nc.getNamespace();
    Bottle msg, reply;
    msg.addString("bot");
    msg.addString("list");
    
    // clearing out current list
    sensorList.clear();
    referenceList.clear();
    effectorList.clear();
    
    sensorInfo.clear();
    effectorInfo.clear();
    
    cout << "Requesting list of ports from name server" << endl;
    Network::write(name.c_str(),
                   msg,
                   reply);
    int ct = reply.size()-1;
    cout << "Got " << ct << " port " << ((ct!=1)?"s":"") << endl ;
    for (int i=1; i<reply.size(); i++) {
        Bottle *entry = reply.get(i).asList();
        if (entry!=NULL) {
            ConstString port = entry->check("name",Value("")).asString();
            if (port!="" && port!="fallback" && port!=name.c_str()) {
                Contact c = Contact::byConfig(*entry);
                if (c.getCarrier()=="mcast") {
                    cout << "Skipping mcast port: " << port.c_str() << endl;
                } else {
                    Address addr = Address::fromContact(c);
                    
                    if (addr.isValid()) {
                        serviceName = port.c_str();
                        
                        if( (serviceName.compare(0,3,"/cb")==0) && (serviceName.compare(serviceName.size()-6,6,"data:o")==0) ) {
                            
                            if(serviceName.at(serviceName.size()-1)!='i') {
                                simpleServiceName = serviceName.substr(4,serviceName.size()-4-7);
                                
                                cout << "found control basis service: " << simpleServiceName.c_str() << endl;
                                
                                //model = gtk_
                                //gchar *msg = g_strdup_printf(simpleServiceName.c_str());
                                //gtk_list_store_append (GTK_LIST_STORE (model), &iter);
                                //gtk_list_store_set (GTK_LIST_STORE (model), &iter, 0, msg, -1);
                                
                                sensorList.addResource(simpleServiceName);
                                referenceList.addResource(simpleServiceName);
                                
                                start = 0;
                                stop = simpleServiceName.find_first_of("/", start);
                                resourceSpace = simpleServiceName.substr(start,stop-start);
                                resourceName = simpleServiceName.substr(stop-start,simpleServiceName.size());	      
                                resInfo.name = resourceName;
                                resInfo.space = resourceSpace;
                                sensorInfo.push_back(resInfo);
                                
                            }
                            //	      cout << "name=%s, space=%s\n", resourceName.c_str(), resourceSpace.c_str());
                            
                        } else if( (serviceName.compare(0,17,"/cb/configuration")==0) && (serviceName.compare(serviceName.size()-6,6,"data:i")==0) ) {
                            simpleServiceName = serviceName.substr(4,serviceName.size()-4-7);
                            
                            // load the runnable configurations
                            start = 0;
                            stop = simpleServiceName.find_first_of("/", start);
                            resourceSpace = simpleServiceName.substr(start,stop-start);
                            resourceName = simpleServiceName.substr(stop-start,simpleServiceName.size());	      
                            resInfo.name = resourceName;
                            resInfo.space = resourceSpace;
                            effectorInfo.push_back(resInfo);
                            effectorList.addResource(simpleServiceName);
                            cout << "found control basis effector: " << simpleServiceName.c_str() << endl;
                            
                            // now load any transformations on those configurations, possible through applying jacobians
                            if(showVirtualEffectors) {
                                for(int i=0; i<jacInfo.size(); i++) {
                                    if( (jacInfo[i].inputSpace == resourceSpace) ) {
                                        simpleServiceName = jacInfo[i].outputSpace + "/" + resourceName;
                                        effectorList.addResource(simpleServiceName);
                                        cout << "found virtual control basis effector: " << simpleServiceName.c_str() << endl;
                                    } else if( (jacInfo[i].outputSpace == resourceSpace) ) {
                                        simpleServiceName = jacInfo[i].inputSpace + resourceName;
                                        effectorList.addResource(simpleServiceName);
                                        cout << "found virtual control basis effector: " << simpleServiceName.c_str() << endl;
                                    }
                                }
                            }
                            
                        }
                        
                    }
                }
            } 
        } 
    }   
}

void CBAPIWindow::loadPotentialFunctions() {

    FILE *fp;
    string fname = "config/potentialFunctions.dat";

    if( (fp=fopen(fname.c_str(), "r")) == NULL ) {
        cout << "problem opening \'" << fname.c_str() << "\' for reading!!!" << endl;
        pfInfo.clear();
        return;
    }

    string pfName;
    string pfSpace;
    string pfHasReference;
    
    char line[128];
    string lineStr;
    
    int start,stop;
    
    PotentialFunctionInfo info;
    
    potentialFunctionList.clear();
    while(fgets(line, 128, fp) != NULL) {
        
        lineStr = string(line);
        
        start = 0;

        stop = lineStr.find_first_of(" \n", start);
        pfName = lineStr.substr(start,stop-start);
        start = lineStr.find_first_not_of(" ", stop+1);

        stop = lineStr.find_first_of(" \n", start);
        pfHasReference = lineStr.substr(start,stop-start);
        start = lineStr.find_first_not_of(" ", stop+1);

        stop = lineStr.find_first_of(" \n", start);
        pfSpace = lineStr.substr(start,stop-start);
        start = lineStr.find_first_not_of(" ", stop+1);

        info.name = pfName;
        info.space = pfSpace;
        if(pfHasReference=="true") 
            info.hasReference = true; 
        else 
            info.hasReference = false; 
        pfInfo.push_back(info);
        
        potentialFunctionList.addResource(pfName);
        
    }

    fclose(fp);
}

void CBAPIWindow::loadJacobians() {

    FILE *fp;
    string fname = "config/jacobians.dat";

    if( (fp=fopen(fname.c_str(), "r")) == NULL ) {
        cout << "problem opening \'" << fname.c_str() << "\' for reading!!!" << endl;
        pfInfo.clear();
        return;
    }
    
    string jacName;
    string jacSpaceIn;
    string jacSpaceOut;    
    char line[128];
    string lineStr;    
    int start,stop;
    JacobianInfo info;

    while(fgets(line, 128, fp) != NULL) {
        
        lineStr = string(line);
        start = 0;

        stop = lineStr.find_first_of(" \n", start);
        jacName = lineStr.substr(start,stop-start);
        start = lineStr.find_first_not_of(" ", stop+1);

        stop = lineStr.find_first_of(" \n", start);
        jacSpaceIn = lineStr.substr(start,stop-start);
        start = lineStr.find_first_not_of(" ", stop+1);
        
        stop = lineStr.find_first_of(" \n", start);
        jacSpaceOut = lineStr.substr(start,stop-start);
        start = lineStr.find_first_not_of(" ", stop+1);
        
        info.name = jacName;
        info.inputSpace = jacSpaceIn;
        info.outputSpace = jacSpaceOut;
        jacInfo.push_back(info);
        
    }
    
    fclose(fp);

}

void CBAPIWindow::on_add_button_clicked() { 

    cout << "ADD" << endl; 
    
    string sen = sensorList.getSelected();
    string ref = referenceList.getSelected();
    string pf = potentialFunctionList.getSelected();
    string eff = effectorList.getSelected();
    
    if( (sen=="") || (pf=="") || (eff=="") ) {
        cout << "Please select control resources" << endl;
        return;
    }
    
    int id;
    for(id=0; id<pfInfo.size(); id++) {
        if(pfInfo[id].name == pf) break;
    }
    if(pfInfo[id].hasReference) {
        cout << "Please select control resources" << endl;
        if(ref=="") return;
    }

    string gainStr = gainEntry.get_text();
    float gain = 1;
    cout << "got input gain: " << gainStr << endl;
    sscanf(gainStr.c_str(),"%f",&gain);
    cout << "Setting controller gain: " << gain << endl;
    cbapi.addControllerToLaw(sen, ref, pf, eff, useJacobianTranspose, (double)gain);
    
    char c[32];
    int n = cbapi.getNumControllers()-1;
    sprintf(c, "%d", n);
    string nStr = string(c);

    controlDefinitionText.append_text("Controller[" + nStr + "]\n");
    controlDefinitionText.append_text("\tsensor: " + sen +"\n");
    if(ref != "") {
        controlDefinitionText.append_text("\tref: " + ref +"\n");
    }
    controlDefinitionText.append_text("\tpf: " + pf +"\n");
    controlDefinitionText.append_text("\teffector: " + eff +"\n");
    controlOutputText.append_text("added controller " + nStr +" to law\n");
    
}

void CBAPIWindow::on_clear_button_clicked() { 
    cout << "CLEAR" << endl; 
    if(cbapi.getNumControllers()>0) {
        cout << "clearing control law" << endl; 
        cbapi.clearControlLaw();
        cout << "clearing text window" << endl; 
        controlDefinitionText.clear_text();
        controlOutputText.append_text("clearing control law\n");
    }
}

void CBAPIWindow::on_run_button_clicked() { 
    cout << "RUN" << endl; 
    if(cbapi.getNumControllers()>0) {
        controlOutputText.append_text("running control law\n");
        cbapi.runControlLaw();
        dataThread->connectCBPAIObjects(&cbapi,&controlOutputText);
        dataThread->startUpdateThread();
        controlLawRunning = true;
    }
}

void CBAPIWindow::on_stop_button_clicked() { 
    cout << "STOP" << endl; 
    if(controlLawRunning) {
        dataThread->stopUpdateThread();
        cbapi.stopControlLaw();
        controlOutputText.append_text("stopping control law\n");
        controlLawRunning = false;
    }
    cout << "STOP FINISHED" << endl; 
}

void CBAPIWindow::on_refresh_button_clicked() { 
    cout << "REFRESH" << endl; 
    refreshResourceList();
    loadPotentialFunctions();
}

int main(int argc, char *argv[]) {
    Gtk::Main kit(argc,argv);
    CBAPIWindow mainWindow;
    Gtk::Main::run(mainWindow); 
    cout << "Success!!" << endl << endl;
    return 1;
}

