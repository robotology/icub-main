// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "controlBasisGui.h"

#include <yarp/String.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/impl/NameClient.h>
#include <yarp/os/impl/NameConfig.h>

#include "PotentialFunctionRegister.h"
#include "JacobianRegister.h"

using namespace CB;
using namespace std;
using namespace yarp::os;
using namespace yarp::os::impl;

CBAPIWindow::CBAPIWindow() :
    cbapiTable(2,2,true),
    controlTabTable(1,2,true),
    sequenceTabTable(1,2,true),
    controlResourcesTable(6,9,true),
    controlDefinitionTable(8,12,true),
    sequenceControllersTable(8,12,true),
    optionsTable(3,1,true),
    sensorList("sensors"),
    referenceList("reference"),
    potentialFunctionList("potential functions"),
    effectorList("effectors"),
    controlDefinitionText("blue","white"),
    controlOutputText("green","black"),
    sequenceControllersText("black","gray"),
    sequenceOutputText("purple","black"),
    addControllerButton(Gtk::Stock::ADD),
    clearControllerButton(Gtk::Stock::CLEAR),
    runControllerButton(Gtk::Stock::MEDIA_PLAY),
    stopControllerButton(Gtk::Stock::MEDIA_STOP),
    refreshButton(Gtk::Stock::REFRESH),
    addControllerToSequenceButton(Gtk::Stock::ADD),
    clearSequenceButton(Gtk::Stock::CLEAR),
    runSequenceButton(Gtk::Stock::MEDIA_PLAY),
    stopSequenceButton(Gtk::Stock::MEDIA_STOP),
    fwdSequenceButton(Gtk::Stock::MEDIA_FORWARD),
    bkSequenceButton(Gtk::Stock::MEDIA_REWIND),
    allowVirtualEffectorsBox("allow virtual effectors"),
    useJacobianTransposeBox("use Jacobian Transpose"),
    usePDControlBox("use PD-Control"),
    controllerGainLabel("gain:"),
    sequenceGainLabel("gain:")
{

    registerPotentialFunctions();
    registerJacobians();

    controlLawRunning = false;
    sequenceRunning = false;
    showVirtualEffectors = false;
    useJacobianTranspose = true;
    usePDControl = true;
    useJacobianTransposeBox.set_active(useJacobianTranspose);
    usePDControlBox.set_active(usePDControl);
    cbapi.useTranspose(useJacobianTranspose);
    cbapi.usePDControl(usePDControl);

    set_title("Control Basis API GUI");
    set_size_request(1200,800);
    set_border_width(6);
    //set_icon_from_file("cb.png");

    addControllerButton.set_border_width(4);
    clearControllerButton.set_border_width(4);
    runControllerButton.set_border_width(4);
    stopControllerButton.set_border_width(4);
    refreshButton.set_border_width(2);

    addControllerToSequenceButton.set_border_width(4);
    clearSequenceButton.set_border_width(4);
    runSequenceButton.set_border_width(4);
    stopSequenceButton.set_border_width(4);
    fwdSequenceButton.set_border_width(4),
    bkSequenceButton.set_border_width(4),

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
    usePDControlBox.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_use_pd_control) );

    addControllerToSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                          &CBAPIWindow::on_add_to_sequence_button_clicked) );
    clearSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                &CBAPIWindow::on_clear_sequence_button_clicked) );
    runSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                &CBAPIWindow::on_run_sequence_button_clicked) );
    stopSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_stop_sequence_button_clicked) );
    fwdSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_fwd_sequence_button_clicked) );
    bkSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_bk_sequence_button_clicked) );

    add(cbapiVBox);
    cbapiVBox.pack_start(cbapiTable);
    cbapiTable.set_border_width(5);

    cbapiTable.attach(controlResourcesFrame,0,2,0,1);
    cbapiTable.attach(cbapiNotebook,0,2,1,2);

    cbapiNotebook.set_border_width(5);
    cbapiNotebook.set_tab_pos(Gtk::POS_BOTTOM);
    
    controlTabTable.set_border_width(5);
    sequenceTabTable.set_border_width(5);

    controlDefinitionTable.set_border_width(5);
    sequenceControllersTable.set_border_width(5);

    controlResourcesFrame.set_label("Controller Resources");
    controlResourcesFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);

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
    optionsTable.attach(usePDControlBox,0,1,2,3);
    controlResourcesTable.attach(optionsTable,7,9,5,6);

    controllerGainEntry.set_max_length(5);
    controllerGainEntry.set_text("1");

    sequenceGainEntry.set_max_length(5);
    sequenceGainEntry.set_text("1");
    
    controlResourcesFrame.add(controlResourcesTable);
   
    // for the control law tab
    controlDefinitionFrame.set_label("Controller Definition");
    controlDefinitionFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    controlDefinitionText.set_border_width(5);
    
    controlDefinitionTable.attach(controlDefinitionText,0,8,0,8);
    controlDefinitionTable.attach(addControllerButton,8,10,0,1); 
    controlDefinitionTable.attach(runControllerButton,8,10,1,2);
    controlDefinitionTable.attach(stopControllerButton,8,10,2,3);
    controlDefinitionTable.attach(clearControllerButton,8,10,3,4);

    controlDefinitionTable.attach(controllerGainLabel,10,11,0,1); 
    controlDefinitionTable.attach(controllerGainEntry,11,12,0,1); 

    controlDefinitionFrame.add(controlDefinitionTable);
    controlTabTable.attach(controlDefinitionFrame,0,1,0,2);

    controlOutputFrame.set_label("Controller Output");
    controlOutputFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    controlOutputText.set_border_width(5);
    controlOutputFrame.add(controlOutputText);
    controlTabTable.attach(controlOutputFrame,1,2,0,2);

    // for the sequence tab
    sequenceControllersFrame.set_label("Control Sequence");
    sequenceControllersFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    sequenceControllersText.set_border_width(5);

    sequenceControllersTable.attach(sequenceControllersText,0,8,0,8);
    sequenceControllersTable.attach(addControllerToSequenceButton,8,10,0,1); 
    sequenceControllersTable.attach(runSequenceButton,8,10,2,3);
    sequenceControllersTable.attach(fwdSequenceButton,8,10,3,4);
    sequenceControllersTable.attach(bkSequenceButton,8,10,4,5);
    sequenceControllersTable.attach(stopSequenceButton,8,10,5,6);
    sequenceControllersTable.attach(clearSequenceButton,8,10,7,8);

    sequenceControllersTable.attach(sequenceGainLabel,10,11,0,1); 
    sequenceControllersTable.attach(sequenceGainEntry,11,12,0,1); 

    sequenceControllersFrame.add(sequenceControllersTable);
    sequenceTabTable.attach(sequenceControllersFrame,0,1,0,2);

    sequenceOutputFrame.set_label("Sequence Output");
    sequenceOutputFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    sequenceOutputText.set_border_width(5);
    sequenceOutputFrame.add(sequenceOutputText);
    sequenceTabTable.attach(sequenceOutputFrame,1,2,0,2);

    cbapiNotebook.append_page(controlTabTable, "Control Laws");
    cbapiNotebook.append_page(sequenceTabTable, "Sequences");
    cbapiNotebook.signal_switch_page().connect(sigc::mem_fun(*this, &CBAPIWindow::on_notebook_switch_page) );

    show_all_children();
    
    cout << "test 0\n";

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

CBAPIWindow::~CBAPIWindow() { 
    for(int i=0; i<pfInfo.size(); i++) 
        delete pfInfo[i];
    pfInfo.clear();
}


void CBAPIWindow::on_control_thread_update(ControlDataThread *dThread) {

    string str = dataThread->getOutputString();

    if(sequenceRunning) {
        sequenceOutputText.append_text(str);
        
        // check to see if the sequence needs to transition
        if ( (cbapi.getState(0, true) == CB::CONVERGED) || 
             (cbapi.getState(0,true) == CB::UNDEFINED) ) {           
            cout << "controller[" << cbapi.getSequenceControllerID() << "] state: " << (int)(cbapi.getState(0,true)) << endl;            

            if(cbapi.getSequenceControllerID() == (cbapi.getNumControllers(true)-1)) {
                dataThread->stopUpdateThread();
                cbapi.stopSequence();
                sequenceOutputText.append_text("control sequence finished\n");
                sequenceRunning = false;
            } else {
                cbapi.goToNextControllerInSequence();
            }
        } 

    } else {
        controlOutputText.append_text(str);
    }
}

void CBAPIWindow::on_notebook_switch_page(GtkNotebookPage* /* page */, guint page_num) {
    cout << "CBAPI Switching to tab " << page_num << endl;
}

void CBAPIWindow::on_sensor_selection() {
    cout << "got sensor selection" << endl;
    string sen = sensorList.getSelected();
    if(sen=="") return;
}

void CBAPIWindow::on_reference_selection() {
    cout << "got reference selection" << endl;
    string ref = referenceList.getSelected();
    if(ref=="") return;
}

void CBAPIWindow::on_potential_function_selection() {
    cout << "got pf selection" << endl;
    string pf = potentialFunctionList.getSelected();
    if(pf=="") return;

    refreshResourceList();
    
    int id;
    for(id=0; id<pfInfo.size(); id++) {
        if(pfInfo[id]->name == pf) break;
    }

    referenceList.clear();
    sensorList.clear();

    printf("sensor 1 info size: %d\n", sensorInfo.size());
    for(int i=0; i<sensorInfo.size(); i) {
        printf("testing sensor %s %s\n", sensorInfo[i].space.c_str(), sensorInfo[i].name.c_str());
        if(sensorInfo[i].space != pfInfo[id]->space) {
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
        if(pfInfo[id]->hasReference) {
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

void CBAPIWindow::on_use_pd_control() {
    cout << "got use pd control toggle" << endl;
    if(usePDControlBox.get_active()) {
        usePDControl = true;
    } else {
        usePDControl = false;
    }
    cbapi.usePDControl(usePDControl);
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

void CBAPIWindow::loadPotentialFunctionsFromFile() {

    FILE *fp;
    string ICUB_ROOT(getenv("ICUB_ROOT"));    
    string fname = ICUB_ROOT + "/src/controlBasis/potentialFunctions/potentialFunctions.dat";
    cout << fname << endl;

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
       
    potentialFunctionList.clear();
    
    while(fgets(line, 128, fp) != NULL) {

        PotentialFunctionInfo *info = new PotentialFunctionInfo();        

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

        info->name = pfName;
        info->space = pfSpace;
        if(pfHasReference=="true") 
            info->hasReference = true; 
        else 
            info->hasReference = false; 
        pfInfo.push_back(info);
        
        potentialFunctionList.addResource(pfName);
        
    }

    fclose(fp);
}

void CBAPIWindow::loadPotentialFunctions() {

    for(int i=0; i<pfInfo.size(); i++) {
        delete pfInfo[i];
    }
    pfInfo.clear();
    potentialFunctionList.clear();

    for(int i=0; i<PotentialFunctionFactory::instance().getNumRegisteredPotentialFunctions(); i++) {

        PotentialFunctionInfo *info = new PotentialFunctionInfo();
        info->name = PotentialFunctionFactory::instance().getPotentialFunctionInfo(i).name;
        info->space = PotentialFunctionFactory::instance().getPotentialFunctionInfo(i).space;
        info->hasReference = PotentialFunctionFactory::instance().getPotentialFunctionInfo(i).hasReference;
        pfInfo.push_back(info);
        potentialFunctionList.addResource(PotentialFunctionFactory::instance().getName(i));   

        cout << "Loaded PF: " << PotentialFunctionFactory::instance().getName(i).c_str() << endl;
    }
            
}

void CBAPIWindow::loadJacobians() {

    FILE *fp;
    string ICUB_ROOT(getenv("ICUB_ROOT"));    
    string fname = ICUB_ROOT + "/src/controlBasis/jacobians/jacobians.dat";
    cout << fname << endl;

    if( (fp=fopen(fname.c_str(), "r")) == NULL ) {
        cout << "problem opening \'" << fname.c_str() << "\' for reading!!!" << endl;
        jacInfo.clear();
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
        if(pfInfo[id]->name == pf) break;
    }
    if(pfInfo[id]->hasReference) {
        cout << "Please select control resources" << endl;
        if(ref=="") return;
    }

    string gainStr = controllerGainEntry.get_text();
    float gain = 1;
    cout << "got input gain: " << gainStr << endl;
    sscanf(gainStr.c_str(),"%f",&gain);
    cout << "Setting controller gain: " << gain << endl;

    cout << "adding controller: " << endl;
    cout << "\t " << pf.c_str() << endl;
    cout << "\t " << sen.c_str() << endl;
    if(ref != "") cout << "\t " << ref.c_str() << endl;
    cout << "\t " << eff.c_str() << endl;

    cbapi.addControllerToLaw(sen, ref, pf, eff, useJacobianTranspose, (double)gain);
    cbapi.usePDControl(usePDControl);

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
    if(controlLawRunning) {
        dataThread->stopUpdateThread();
        cbapi.stopControlLaw();
        controlOutputText.append_text("stopping control law\n");
        controlLawRunning = false;
    }
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
    if(sequenceRunning) {
        cout << "CBAPI Can't run control law because sequence is running!!" << endl;
        return;
    }
    if(cbapi.getNumControllers()>0) {       
        controlOutputText.append_text("running control law\n");
        cbapi.runControlLaw();
        dataThread->connectCBAPIObjects(&cbapi,&controlOutputText, false);
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

void CBAPIWindow::on_add_to_sequence_button_clicked() { 
    cout << "ADD TO SEQUENCE" << endl; 

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
        if(pfInfo[id]->name == pf) break;
    }
    if(pfInfo[id]->hasReference) {
        cout << "Please select control resources" << endl;
        if(ref=="") return;
    }

    string gainStr = sequenceGainEntry.get_text();
    float gain = 1;
    cout << "got input gain: " << gainStr << endl;
    sscanf(gainStr.c_str(),"%f",&gain);
    cout << "Setting controller gain: " << gain << endl;

    cout << "adding controller: " << endl;
    cout << "\t " << pf.c_str() << endl;
    cout << "\t " << sen.c_str() << endl;
    if(ref != "") cout << "\t " << ref.c_str() << endl;
    cout << "\t " << eff.c_str() << endl;

    cbapi.addControllerToSequence(sen, ref, pf, eff, useJacobianTranspose, (double)gain);
    cbapi.usePDControl(usePDControl);

    char c[32];
    int n = cbapi.getNumControllers(true)-1;
    sprintf(c, "%d", n);
    string nStr = string(c);

    sequenceControllersText.append_text("Controller[" + nStr + "]\n");
    sequenceControllersText.append_text("\tsensor: " + sen +"\n");
    if(ref != "") {
        sequenceControllersText.append_text("\tref: " + ref +"\n");
    }
    sequenceControllersText.append_text("\tpf: " + pf +"\n");
    sequenceControllersText.append_text("\teffector: " + eff +"\n");
    sequenceOutputText.append_text("added controller " + nStr +" to sequence\n");

}

void CBAPIWindow::on_clear_sequence_button_clicked() { 
    cout << "CLEAR SEQUENCE" << endl; 
    if(sequenceRunning) {
        dataThread->stopUpdateThread();
        cbapi.stopSequence();
        sequenceOutputText.append_text("stopping control sequence\n");
        sequenceRunning = false;
    }
    if(cbapi.getNumControllers(true)>0) {
        cout << "clearing sequence" << endl; 
        cbapi.clearSequence();
        sequenceControllersText.clear_text();
        sequenceOutputText.append_text("clearing sequence\n");
    }
}

void CBAPIWindow::on_run_sequence_button_clicked() { 
    cout << "RUN SEQUENCE" << endl; 
    if(controlLawRunning) {
        cout << "CBAPI Can't run sequence because control law is running!!" << endl;
        return;
    }
    if(cbapi.getNumControllers(true)>0) {
        sequenceOutputText.append_text("running sequence\n");
        cbapi.runSequence();
        dataThread->connectCBAPIObjects(&cbapi,&sequenceOutputText, true);
        dataThread->startUpdateThread();
        sequenceRunning = true;
    }
}

void CBAPIWindow::on_fwd_sequence_button_clicked() { 
    cout << "SEQUENCE FWD" << endl; 
    if(sequenceRunning) {
        cbapi.goToNextControllerInSequence();        
    } else {
        cout << "Sequence not running, can't proceed!!" << endl;
    }
}

void CBAPIWindow::on_bk_sequence_button_clicked() { 
    cout << "SEQUENCE BK" << endl; 
    if(sequenceRunning) {
        cbapi.goToPreviousControllerInSequence();        
    } else {
        cout << "Sequence not running, can't go back!!" << endl;
    }
}

void CBAPIWindow::on_stop_sequence_button_clicked() { 
    cout << "STOP SEQUENCE" << endl; 
    if(sequenceRunning) {
        dataThread->stopUpdateThread();
        cbapi.stopSequence();
        sequenceOutputText.append_text("stopping control sequence\n");
        sequenceRunning = false;
    }
    cout << "STOP FINISHED" << endl; 
}

int main(int argc, char *argv[]) {
    Gtk::Main kit(argc,argv);
    CBAPIWindow mainWindow;
    Gtk::Main::run(mainWindow); 
    cout << "Success!!" << endl << endl;
    return 1;
}

