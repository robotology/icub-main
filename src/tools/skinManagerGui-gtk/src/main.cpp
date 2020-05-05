/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrea Del Prete
 * email:   andrea.delprete@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
/**
 *
 * @ingroup icub_tools
 * @ingroup icub_guis
 * \defgroup icub_skinManagerGui skinManagerGui
 *
 * A simple graphical interface for controlling and monitoring an instance of the \ref icub_skinManager module.
 * This GUI needs at least the version 2.14 of GtkPlus.
 *
 * \image html driftCompensationGui_filters.png "Screenshots: skinManagerGui running on Windows"
 *
 * \section intro_sec Description
 *
 * This GUI can be used for the following purposes:
 *
 * - calibrate the skin
 * - see the touch threshold of each skin taxels
 * - turn on/off the binarization filter
 * - turn on/off the smooth filter
 * - tune the smooth factor of the smooth filter
 * - tune the compensation algorithm parameters (i.e. safety threshold, compensation gain, contact compensation gain)
 * - monitor the skin data frequency
 * - monitor the drift of each skin taxel
 * - get warning and error messages related to the skin
 * - see all the ports read by the interested \ref icub_skinManager module 
 *
 * \section parameters_sec Parameters
 * 
 * \code
 * --name: name of the gui (used to form port names)
 * --from: configuration file
 * --context: directory where to search the configuration file (specified from $ICUB_ROOT/app)
 * --xpos: position of the gui on the screen on the x axis expressed in pixel 
 * --ypos: position of the gui on the screen on the y axis expressed in pixel 
 * \endcode
 * Example:
 * \code
 * skinManagerGui --name skinManGuiLeft --from skinManGuiLeft.ini --context skinGui
 * \endcode
 *
 * \section portsa_sec Ports Accessed
 * None
 *
 * \section portsc_sec Ports Created
 * Three ports are created for communicating with the \ref icub_skinManager module.
 * The port names are:
 * - "/" + guiName + "/rpc:o"
 * - "/" + guiName + "/monitor:i"
 * - "/" + guiName + "/info:i"
 * \n These ports should be externally connected to the corresponding \ref icub_skinManager ports.
 * 
 * \section conf_file_sec Configuration Files
 *
 * None.
 *
 * \section tested_os_sec Tested OS
 * Linux and Windows.
 *
 * \author Andrea Del Prete (andrea.delprete@iit.it)
 *
 *Copyright (C) 2010 RobotCub Consortium
 *
 *CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 *This file can be edited at main/src/tools/skinManagerGui/src/main.cpp.
 **/

#include <string>
#include "iCub/skinManagerGui/guiCallback.h"

bool initGuiStatus() {
    Bottle reply = sendRpcCommand(true, get_binarization);
    if(string(reply.toString().c_str()).compare("on") == 0){
        gtk_toggle_button_set_active(btnBinarization, true);
        gtk_button_set_label(GTK_BUTTON(btnBinarization), "ON");
    }

    reply = sendRpcCommand(true, get_smooth_filter);
    if(string(reply.toString().c_str()).compare("on") == 0){
        gtk_toggle_button_set_active(btnSmooth, true);
        gtk_button_set_label(GTK_BUTTON(btnSmooth), "ON");
        gtk_widget_set_sensitive(GTK_WIDGET(scaleSmooth), true);
    }else{
        gtk_widget_set_sensitive(GTK_WIDGET(scaleSmooth), false);
    }

    reply = sendRpcCommand(true, get_smooth_factor);
    currentSmoothFactor = reply.get(0).asDouble();
    gtk_adjustment_set_value(scaleSmooth->range.adjustment, currentSmoothFactor);   

    reply = sendRpcCommand(true, get_threshold);
    if(reply.isNull() || reply.size()==0 || !reply.get(0).isInt()){
        printLog("Error while getting the safety threshold");
        return false;
    }else{
        currentThreshold = reply.get(0).asInt();
        gtk_adjustment_set_value(spinThreshold->adjustment, currentThreshold);
    }

    reply = sendRpcCommand(true, get_gain);
    if(reply.isNull() || reply.size()==0 || (!reply.get(0).isDouble() && !reply.get(0).isInt())){
        printLog("Error while getting the compensation gain");
        return false;
    }else{
        currentCompGain = reply.get(0).asDouble();
        gtk_adjustment_set_value(spinGain->adjustment, currentCompGain);
    }

    reply = sendRpcCommand(true,get_cont_gain);
    if(reply.isNull() || reply.size()==0 || (!reply.get(0).isDouble() && !reply.get(0).isInt())){
        printLog("Error while getting the contact compensation gain");
        return false;
    }else{
        currentContCompGain = reply.get(0).asDouble();
        gtk_adjustment_set_value(spinContGain->adjustment, currentContCompGain);
    } 

    reply = sendRpcCommand(true,get_max_neigh_dist);
    if(reply.isNull() || reply.size()==0 || (!reply.get(0).isDouble() && !reply.get(0).isInt())){
        printLog("Error while getting the max neighbor distance");
        return false;
    }else{
        currentMaxNeighDist = reply.get(0).asDouble();
        gtk_adjustment_set_value(spinMaxNeighDist->adjustment, 1e2*currentMaxNeighDist);
    }

    // get module information
    reply = sendRpcCommand(true, get_info);
    if(reply.isNull() || reply.size()!=3){
        printLog("Error while reading the module information");
        gtk_label_set_text(lblInfo, reply.toString().c_str());
        return false;
    }
    stringstream ss;
    ss<< reply.get(0).toString().c_str()<< endl;
    ss<< reply.get(1).toString().c_str()<< "\nInput ports:";
    Bottle* portList = reply.get(2).asList();
    portNames.resize(portList->size()/2);
    portDim.resize(portList->size()/2);
    //int numTaxels = 0;
    for(unsigned int i=0;i<portDim.size();i++){
        portNames[i] = portList->get(i*2).toString().c_str();
        portDim[i] = portList->get(i*2+1).asInt();
        //numTaxels += portDim[i];
        ss<< "\n - "<< portNames[i]<< " ("<< portDim[i]<< " taxels)";
    }
    gtk_label_set_text(lblInfo, ss.str().c_str());

    // plot        
    unsigned int numTriangles = portDim[0]/12;    
    GtkTreeIter iter;    
    //gtk_label_set_text(lblMaxX, ss.str().c_str());
    for(unsigned int i=0;i<portDim.size();i++){
        gtk_list_store_append (listPort, &iter);
        gtk_list_store_set (listPort, &iter, 0, i, 1, portNames[i].c_str(), -1);
    }    
    for(unsigned int i=0;i<numTriangles;i++){
        gtk_list_store_append (listTriangle, &iter);
        gtk_list_store_set (listTriangle, &iter, 0, i, -1);
    }
    for(unsigned int i=0;i<12;i++){
        gtk_list_store_append (listTaxel, &iter);
        gtk_list_store_set (listTaxel, &iter, 0, i, -1);
    }
    gtk_combo_box_set_active(comboPort, 0);
    gtk_combo_box_set_active(comboTriangle, 0);
    gtk_combo_box_set_active(comboTaxel, 0);
    stringstream maxXS; maxXS<< currentSampleNum/currentSampleFreq;
    gtk_label_set_text(lblMaxX, maxXS.str().c_str());
    gtk_curve_set_range(curveComp, 0, (gfloat)currentSampleNum, 0, 255);

    // check whether the skin calibration is in process
    reply = sendRpcCommand(true, is_calibrating);
    if(string(reply.toString().c_str()).compare("yes")==0){
        gtk_widget_show(GTK_WIDGET(progBarCalib));
        g_timeout_add(100, progressbar_calibration, NULL);
        gtk_widget_set_sensitive(GTK_WIDGET(btnCalibration), false);
    }

    return true;
}

bool initNetwork(Network& yarp, ResourceFinder &rf, int argc, char *argv[], string &guiName, unsigned int& gXpos, unsigned int& gYpos){    
    rf.setDefaultConfigFile("skinManGui.ini");      //overridden by --from parameter
    rf.setDefaultContext("skinGui");                    //overridden by --context parameter
    rf.configure(argc, argv);

    gXpos=10; 
    gYpos=10;
    if (rf.check("xpos")) gXpos=rf.find("xpos").asInt();
    if (rf.check("ypos")) gYpos=rf.find("ypos").asInt();

    string driftCompRpcPortName     = rf.check("skinManRpcPort", Value("/skinManager/rpc")).asString().c_str();
    string driftCompMonitorPortName = rf.check("skinManMonitorPort", Value("/skinManager/monitor:o")).asString().c_str();
    string skinDiagnosticsErrorPortName = rf.check("skinDiagnosticsErrorsPort", Value("/diagnostics/skin/errors:o")).asString().c_str();

    guiName                         = rf.check("name", Value("skinManGui")).asString().c_str();
    string guiRpcPortName           = "/" + guiName + "/rpc:o";
    string guiMonitorPortName       = "/" + guiName + "/monitor:i";
    string guiInfoPortName          = "/" + guiName + "/info:i";
    string guiDiagnosticsErrorPortName = "/" + guiName + "/diagnostics/skin/errors:i";
    //string wholeBodyRpcPortName       = "/" + guiName + "/wholeBody/rpc";
    if (!guiRpcPort.open(guiRpcPortName.c_str())) {
        string msg = string("Unable to open port ") + guiRpcPortName.c_str();
        openDialog(msg.c_str(), GTK_MESSAGE_ERROR);
        return false;
    }
    if (!driftCompMonitorPort.open(guiMonitorPortName.c_str())){
        string msg = string("Unable to open port ") + guiMonitorPortName.c_str();
        openDialog(msg.c_str(), GTK_MESSAGE_ERROR);
        return false;
    }
    if (!driftCompInfoPort.open(guiInfoPortName.c_str())){
        string msg = string("Unable to open port ") + guiInfoPortName.c_str();
        openDialog(msg.c_str(), GTK_MESSAGE_ERROR);
        return false;
    }

    // Open skin diagnostics port
    if (!portSkinDiagnosticsErrorsIn.open(guiDiagnosticsErrorPortName.c_str())) {
        string msg = string("Unable to open port ") + guiDiagnosticsErrorPortName.c_str();
        openDialog(msg.c_str(), GTK_MESSAGE_ERROR);
        return false;
    }
        
 //   if (!wholeBodyRpcPort.open(wholeBodyRpcPortName.c_str())){
    //  string msg = string("Unable to open port ") + wholeBodyRpcPortName.c_str();
    //  openDialog(msg.c_str(), GTK_MESSAGE_ERROR);
    //  return false;
    //}

    driftCompInfoPort.setStrict();
    
    // automatic connections removed because they gave problems when running the gui 
    // just after the skinDriftCompensation module (using manager.py)
    /*if(!yarp.connect(guiRpcPortName.c_str(), driftCompRpcPortName.c_str())){
        string msg = string("Unable to connect to skinDriftCompensation rpc port: ") 
            + driftCompRpcPortName.c_str() + ". Connect later.";
        openDialog(msg.c_str(), GTK_MESSAGE_WARNING);
    }
    if(!yarp.connect(driftCompMonitorPortName.c_str(), guiMonitorPortName.c_str())){
        string msg = string("Unable to connect to skinDriftCompensation monitor port: ") 
            + driftCompMonitorPortName.c_str() + ". Connect later.";
        openDialog(msg.c_str(), GTK_MESSAGE_WARNING);
    }
    if(!yarp.connect(driftCompInfoPortName.c_str(), guiInfoPortName.c_str())){
        string msg = string("Unable to connect to skinDriftCompensation info port: ") 
            + driftCompInfoPortName.c_str() + ". Connect later.";
        openDialog(msg.c_str(), GTK_MESSAGE_WARNING);
    }*/
    /*if(!yarp.connect(wholeBodyRpcPort.getName().c_str(), "/wholeBodyDynamics/rpc:i"))
        openDialog("Unable to connect to wholeBodyDynamics rpc port. Connect later.", GTK_MESSAGE_WARNING);*/
    return true;
}


int main (int argc, char *argv[])
{       
    GtkBuilder              *builder;       
    GtkButton               *btnTouchThr;
    GtkButton               *btnClearLog;
    GtkSpinButton           *spinSampleFreq;
    GtkSpinButton           *spinSampleNum;
    GError                  *error = NULL;

    Network yarp;
    ResourceFinder rf;
    string guiName;
    unsigned int gXpos, gYpos;

#if !GLIB_CHECK_VERSION(2, 32, 0)
    // since Glib 2.32 g_thread_init is deprecated
    g_thread_init (NULL);
#endif
    gdk_threads_init ();    
    gdk_threads_enter ();
    gtk_init (&argc, &argv);    // initialize gtk
    builder = gtk_builder_new ();
        
    if(!initNetwork(yarp, rf, argc, argv, guiName, gXpos, gYpos))
        return 0;

    rf.setDefault("gladeFile", "skinManGui.glade");
    string gladeFile = rf.findFile("gladeFile");
    
    if( !gtk_builder_add_from_file (builder, gladeFile.c_str(), &error)){
        g_warning( "%s", error->message );
        clean_exit();
        return 0;
    }

    // get pointers to the widgets we are interested in
    window          = GTK_WINDOW (gtk_builder_get_object (builder, "window"));

    btnSmooth       = GTK_TOGGLE_BUTTON (gtk_builder_get_object (builder, "btnSmooth"));
    btnBinarization = GTK_TOGGLE_BUTTON (gtk_builder_get_object (builder, "btnBinarization"));
    btnClearLog     = GTK_BUTTON (gtk_builder_get_object (builder, "btnClearLog"));
    scaleSmooth     = GTK_SCALE (gtk_builder_get_object (builder, "scaleSmooth"));  
    btnCalibration  = GTK_BUTTON (gtk_builder_get_object (builder, "btnCalibration"));
    progBarCalib    = GTK_PROGRESS_BAR (gtk_builder_get_object (builder, "progressbarCalib"));
    btnTouchThr     = GTK_BUTTON (gtk_builder_get_object (builder, "btnThreshold"));
    spinThreshold   = GTK_SPIN_BUTTON (gtk_builder_get_object (builder, "spinbuttonThreshold"));
    spinGain        = GTK_SPIN_BUTTON (gtk_builder_get_object (builder, "spinbuttonGain"));
    spinContGain    = GTK_SPIN_BUTTON (gtk_builder_get_object (builder, "spinbuttonContGain"));
    spinMaxNeighDist= GTK_SPIN_BUTTON (gtk_builder_get_object (builder, "spinbuttonMaxNeighDist"));

    tbLog           = GTK_TEXT_BUFFER (gtk_builder_get_object (builder, "textbufferLog"));
    tvLog           = GTK_TEXT_VIEW (gtk_builder_get_object (builder, "textviewLog"));
    lblInfo         = GTK_LABEL (gtk_builder_get_object (builder, "labelInfo"));
    treeBaselines   = GTK_TREE_VIEW (gtk_builder_get_object (builder, "treeview"));
    treeStoreComp   = GTK_TREE_STORE (gtk_builder_get_object (builder, "treestore"));

    curveComp       = GTK_CURVE (gtk_builder_get_object (builder, "curve"));
    lblMaxY         = GTK_LABEL (gtk_builder_get_object (builder, "lblMaxY"));
    lblMinY         = GTK_LABEL (gtk_builder_get_object (builder, "lblMinY"));
    lblMaxX         = GTK_LABEL (gtk_builder_get_object (builder, "lblMaxX"));
    lblMinX         = GTK_LABEL (gtk_builder_get_object (builder, "lblMinX"));
    comboPort       = GTK_COMBO_BOX (gtk_builder_get_object (builder, "comboboxPort"));
    comboTriangle   = GTK_COMBO_BOX (gtk_builder_get_object (builder, "comboboxTriangle"));
    comboTaxel      = GTK_COMBO_BOX (gtk_builder_get_object (builder, "comboboxTaxel"));
    listPort        = GTK_LIST_STORE(gtk_builder_get_object (builder, "liststorePort"));
    listTriangle    = GTK_LIST_STORE(gtk_builder_get_object (builder, "liststoreTriangle"));
    listTaxel       = GTK_LIST_STORE(gtk_builder_get_object (builder, "liststoreTaxel"));
    spinSampleFreq  = GTK_SPIN_BUTTON (gtk_builder_get_object (builder, "spinbuttonSampleFreq"));
    spinSampleNum   = GTK_SPIN_BUTTON (gtk_builder_get_object (builder, "spinbuttonSampleNum"));

    statusBar       = GTK_STATUSBAR (gtk_builder_get_object (builder, "statusBar"));
    statusBarFreq   = GTK_STATUSBAR (gtk_builder_get_object (builder, "statusbarFreq"));

    // if the rpc port is connected, then initialize the gui status
    initDone = false;
    if(guiRpcPort.getOutputCount()>0)       
        initDone = initGuiStatus();

    if(initDone)
        printLog("GUI connected!");
    else
        printLog("GUI not connected. Connect it to the module to make it work.");
    // otherwise the gui will try to initialize every timeout (i.e. 1 sec)

    currentSampleFreq = 5;
    currentSampleNum = 100;
    dataPlot.resize(currentSampleNum);
    gtk_adjustment_set_value(spinSampleFreq->adjustment, currentSampleFreq);
    gtk_adjustment_set_value(spinSampleNum->adjustment, currentSampleNum);

    // connect all the callback functions (after the initialization, so as not to activate the callbacks)
    g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy_event), NULL);
    g_signal_connect(btnSmooth, "button-press-event", G_CALLBACK(toggle_button_smooth), scaleSmooth);
    g_signal_connect(btnBinarization, "button-press-event", G_CALLBACK(toggle_button_binarization), NULL);
    g_signal_connect(btnCalibration, "button-press-event", G_CALLBACK(button_calibration), NULL);
    g_signal_connect(btnTouchThr, "button-press-event", G_CALLBACK(button_threshold), NULL);
    g_signal_connect(btnClearLog, "button-press-event", G_CALLBACK(button_clear_log), NULL);
    g_signal_connect(scaleSmooth, "change-value", G_CALLBACK(scale_smooth_value_changed), NULL);
    g_signal_connect(spinThreshold, "value-changed", G_CALLBACK(spin_threshold_value_changed), NULL);
    g_signal_connect(spinGain, "value-changed", G_CALLBACK(spin_gain_value_changed), NULL);
    g_signal_connect(spinContGain, "value-changed", G_CALLBACK(spin_cont_gain_value_changed), NULL);
    g_signal_connect(spinMaxNeighDist, "value-changed", G_CALLBACK(spin_max_neigh_dist_value_changed), NULL);
    g_signal_connect(comboPort , "changed", G_CALLBACK(comboPort_changed), NULL);
    g_signal_connect(comboTriangle , "changed", G_CALLBACK(comboTriangle_changed), NULL);
    g_signal_connect(comboTaxel , "changed", G_CALLBACK(comboTaxel_changed), NULL);
    g_signal_connect(spinSampleFreq, "value-changed", G_CALLBACK(spinSampleFreq_value_changed), NULL);
    g_signal_connect(spinSampleNum, "value-changed", G_CALLBACK(spinSampleNum_value_changed), NULL);
    timeoutId = gdk_threads_add_timeout(1000/currentSampleFreq, (periodic_timeout), NULL);   // thread safe version of "g_timeout_add()

    // free the memory used by the glade xml file
    g_object_unref (G_OBJECT (builder));

    gtk_widget_show(GTK_WIDGET(window));
    gtk_window_set_title(window, guiName.c_str());
    gtk_window_set_resizable(window, true);
    gtk_window_set_default_size(GTK_WINDOW(window),600,200);
    gtk_window_resize(GTK_WINDOW(window),600,200);
    gtk_window_move(GTK_WINDOW(window),gXpos,gYpos);

    
    //printf("Main thread: %p\n", g_thread_self());
    gtk_main ();
    gdk_threads_leave();

    return 0;
}
