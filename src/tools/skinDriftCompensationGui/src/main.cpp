/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
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
#include "iCub/skinDriftCompensationGui/guiCallback.h"

void initGuiStatus(GtkToggleButton* btnSmooth, GtkToggleButton* btnBinarization, GtkScale* scaleSmooth){
    Bottle reply = sendRpcCommand(true, 2, "get", "binarization");
	if(string(reply.toString().c_str()).compare("on") == 0){
		gtk_toggle_button_set_active(btnBinarization, true);
		gtk_button_set_label(GTK_BUTTON(btnBinarization), "ON");
	}

	reply = sendRpcCommand(true, 3, "get", "smooth", "filter");
	if(string(reply.toString().c_str()).compare("on") == 0){
		gtk_toggle_button_set_active(btnSmooth, true);
		gtk_button_set_label(GTK_BUTTON(btnSmooth), "ON");
		gtk_widget_set_sensitive(GTK_WIDGET(scaleSmooth), true);
	}else{
		gtk_widget_set_sensitive(GTK_WIDGET(scaleSmooth), false);
	}

	reply = sendRpcCommand(true, 3, "get", "smooth", "factor");
	currentSmoothFactor = reply.get(0).asDouble();
	gtk_adjustment_set_value(scaleSmooth->range.adjustment, currentSmoothFactor);

	// check whether the skin calibration is in process
	reply = sendRpcCommand(true, 2, "is", "calibrating");
	if(string(reply.toString().c_str()).compare("yes")==0){
		gtk_widget_show(GTK_WIDGET(progBarCalib));
		g_timeout_add(100, progressbar_calibration, NULL);
		gtk_widget_set_sensitive(GTK_WIDGET(btnCalibration), false);
	}

    // get module information
    reply = sendRpcCommand(true, 2, "get", "info");
    string s = reply.get(0).toString().c_str();
    for(int i=1;i<reply.size();i++){
        s += "\n";
        s += reply.get(i).toString().c_str();
    }
    gtk_label_set_text(lblInfo, s.c_str());
}

bool initNetwork(Network& yarp, ResourceFinder &rf, int argc, char *argv[], string &guiName, unsigned int& gXpos, unsigned int& gYpos){    
    rf.setVerbose(true);
	rf.setDefaultConfigFile("skinDriftCompensationGui.ini");		//overridden by --from parameter
	rf.setDefaultContext("skinGui/conf");							//overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    gXpos=10; 
    gYpos=10;
	if (rf.check("xpos")) gXpos=rf.find("xpos").asInt();
    if (rf.check("ypos")) gYpos=rf.find("ypos").asInt();

	string driftCompRpcPortName		= rf.check("driftCompRpcPort", Value("/skinDriftComp/rpc")).asString().c_str();
    string driftCompMonitorPortName	= rf.check("driftCompMonitorPort", Value("/skinDriftComp/monitor:o")).asString().c_str();
    string driftCompInfoPortName	= rf.check("driftCompInfoPort", Value("/skinDriftComp/info:o")).asString().c_str();    
	guiName					        = rf.check("name", Value("skinDriftCompGui")).asString().c_str();
	string guiRpcPortName			= "/" + guiName + "/rpc:o";
	string guiMonitorPortName		= "/" + guiName + "/monitor:i";
    string guiInfoPortName		    = "/" + guiName + "/info:i";
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
    driftCompInfoPort.setStrict();
	
	if(!yarp.connect(guiRpcPortName.c_str(), driftCompRpcPortName.c_str())){
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
	}
    return true;
}


int main (int argc, char *argv[])
{		
	GtkBuilder              *builder;	
	GtkToggleButton			*btnSmooth;
	GtkToggleButton			*btnBinarization;
	GtkScale				*scaleSmooth;	
	GtkButton				*btnTouchThr;
	GError					*error = NULL;

    Network yarp;
    ResourceFinder rf;
    string guiName;
    unsigned int gXpos, gYpos;

	g_thread_init (NULL);
    gdk_threads_init ();	
    gdk_threads_enter ();
	gtk_init (&argc, &argv);	// initialize gtk
	builder = gtk_builder_new ();
		
	if(!initNetwork(yarp, rf, argc, argv, guiName, gXpos, gYpos))
        return 0;
	rf.setDefault("gladeFile", "skinDriftCompGui.glade");
	ConstString gladeFile = rf.findFile("gladeFile");
	
	if( !gtk_builder_add_from_file (builder, gladeFile.c_str(), &error)){
		g_warning( "%s", error->message );
        return 0;
	}

	// get pointers to the widgets we are interested in
	window			= GTK_WINDOW (gtk_builder_get_object (builder, "window"));
	btnSmooth		= GTK_TOGGLE_BUTTON (gtk_builder_get_object (builder, "btnSmooth"));
	btnBinarization = GTK_TOGGLE_BUTTON (gtk_builder_get_object (builder, "btnBinarization"));
	scaleSmooth		= GTK_SCALE (gtk_builder_get_object (builder, "scaleSmooth"));
	statusBar		= GTK_STATUSBAR (gtk_builder_get_object (builder, "statusBar"));
	statusBarFreq	= GTK_STATUSBAR (gtk_builder_get_object (builder, "statusbarFreq"));
	btnCalibration	= GTK_BUTTON (gtk_builder_get_object (builder, "btnCalibration"));
	progBarCalib	= GTK_PROGRESS_BAR (gtk_builder_get_object (builder, "progressbarCalib"));
	btnTouchThr		= GTK_BUTTON (gtk_builder_get_object (builder, "btnThreshold"));
    lblInfo         = GTK_LABEL (gtk_builder_get_object (builder, "labelInfo"));
    treeBaselines   = GTK_TREE_VIEW (gtk_builder_get_object (builder, "treeview"));
    listStoreComp   = GTK_LIST_STORE (gtk_builder_get_object (builder, "liststore"));
    curveComp       = GTK_CURVE (gtk_builder_get_object (builder, "curve"));
    lblMaxY         = GTK_LABEL (gtk_builder_get_object (builder, "lblMaxY"));
    lblMinY         = GTK_LABEL (gtk_builder_get_object (builder, "lblMinY"));

	// if the rpc port is connected, then initialize the gui status
	if(guiRpcPort.getOutputCount()>0){
		initGuiStatus(btnSmooth, btnBinarization, scaleSmooth);
	}

	// connect all the callback functions (after the initialization, so as not to activate the callbacks)
	g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy_event), NULL);
	g_signal_connect(btnSmooth, "button-press-event", G_CALLBACK(toggle_button_smooth), scaleSmooth);
	g_signal_connect(btnBinarization, "button-press-event", G_CALLBACK(toggle_button_binarization), NULL);
	g_signal_connect(btnCalibration, "button-press-event", G_CALLBACK(button_calibration), NULL);
	g_signal_connect(btnTouchThr, "button-press-event", G_CALLBACK(button_threshold), NULL);
	g_signal_connect(scaleSmooth, "change-value", G_CALLBACK(scale_smooth_value_changed), NULL);
	gdk_threads_add_timeout(2000, (periodic_timeout), NULL);   // thread safe version of "g_timeout_add()

	// free the memory used by the glade xml file
	g_object_unref (G_OBJECT (builder));

	gtk_widget_show(GTK_WIDGET(window));
	gtk_window_set_title(window, guiName.c_str());
	gtk_window_set_resizable(window, true);
	gtk_window_set_default_size(GTK_WINDOW(window),300,200);
    gtk_window_resize(GTK_WINDOW(window),300,200);
	gtk_window_move(GTK_WINDOW(window),gXpos,gYpos);

	
	//printf("Main thread: %p\n", g_thread_self());
	gtk_main ();
	gdk_threads_leave();

	return 0;
}
