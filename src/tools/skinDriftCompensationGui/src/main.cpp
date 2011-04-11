#include <string>
#include <sstream>  
#include <cstdarg>

#include <gtk/gtk.h>

#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

using namespace std;
using namespace yarp::os;

GtkWindow               *window;
GtkStatusbar			*statusBar;
GtkProgressBar			*progBarCalib;
GtkButton				*btnCalibration;
Port					guiRpcPort;
double					currentSmoothFactor;

static void openDialog(const char* msg, GtkMessageType type){
	GtkWidget* dialog = gtk_message_dialog_new (window,
                                 GTK_DIALOG_DESTROY_WITH_PARENT,
                                 type,
                                 GTK_BUTTONS_CLOSE,
                                 msg);
	gtk_dialog_run (GTK_DIALOG (dialog));
	gtk_widget_destroy (dialog);
}

static void setStatusBarText(string text){
	guint contextId = gtk_statusbar_get_context_id(statusBar, text.c_str());
	gtk_statusbar_push(statusBar, contextId, text.c_str());
}

static Bottle sendRpcCommand(bool responseExpected, int commandWordCount, const char* command, ...){
	Bottle resp;
	// check whether the port is connected
	if(guiRpcPort.getOutputCount()==0){
		openDialog((string("Connection to the rpc port of the skinDriftCompensation")
			+ "module not available. Connect and try again.").c_str(), GTK_MESSAGE_ERROR);
		return resp;
	}

	// create the bottle
	Bottle b;
	b.addString(command);

	va_list ap;
	va_start(ap, command);		// Requires the last fixed parameter (to get the address)		
	for(int i=1;i<commandWordCount;i++){
		const char* c = va_arg(ap, const char*); // Requires the type to cast to. Increments ap to the next argument.
		b.addString(c);
	}
    va_end(ap);
	
	
	//g_print("Going to send rpc msg: %s\n", b.toString().c_str());
	if(responseExpected){		
		guiRpcPort.write(b, resp);		
	}else{
		guiRpcPort.write(b);
	}
	return resp;
}

static void on_window_destroy_event(GtkObject *object, gpointer user_data){
	guiRpcPort.interrupt();
	guiRpcPort.close();
	gtk_main_quit();
}

gboolean scale_smooth_value_changed(GtkRange* range, GtkScrollType scroll, gdouble value, gpointer user_data){
	// check whether the smooth factor has changed
	double smoothFactor = double(int((value*10)+0.5))/10.0;
	if(smoothFactor==currentSmoothFactor)
		return false;

	// set the smooth factor
	Bottle b, setReply;
	b.addString("set"); b.addString("smooth"); b.addString("factor"); b.addDouble(smoothFactor);
	guiRpcPort.write(b, setReply);
	
	// read the smooth factor
	Bottle getReply = sendRpcCommand(true, 3, "get", "smooth", "factor");
	currentSmoothFactor = getReply.get(0).asDouble();
	currentSmoothFactor = double(int((currentSmoothFactor*10)+0.5))/10.0;

	if(smoothFactor==currentSmoothFactor){
		stringstream msg; msg << "Smooth factor changed: " << smoothFactor;
		setStatusBarText(msg.str());
		return false;
	}else{
		stringstream msg; msg << "Unable to set the smooth factor to " << smoothFactor;
		msg<< ".\nSet command reply: "<< setReply.toString().c_str();
		openDialog(msg.str().c_str(), GTK_MESSAGE_ERROR);
		return true;
	}
}


static gboolean toggle_button_smooth(GtkToggleButton *widget, GdkEvent *ev, gpointer data){
	gboolean btnState = gtk_toggle_button_get_active(widget);

	// if the button is on it means it is going to be turned on
    if (!btnState){
		sendRpcCommand(false, 4, "set", "smooth", "filter", "on");
		Bottle reply = sendRpcCommand(true, 3, "get", "smooth", "filter");
		if(string(reply.toString().c_str()).compare("on") == 0){
			gtk_button_set_label(GTK_BUTTON(widget), "ON");
			gtk_widget_set_sensitive(GTK_WIDGET(data), true);
			setStatusBarText("Smooth filter turned on");
			return false;	// propagate the event further
		}else{			
			setStatusBarText(string("Error! Unable to turn the smooth filter on: ") + reply.toString().c_str());
			return true;	//stop other handlers from being invoked for the event
		}
    } else {
		sendRpcCommand(false, 4, "set", "smooth", "filter", "off");
		Bottle reply = sendRpcCommand(true, 3, "get", "smooth", "filter");
		if(string(reply.toString().c_str()).compare("off") == 0){
			gtk_button_set_label(GTK_BUTTON(widget), "OFF");
			gtk_widget_set_sensitive(GTK_WIDGET(data), false);
			setStatusBarText("Smooth filter turned off");
			return false;	// propagate the event further
		}else{			
			setStatusBarText(string("Error! Unable to turn the smooth filter off: ") + reply.toString().c_str());
			return true;	//stop other handlers from being invoked for the event
		}
    }
}

static gboolean toggle_button_binarization (GtkToggleButton *widget, GdkEvent *ev, gpointer data){
	gboolean btnState = gtk_toggle_button_get_active(widget);

    if (!btnState){
		sendRpcCommand(false, 3, "set", "binarization", "on");
		Bottle reply = sendRpcCommand(true, 2, "get", "binarization");
		if(string(reply.toString().c_str()).compare("on")==0){
			gtk_button_set_label(GTK_BUTTON(widget), "ON");
			setStatusBarText("Binarization filter turned on");
			return false;	// propagate the event further
		}else{
			setStatusBarText(string("Error! Unable to turn the binarization filter on: ") + reply.toString().c_str());
			return true;	//stop other handlers from being invoked for the event
		}
    } else {
		sendRpcCommand(false, 3, "set", "binarization", "off");
		Bottle reply = sendRpcCommand(true, 2, "get", "binarization");
		if(string(reply.toString().c_str()).compare("off")==0){
			gtk_button_set_label(GTK_BUTTON(widget), "OFF");
			setStatusBarText("Binarization filter turned off");
			return false;	// propagate the event further
		}else{
			setStatusBarText(string("Error! Unable to turn the binarization filter off: ") + reply.toString().c_str());
			return true;	//stop other handlers from being invoked for the event
		}
    }
}



static gint progressbar_calibration(gpointer data){
	gtk_progress_bar_pulse(progBarCalib);

	// check whether the calibration is still in progress
	Bottle reply = sendRpcCommand(true, 2, "is", "calibrating");
	if(string(reply.toString().c_str()).compare("yes")==0)
		return true;

	gtk_widget_hide(GTK_WIDGET(progBarCalib));
	gtk_widget_set_sensitive(GTK_WIDGET(btnCalibration), true);
	setStatusBarText("Calibration done");
	return false;
}
static gboolean button_calibration (GtkToggleButton *widget, GdkEvent *ev, gpointer data){	
	sendRpcCommand(false, 2, "force", "calibration");
	gtk_widget_show(GTK_WIDGET(progBarCalib));
	g_timeout_add(100, progressbar_calibration, NULL);
	gtk_widget_set_sensitive(GTK_WIDGET(widget), false);
	return false;
}


static gboolean button_threshold(GtkToggleButton *widget, GdkEvent *ev, gpointer data){
	Bottle touchThr = sendRpcCommand(true, 2, "get", "percentile");
	stringstream msg;
	for(int i=0; i< touchThr.size(); i++){
		if(i%12==0){
			if(i!=0)
				msg<< "\n";
			msg<< "TR"<< i/12<< ":\t";
		}
		msg << int(touchThr.get(i).asDouble())<< ";\t";
	}
	openDialog(msg.str().c_str(), GTK_MESSAGE_INFO);
	return false;
}
int main (int argc, char *argv[])
{	
	GtkBuilder              *builder;	
	GtkToggleButton			*btnSmooth;
	GtkToggleButton			*btnBinarization;
	GtkScale				*scaleSmooth;	
	GtkButton				*btnTouchThr;
	
	GError					*error = NULL;

	gtk_init (&argc, &argv);	// initialize gtk
	builder = gtk_builder_new ();
	string icubRoot = getenv("ICUB_ROOT");
	if( !gtk_builder_add_from_file (builder, (icubRoot+
		"/main/src/tools/skinDriftCompensationGui/skinDriftCompGui.glade").c_str(), &error)){
		g_warning( "%s", error->message );
        return 0;
	}

	// get pointers to the widgets we are interested in
	window			= GTK_WINDOW (gtk_builder_get_object (builder, "window"));
	btnSmooth		= GTK_TOGGLE_BUTTON (gtk_builder_get_object (builder, "btnSmooth"));
	btnBinarization = GTK_TOGGLE_BUTTON (gtk_builder_get_object (builder, "btnBinarization"));
	scaleSmooth		= GTK_SCALE (gtk_builder_get_object (builder, "scaleSmooth"));
	statusBar		= GTK_STATUSBAR (gtk_builder_get_object (builder, "statusBar"));
	btnCalibration	= GTK_BUTTON (gtk_builder_get_object (builder, "btnCalibration"));
	progBarCalib	= GTK_PROGRESS_BAR (gtk_builder_get_object (builder, "progressbarCalib"));
	btnTouchThr		= GTK_BUTTON (gtk_builder_get_object (builder, "btnThreshold"));
	


	Network yarp;
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("skinDriftCompensationGui.ini");		//overridden by --from parameter
	rf.setDefaultContext("skinGui/conf");							//overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);
		
	string driftCompRpcPortName	= rf.check("driftCompRpcPort", Value("/skinDriftComp/rpc")).asString().c_str();
	string guiName				= rf.check("name", Value("skinDriftCompGui")).asString().c_str();
	string guiRpcPortName		= "/" + guiName + "/rpc:o";
	if (!guiRpcPort.open(guiRpcPortName.c_str())) {
		string msg = string("Unable to open port ") + guiRpcPortName.c_str();
		openDialog(msg.c_str(), GTK_MESSAGE_ERROR);
		return 0;
	}
	
	if(!yarp.connect(guiRpcPortName.c_str(), driftCompRpcPortName.c_str())){
		string msg = string("Unable to connect to skinDriftCompensation rpc port: ") 
			+ driftCompRpcPortName.c_str() + ". Connect later.";
		openDialog(msg.c_str(), GTK_MESSAGE_WARNING);
	}

	// if the rpc port is connected, then initialize the gui status
	if(guiRpcPort.getOutputCount()>0){
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
	}



	// connect all the callback functions (after the initialization, so as not to activate the callbacks)
	g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy_event), NULL);
	g_signal_connect(btnSmooth, "button-press-event", G_CALLBACK(toggle_button_smooth), scaleSmooth);
	g_signal_connect(btnBinarization, "button-press-event", G_CALLBACK(toggle_button_binarization), NULL);
	g_signal_connect(btnCalibration, "button-press-event", G_CALLBACK(button_calibration), NULL);
	g_signal_connect(btnTouchThr, "button-press-event", G_CALLBACK(button_threshold), NULL);
	g_signal_connect(scaleSmooth, "change-value", G_CALLBACK(scale_smooth_value_changed), NULL);

	// free the memory used by the glade xml file
	g_object_unref (G_OBJECT (builder));
	gtk_widget_show(GTK_WIDGET(window));
	gtk_window_set_title(window, guiName.c_str());
	gtk_window_set_resizable(window, true);
	gtk_window_set_default_size(GTK_WINDOW(window),300,200);
    gtk_window_resize(GTK_WINDOW(window),300,200);
	gtk_window_move(GTK_WINDOW(window),10,10);

	

	gtk_main ();

	return 0;
}
