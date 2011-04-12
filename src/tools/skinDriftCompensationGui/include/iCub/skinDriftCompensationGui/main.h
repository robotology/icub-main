#include <string>
#include <sstream>  
#include <cstdarg>
#include <cstdlib>

#include <gtk/gtk.h>

#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

using namespace std;
using namespace yarp::os;

GtkWindow               *window;
GtkStatusbar			*statusBar;
GtkStatusbar			*statusBarFreq;
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

static void setStatusBarFreq(double freq){
	stringstream text; text<< "Data frequency: "<< freq;
	guint contextId = gtk_statusbar_get_context_id(statusBar, text.str().c_str());
	gtk_statusbar_push(statusBar, contextId, text.str().c_str());
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

