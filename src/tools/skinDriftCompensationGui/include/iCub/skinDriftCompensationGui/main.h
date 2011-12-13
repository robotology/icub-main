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

#include <string>
#include <sstream> 
#include <iomanip>					// io manipulator (setw, setfill)
#include <cstdarg>
//#include <cstdlib>
#include <math.h>
#include <vector>

#include <gtk/gtk.h>
#include <glib.h>

#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Semaphore.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

// main window
GtkWindow               *window;
GtkStatusbar			*statusBar;
GtkStatusbar			*statusBarFreq;
// first tab
GtkProgressBar			*progBarCalib;
GtkButton				*btnCalibration;
GtkToggleButton			*btnSmooth;
GtkToggleButton			*btnBinarization;
GtkScale				*scaleSmooth;	
GtkTextView             *tvLog;
GtkTextBuffer           *tbLog;
GtkSpinButton           *spinThreshold;
GtkSpinButton           *spinGain;
GtkSpinButton           *spinContGain;
// second tab
GtkTreeView             *treeBaselines;
GtkTreeStore            *treeStoreComp;
// third tab
GtkCurve                *curveComp;
GtkLabel                *lblMaxY;
GtkLabel                *lblMinY;
GtkLabel                *lblMaxX;
GtkLabel                *lblMinX;
GtkComboBox             *comboPort;
GtkComboBox             *comboTriangle;
GtkComboBox             *comboTaxel;
GtkListStore            *listPort;
GtkListStore            *listTriangle;
GtkListStore            *listTaxel;
// fourth tab
GtkLabel                *lblInfo;

// ports for communicating with the module
Port					guiRpcPort;             // to send rpc command to the module
BufferedPort<Vector>	driftCompMonitorPort;   // for reading streaming data (frequency, drift)
BufferedPort<Bottle>	driftCompInfoPort;      // for reading sporadic msgs (errors, warnings)

// global data
guint                   timeoutId;              // id of the timeout callback function
vector<string>			portNames;				// names of the skin input ports
vector<unsigned int>	portDim;				// number of taxels of the input ports
double					currentSmoothFactor;    // current smooth factor value
bool                    initDone;               // true if the gui has been initialized
unsigned int            currentThreshold;       // current safety threshold
double                  currentCompGain;        // current compensation gain
double                  currentContCompGain;    // current contact compensation gain
int                     currentSampleFreq;
int                     currentSampleNum;       // size of the dataPlot array

// plot data
Semaphore               plotSem;
gint                    port2plot;              // index of the skin port to plot
gint                    tr2plot;                // triangle to plot
gint                    tax2plot;               // taxel to plot
vector<gfloat>          dataPlot;               // data to plot

bool initGuiStatus();

static void printLog(string text){
    text = text + "\n";
    GtkTextIter tbIter;    
    gtk_text_buffer_get_end_iter(tbLog, &tbIter);
    gtk_text_buffer_insert(tbLog, &tbIter, text.c_str(), text.length());
    gtk_text_view_scroll_to_iter(tvLog, &tbIter, 0.0, false, 0.0, 0.0);
}

static double round(double value, int decimalDigit){
	double q = pow(10.0, decimalDigit);
	return double(int((value*q)+0.5))/q;
}

static void openDialog(const char* msg, GtkMessageType type){
	GtkWidget* dialog = gtk_message_dialog_new (window,
                                 GTK_DIALOG_DESTROY_WITH_PARENT,
                                 type,
                                 GTK_BUTTONS_CLOSE,
                                 "%s", msg);
	gtk_dialog_run (GTK_DIALOG (dialog));
	gtk_widget_destroy (dialog);
}

static void setStatusBarText(string text){
	guint contextId = gtk_statusbar_get_context_id(statusBar, text.c_str());
	gtk_statusbar_push(statusBar, contextId, text.c_str());
}

static void setStatusBarFreq(bool freqUpdated, double freq){
	stringstream text;
	if(freqUpdated){
		freq = round(freq, 2);
		text<< "SkinDriftCompensation frequency: "<< freq;
	}else{
		text<< "Cannot read the frequency. Probably the skinDriftCompensation module has stopped working.";
	}
	guint contextId = gtk_statusbar_get_context_id(statusBarFreq, text.str().c_str());
	gtk_statusbar_push(statusBarFreq, contextId, text.str().c_str());
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


static void resetPlotData(){
    for(int i=0; i<currentSampleNum; i++)
        dataPlot[i] = 0;
}