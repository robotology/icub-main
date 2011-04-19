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
#include "iCub/skinDriftCompensationGui/main.h"

static void on_window_destroy_event(GtkObject *object, gpointer user_data){
	guiRpcPort.interrupt();
	driftCompMonitorPort.interrupt();
    driftCompInfoPort.interrupt();

	guiRpcPort.close();
	driftCompMonitorPort.close();    
    driftCompInfoPort.close();
	gtk_main_quit();
}

static gboolean scale_smooth_value_changed(GtkRange* range, GtkScrollType scroll, gdouble value, gpointer user_data){
	// check whether the smooth factor has changed
	double smoothFactor = round(value, 1); //double(int((value*10)+0.5))/10.0;
	if(smoothFactor==currentSmoothFactor)
		return false;

	// set the smooth factor
	Bottle b, setReply;
	b.addString("set"); b.addString("smooth"); b.addString("factor"); b.addDouble(smoothFactor);
	guiRpcPort.write(b, setReply);
	
	// read the smooth factor
	Bottle getReply = sendRpcCommand(true, 3, "get", "smooth", "factor");
	currentSmoothFactor = getReply.get(0).asDouble();
	currentSmoothFactor = round(currentSmoothFactor, 1); //double(int((currentSmoothFactor*10)+0.5))/10.0;

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

	// if the button is off it means it is going to be turned on
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
	//printf("Button callback thread: %p\n", g_thread_self());
	Bottle touchThr = sendRpcCommand(true, 2, "get", "percentile");
    for(int j=0; j<=(touchThr.size()-1)/(16*12); j++){
	    stringstream msg;
        //msg<< "touchThr size: "<< touchThr.size()<< "\n";
	    for(int i=j*16*12; i< min(touchThr.size(), 16*12*(j+1)); i++){            
		    if(i%12==0){
			    if(i!=0)
				    msg<< "\n";
			    msg<< "TR"<< i/12<< ":\t";
		    }
		    msg << int(touchThr.get(i).asDouble())<< ";\t";
	    }
	    openDialog(msg.str().c_str(), GTK_MESSAGE_INFO);
    }
	return false;
}



static gint periodic_timeout(gpointer data){    
	//printf("Timeout thread: %p\n", g_thread_self());
	if(driftCompMonitorPort.getInputCount()>0){
		Bottle* b = driftCompMonitorPort.read(false);
		if(b){
            //set the frequency
			double freq = b->get(0).asDouble();	
			setStatusBarFreq(true, freq);            

            // set the drift
            const int numTax = b->size()-1;
            if(numTax>0){
                // draw the drift plot
                vector<gfloat> compensations(numTax);
                gfloat maxY=0, minY=0;
                for(int i=1; i<=numTax; i++){
                    compensations[i-1] = (gfloat) b->get(i).asDouble();
                    if(compensations[i-1]>maxY)
                        maxY = compensations[i-1];
                    else if(compensations[i-1]<minY)
                        minY = compensations[i-1];
                }
                gtk_curve_set_range(curveComp, 0, (gfloat)numTax, minY, maxY);
                gtk_curve_set_vector(curveComp, numTax, &compensations[0]);                
                stringstream maxYS; maxYS<< maxY;
                gtk_label_set_text(lblMaxY, maxYS.str().c_str());
                stringstream minYS; minYS<< minY;
                gtk_label_set_text(lblMinY, minYS.str().c_str());

                // update the drift list
                gtk_list_store_clear(listStoreComp);
                GtkTreeIter iter;
                double sum, mean;
                for(int i=0; i<numTax/12; i++){                    
                    sum=0;
                    for(int j=0; j<12; j++){
                        sum += b->get(i+1).asDouble();
                    }
                    mean = round(sum/12.0, 1);

                    gtk_list_store_append(listStoreComp, &iter);
                    stringstream ssTr; ssTr<<i;
                    stringstream ssDrift; ssDrift<<mean;
                    gtk_list_store_set(listStoreComp, &iter, 0, ssTr.str().c_str(), 1, ssDrift.str().c_str(), -1);                    
                }
                gtk_tree_view_set_model(treeBaselines, GTK_TREE_MODEL( listStoreComp));
            }
		}else
			setStatusBarFreq(false, 0);
	}else
		setStatusBarFreq(false, 0);

    // check if there is a message on the info port
    Bottle* infoMsg = driftCompInfoPort.read(false);
    if(infoMsg){
        openDialog(infoMsg->toString(), GTK_MESSAGE_INFO);
    }

	return true;
}