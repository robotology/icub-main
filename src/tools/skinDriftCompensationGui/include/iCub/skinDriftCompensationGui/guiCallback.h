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


static void clean_exit(){
	guiRpcPort.interrupt();
	driftCompMonitorPort.interrupt();
    driftCompInfoPort.interrupt();

	guiRpcPort.close();
	driftCompMonitorPort.close();    
    driftCompInfoPort.close();
}

static void on_window_destroy_event(GtkObject *object, gpointer user_data){
	clean_exit();
	gtk_main_quit();
}

static void spin_threshold_value_changed(GtkSpinButton *spinbutton, gpointer user_data){
    int safetyThr = gtk_spin_button_get_value_as_int(spinbutton);
    if(safetyThr == currentThreshold)
        return;
    
    // set the threshold
    Bottle b, setReply;
    b.addString("set"); b.addString("threshold"); b.addInt(safetyThr);
    guiRpcPort.write(b, setReply);

    // read the threshold
	Bottle getReply = sendRpcCommand(true, 2, "get", "threshold");
	currentThreshold = getReply.get(0).asInt();

	if(safetyThr==currentThreshold){
        stringstream msg; msg << "Safety threshold changed: " << safetyThr;
        setStatusBarText(msg.str().c_str());
        return;
    }
    
    stringstream msg; msg << "Unable to set the threshold to " << safetyThr;
	msg<< ".\nSet command reply: "<< setReply.toString().c_str();
	openDialog(msg.str().c_str(), GTK_MESSAGE_ERROR);
    // setting the old value
    gtk_spin_button_set_value(spinbutton, currentThreshold);
}

static void spin_gain_value_changed(GtkSpinButton *spinbutton, gpointer user_data){
    double compGain = gtk_spin_button_get_value(spinbutton);
    if(compGain == currentCompGain)
        return;

    // set the gain
    Bottle b, setReply;
    b.addString("set"); b.addString("gain"); b.addDouble(compGain);
    guiRpcPort.write(b, setReply);

    // read the gain
	Bottle getReply = sendRpcCommand(true, 2, "get", "gain");
	currentCompGain = getReply.get(0).asDouble();

	if(compGain==currentCompGain){
        stringstream msg; msg << "Compensation gain changed: " << compGain;
        setStatusBarText(msg.str().c_str());
        return;
    }
    
    stringstream msg; msg << "Unable to set the compensation gain to " << compGain;
	msg<< ".\nSet command reply: "<< setReply.toString().c_str();
	openDialog(msg.str().c_str(), GTK_MESSAGE_ERROR);   
    // setting the old value
    gtk_spin_button_set_value(spinbutton, currentCompGain);
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
	}
	stringstream msg; msg << "Unable to set the smooth factor to " << smoothFactor;
	msg<< ".\nSet command reply: "<< setReply.toString().c_str();
	openDialog(msg.str().c_str(), GTK_MESSAGE_ERROR);
    // setting the old value
    gtk_range_set_value(range, currentSmoothFactor);
	return true;
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
    printLog("Calibration done!");
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
	int index = 0;
    for(unsigned int j=0; j<portDim.size(); j++){
	    stringstream msg;
		msg<< "Thresholds for port "<< portNames[j]<< ":\n";
	    for(unsigned int i=0; i<portDim[j]; i++){            
		    if(i%12==0){
			    if(i!=0)
				    msg<< "\n";
			    msg<< "TR"<< i/12<< ":\t";
		    }
		    msg << int(touchThr.get(index).asDouble())<< ";\t";
			index++;
	    }
	    openDialog(msg.str().c_str(), GTK_MESSAGE_INFO);
    }
	return false;
}

static gboolean button_clear_log(GtkToggleButton *widget, GdkEvent *ev, gpointer data){
	gtk_text_buffer_set_text(tbLog, "", 0);
	return false;
}

static gint periodic_timeout(gpointer data){    
	//printf("Timeout thread: %p\n", g_thread_self());

    // if the gui has not been initialized yet
    if(!initDone){
        // if the rpc port is connected, then initialize the gui status
        if(guiRpcPort.getOutputCount()>0)
            initDone = initGuiStatus();

        if(initDone)
            printLog("GUI connected!");
	    else
            return true;
    }

	if(driftCompMonitorPort.getInputCount()==0){
        setStatusBarFreq(false, 0);
    }else{
		Bottle* b = driftCompMonitorPort.read(false);
        if(!b){
            setStatusBarFreq(false, 0);
        }else{
            //set the frequency
			double freq = b->get(0).asDouble();
			setStatusBarFreq(true, freq);

            // set the drift
            const int numTax = b->size()-1;
            const int numTr = numTax/12;
            if(numTax>0){
                // draw the drift plot
                vector<gfloat> driftPerTr(numTax/12);
                gdouble maxY=0, minY=0, sumTr, meanTr;
                for(int i=0; i<numTr; i++){
					sumTr=0;
					for(int j=0; j<12; j++){
						sumTr += (gfloat) b->get(i*12+j+1).asDouble();
					}
					meanTr = sumTr/12.0;
					driftPerTr[i] = (gfloat)round(meanTr, 2);
                    if(meanTr>maxY)
                        maxY = meanTr;
                    else if(meanTr<minY)
                        minY = meanTr;
                }
                /*gtk_curve_set_range(curveComp, 0, (gfloat)(numTr), (gfloat)(minY-1), (gfloat)(maxY+1));
                gtk_curve_set_vector(curveComp, numTr, &driftPerTr[0]);                
                stringstream maxYS; maxYS<< maxY;
                gtk_label_set_text(lblMaxY, maxYS.str().c_str());
                stringstream minYS; minYS<< minY;
                gtk_label_set_text(lblMinY, minYS.str().c_str());*/
                               

				GtkTreeIter iterPort, iterTr, iterTax;
				gboolean valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(treeStoreComp), &iterPort);
				if(!valid){
					// create the drift list
					for(unsigned int i=0; i<portNames.size(); i++){
						gtk_tree_store_append(treeStoreComp, &iterPort, NULL);
						for(unsigned int j=0; j<portDim[i]/12; j++){
							gtk_tree_store_append(treeStoreComp, &iterTr, &iterPort);
							for(int k=0; k<12; k++){
								gtk_tree_store_append(treeStoreComp, &iterTax, &iterTr);
							}
						}
					}
					gtk_tree_view_set_model(treeBaselines, GTK_TREE_MODEL( treeStoreComp));
					gtk_tree_model_get_iter_first(GTK_TREE_MODEL(treeStoreComp), &iterPort);
				}
				
                // update the drift list
                gdouble sumPort, meanPort;
				stringstream trS, taxS;
				int index=1, portIndex=0;				
				for(unsigned int i=0; i<portNames.size(); i++){		
					sumPort = 0;
					for(unsigned int j=0; j<portDim[i]/12; j++){						
						meanTr = driftPerTr[portIndex];
						portIndex++;
						sumPort += meanTr;
						trS.str(""); trS<<j;
						gtk_tree_model_iter_nth_child(GTK_TREE_MODEL(treeStoreComp), &iterTr, &iterPort, j);
						gtk_tree_store_set((treeStoreComp), &iterTr, 1, trS.str().c_str(), 3, meanTr, -1);
						for(int k=0; k<12; k++){
							gdouble drift = round(b->get(index).asDouble(), 2);
							index++;
							taxS.str(""); taxS<<k;
							if(gtk_tree_model_iter_nth_child(GTK_TREE_MODEL(treeStoreComp), &iterTax, &iterTr, k))
								gtk_tree_store_set((treeStoreComp), &iterTax, 2, taxS.str().c_str(), 3, drift, -1);
						}
					}

					meanPort = sumPort/(portDim[i]/12);
					gtk_tree_store_set((treeStoreComp), &iterPort, 0, portNames[i].c_str(), 3, meanPort, -1);
					gtk_tree_model_iter_next(GTK_TREE_MODEL(treeStoreComp), &iterPort);
				}
            }
		}
    }

    // check if there are messages on the info port
    Bottle* infoMsg = driftCompInfoPort.read(false);
    while(infoMsg){
        //openDialog(infoMsg->toString(), GTK_MESSAGE_INFO);
        printLog(infoMsg->toString().c_str());
        infoMsg = driftCompInfoPort.read(false);
    }

	return true;
}