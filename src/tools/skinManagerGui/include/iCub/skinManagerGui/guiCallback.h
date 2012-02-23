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
#include "iCub/skinManagerGui/main.h"
#include "iCub/skinDynLib/rpcSkinManager.h"
#include <stdlib.h> //atoi


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
    b.addInt(set_threshold); b.addInt(safetyThr);
    guiRpcPort.write(b, setReply);

    // read the threshold
	Bottle getReply = sendRpcCommand(true, get_threshold);
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
    b.addInt(set_gain); b.addDouble(compGain);
    guiRpcPort.write(b, setReply);

    // read the gain
	Bottle getReply = sendRpcCommand(true, get_gain);
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

static void spin_cont_gain_value_changed(GtkSpinButton *spinbutton, gpointer user_data){
    double contCompGain = gtk_spin_button_get_value(spinbutton);
    if(contCompGain == currentContCompGain)
        return;

    // set the gain
    Bottle b, setReply;
    b.addInt(set_cont_gain); b.addDouble(contCompGain);
    guiRpcPort.write(b, setReply);

    // read the gain
	Bottle getReply = sendRpcCommand(true, get_cont_gain);
	currentContCompGain = getReply.get(0).asDouble();

	if(contCompGain==currentContCompGain){
        stringstream msg; msg << "Contact compensation gain changed: " << contCompGain;
        setStatusBarText(msg.str().c_str());
        return;
    }
    
    stringstream msg; msg << "Unable to set the contact compensation gain to " << contCompGain;
	msg<< ".\nSet command reply: "<< setReply.toString().c_str();
	openDialog(msg.str().c_str(), GTK_MESSAGE_ERROR);   
    // setting the old value
    gtk_spin_button_set_value(spinbutton, currentContCompGain);
}

static gboolean scale_smooth_value_changed(GtkRange* range, GtkScrollType scroll, gdouble value, gpointer user_data){
	// check whether the smooth factor has changed
	double smoothFactor = round(value, 1); //double(int((value*10)+0.5))/10.0;
	if(smoothFactor==currentSmoothFactor)
		return false;

	// set the smooth factor
	Bottle b, setReply;
	b.addInt(set_smooth_factor); b.addDouble(smoothFactor);
	guiRpcPort.write(b, setReply);
	
	// read the smooth factor
	Bottle getReply = sendRpcCommand(true, get_smooth_factor);
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
        Bottle b, setReply;
	    b.addInt(set_smooth_filter); b.addString("on");
	    guiRpcPort.write(b, setReply);
		Bottle reply = sendRpcCommand(true, get_smooth_filter);
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
		Bottle b, setReply;
	    b.addInt(set_smooth_filter); b.addString("off");
	    guiRpcPort.write(b, setReply);
		Bottle reply = sendRpcCommand(true, get_smooth_filter);
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
        Bottle b, setReply;
	    b.addInt(set_binarization); b.addString("on");
	    guiRpcPort.write(b, setReply);
		Bottle reply = sendRpcCommand(true, get_binarization);
		if(string(reply.toString().c_str()).compare("on")==0){
			gtk_button_set_label(GTK_BUTTON(widget), "ON");
			setStatusBarText("Binarization filter turned on");
			return false;	// propagate the event further
		}else{
			setStatusBarText(string("Error! Unable to turn the binarization filter on: ") + reply.toString().c_str());
			return true;	//stop other handlers from being invoked for the event
		}
    } else {
		Bottle b, setReply;
	    b.addInt(set_binarization); b.addString("off");
	    guiRpcPort.write(b, setReply);
		Bottle reply = sendRpcCommand(true, get_binarization);
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
	Bottle reply = sendRpcCommand(true, is_calibrating);
	if(string(reply.toString().c_str()).compare("yes")==0)
		return true;

	gtk_widget_hide(GTK_WIDGET(progBarCalib));
	gtk_widget_set_sensitive(GTK_WIDGET(btnCalibration), true);
	setStatusBarText("Calibration done");
    printLog("Calibration done!");
	return false;
}
static gboolean button_calibration (GtkToggleButton *widget, GdkEvent *ev, gpointer data){	
	sendRpcCommand(false, calibrate);
	gtk_widget_show(GTK_WIDGET(progBarCalib));
	g_timeout_add(100, progressbar_calibration, NULL);
	gtk_widget_set_sensitive(GTK_WIDGET(widget), false);
	return false;
}


static gboolean button_threshold(GtkToggleButton *widget, GdkEvent *ev, gpointer data){
	//printf("Button callback thread: %p\n", g_thread_self());
	Bottle touchThr = sendRpcCommand(true, get_touch_thr);
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


static void comboPort_changed(GtkComboBox* combo, gpointer data){
    gchararray portName;
    gint newPort2plot=-1;
    GtkTreeIter iter;
    GtkTreeModel* model;
    if( gtk_combo_box_get_active_iter(comboPort, &iter)){                    
        model = gtk_combo_box_get_model(comboPort);         // Obtain data model from combo box.                    
        gtk_tree_model_get( model, &iter, 0, &newPort2plot, 1, &portName, -1 ); // Obtain string from model.
    }
    if(newPort2plot!=port2plot){    // port has changed
        port2plot = newPort2plot;
        resetPlotData();
        unsigned int numTriangles = portDim[port2plot]/12;
        gtk_list_store_clear(listTriangle);
        for(unsigned int i=0;i<numTriangles;i++){
            gtk_list_store_append (listTriangle, &iter);
            gtk_list_store_set (listTriangle, &iter, 0, i, -1);
        }
        gtk_combo_box_set_active(comboTriangle, 0);
    }
}
static void comboTriangle_changed(GtkComboBox* combo, gpointer data){
    gint newTr2plot=-1;
    GtkTreeIter iter;
    GtkTreeModel* model;
    if( gtk_combo_box_get_active_iter(comboTriangle, &iter)){                    
        model = gtk_combo_box_get_model(comboTriangle);     // Obtain data model from combo box.                    
        gtk_tree_model_get( model, &iter, 0, &newTr2plot, -1 ); 
    }
    if(newTr2plot!=tr2plot){    // triangle has changed
        tr2plot = newTr2plot;
        resetPlotData();
    }
}
static void comboTaxel_changed(GtkComboBox* combo, gpointer data){
    gint newTax2plot=-1;
    GtkTreeIter iter;
    GtkTreeModel* model;
    if( gtk_combo_box_get_active_iter(comboTaxel, &iter)){                    
        model = gtk_combo_box_get_model(comboTaxel);     // Obtain data model from combo box.                    
        gtk_tree_model_get( model, &iter, 0, &newTax2plot, -1 ); 
    }
    if(newTax2plot!=tax2plot){    // taxel has changed
        tax2plot = newTax2plot;
        resetPlotData();
    }
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
		Vector* b = driftCompMonitorPort.read(false);
        if(!b || b->size()==0){
            setStatusBarFreq(false, 0);
        }else{
            //set the frequency
			double freq = (*b)[0];
			setStatusBarFreq(true, freq);

            // set the drift
            int numTax = 0;
            for(unsigned int i=0; i<portDim.size(); i++) numTax += portDim[i];
            const int numTr = numTax/12;
            if(numTax>0){
                // *** UPDATE TAXEL PLOT (3RD TAB)                
                plotSem.wait();
                for(int i=0; i<currentSampleNum-1; i++)
                    dataPlot[i] = dataPlot[i+1];
                int index = 1+numTax+ tr2plot*12 + tax2plot;
                for(int i=0;i<port2plot;i++)
                    index += portDim[i];
                dataPlot[currentSampleNum-1] = (gfloat)(*b)[index];
                gtk_curve_set_vector(curveComp, currentSampleNum, &(dataPlot[0]));
                plotSem.post();
                               

                // *** UPDATE DRIFT TREE VIEW (2ND TAB)
                vector<gfloat> driftPerTr(numTr);
                gdouble sumTr;
                for(int i=0; i<numTr; i++){
					sumTr=0;
					for(int j=0; j<12; j++){
						sumTr += (gfloat) (*b)(i*12+j+1);
					}
					driftPerTr[i] = (gfloat)round(sumTr/12.0, 2);
                }

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
				index=1;
                int portIndex=0;
                gdouble meanTr;
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
							gdouble drift = round((*b)(index), 2);
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

static void spinSampleFreq_value_changed(GtkSpinButton *spinbutton, gpointer user_data){
    int freq = gtk_spin_button_get_value_as_int(spinbutton);
    if(freq==currentSampleFreq)
        return;
    currentSampleFreq = freq;
    guint period = (int)(1000.0/freq);
    // set the freq
    g_source_remove(timeoutId);
    timeoutId = gdk_threads_add_timeout(period, (periodic_timeout), NULL);   // thread safe version of "g_timeout_add()
    stringstream maxXS; maxXS<< currentSampleNum/currentSampleFreq;
    gtk_label_set_text(lblMaxX, maxXS.str().c_str());
}
static void spinSampleNum_value_changed(GtkSpinButton *spinbutton, gpointer user_data){
    int sampleNum = gtk_spin_button_get_value_as_int(spinbutton);
    if(sampleNum==currentSampleNum)
        return;
    
    plotSem.wait();
    vector<gfloat> newDataPlot(sampleNum);
    unsigned int data2copy = min(sampleNum, currentSampleNum);                
    for(unsigned int i=1;i<=data2copy;i++){
        newDataPlot[sampleNum-i] = dataPlot[currentSampleNum-i];
    }
    dataPlot = newDataPlot;
    currentSampleNum = sampleNum;
    
    // update plot label and limits
    stringstream maxXS; maxXS<< currentSampleNum/currentSampleFreq;
    gtk_label_set_text(lblMaxX, maxXS.str().c_str());
    gtk_curve_set_range(curveComp, 0, (gfloat)currentSampleNum, 0, 255);
    plotSem.post();
}