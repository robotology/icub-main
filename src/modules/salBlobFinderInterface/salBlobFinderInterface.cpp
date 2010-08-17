// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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

#include "salBlobFinderInterface.h"

#include <yarp/os/Property.h> 
#include <yarp/os/Network.h> 
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>

#include <string>
#include <sstream>
#include <cstring>

using namespace yarp::os;
using namespace std;

GtkWidget *mainWindow=0;
static GdkPixbuf *frame = NULL;
std::string* message; //reference to the string refering to the last command to send

BufferedPort<yarp::sig::FlexImage> *ptr_inputPort=0;
std::string* command;   // reference to the string refering to the last command to send
int _frameN;            // Frame Number
bool _savingSet;        //Save Set of Images mode
yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort=0;      //Output Point Port
yarp::os::Bottle _outBottle;                                //Output Bottle Container

pgmOptions _options;

GtkWidget *saveSingleDialog;
GtkWidget *saveSetDialog;
GtkWidget *menubar;
GtkWidget *fileMenu, *imageMenu, *helpMenu;
GtkWidget *fileItem, *imageItem, *helpItem;
GtkWidget *fileSingleItem, *fileSetItem, *fileQuitItem;
GtkWidget *imageSizeItem, *imageRatioItem, *imageFreezeItem, *imageFramerateItem;
GtkWidget *synchroDisplayItem;
GtkWidget *helpAboutItem;
// StatusBar
GtkWidget *statusbar;
GtkWidget *fpsStatusBar;
GtkWidget *fpsStatusBar2;

guint timeout_ID=0;
guint timeout_update_ID=0;

ViewerResources _resources;
double timeCentroid;               //time costant for the centroid ouput
double targetRed;                  //colour red for the target object
double targetBlue;                 //colour green for the target object
double targetGreen;                //colour blue for the target object
double salienceTD;                 //saliency top-down coefficient
double salienceBU;                  //saliency bottom-up coefficient
double reactivity;                 //time costant
double minBoundingArea;            //min dimension of the blob
int minBLOB;                    //min dimension of the blob
int maxBLOB;                    //max dimension of the blob
double targetRED;                  //colour RER for the target
double targetGREEN;                //colour GREEN for the target
double targetBLUE;                  //colour BLUE for the target

bool watershed_flag;            //flag watershed
bool tagged_flag;               //flag for the tag
bool meanColour_flag;           //flag minColour
bool maxSaliencyBlob_flag;      //flag for maxSaliencyBlob
bool blobList_flag;             //flag for the blobList
bool colorVQ_flag;              //flag for the colorVQ
bool contrastLP_flag;           //flag contrastLP (saliency map)
bool foveaBlob_flag;            //flag for foveaBlob

static void createObjects() {
    ptr_inputPort = new BufferedPort<yarp::sig::FlexImage>;
    message=new string("");
    //ptr_portCallback = new InputCallback;
}

static void deleteObjects() {
    /*if (ptr_inputPort!=0)
        delete ptr_inputPort;
    if (ptr_portCallback!=0)
        delete ptr_portCallback;*/
}




//-------------------------------------------------
// Main Window Callbacks
//-------------------------------------------------
/**
* usual callback function 
*/
static void callback( GtkWidget *widget,gpointer   data ){
    printf ("Hello again - %s was pressed \n", (char *) data);
    
    if(!strcmp((char *)data,"OpencvSobel")){
        printf("OpencvSobel");
        command->assign("set ocv");
    }
    if(!strcmp((char *)data,"ConvolveMax")){
        printf("ConvolveMax");
        command->assign("set max");
    }
    if(!strcmp((char *)data,"ConvolveSeq")){
        printf("ConvolveSeq");
        command->assign("set seq");
    }
    if(!strcmp((char *)data,"ConvolveFil")){
        printf("ConvolvFil");
        command->assign("set fil");
    }
    if(!strcmp((char *)data,"IppSobel")){
        printf("IppSobel");
        command->assign("set ipp");
    }

}



//-------------------------------------------------
// Call Backs
//-------------------------------------------------
static void cb_digits_scale( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    salienceBU=adj->value/100;
    //printf("salienceBU: %f",salienceBU);
    std::string str("");
    sprintf((char *)str.c_str(),"set kbu %2.2f",salienceBU);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scale2( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    salienceTD=adj->value/100;
    //printf("salienceTD: %f",salienceTD);
    std::string str("");
    sprintf((char *)str.c_str(),"set ktd %2.2f",salienceTD);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scale3( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    maxBLOB=adj->value;
    //printf("maxBLOB: %f",maxBLOB);
    std::string str("");
    sprintf((char *)str.c_str(),"set Mdb %d",maxBLOB);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scale4( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    minBLOB=adj->value;
    //printf("minBLOB: %d",minBLOB);
    std::string str("");
    sprintf((char *)str.c_str(),"set mdb %d",minBLOB);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaler( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    targetRED=adj->value;
    //printf("targetRED: %f",targetRED);
    std::string str("");
    sprintf((char *)str.c_str(),"set rin %2.2f",targetRED);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaleg( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    targetGREEN=adj->value;
    //printf("targetGREEN: %f",targetGREEN);
    std::string str("");
    sprintf((char *)str.c_str(),"set gin %2.2f",targetGREEN);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaleb( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    targetBLUE=adj->value;
    //printf("targetBLUE: %f",targetBLUE);
    std::string str("");
    sprintf((char *)str.c_str(),"set bin %2.2f",targetBLUE);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scalemin( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    minBoundingArea=adj->value;
    //printf("minBoundingArea: %f",minBoundingArea);
    std::string str("");
    sprintf((char *)str.c_str(),"set mBA %2.2f",minBoundingArea);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaletime( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    reactivity=adj->value;
    //printf("constant time for the iKinControlGaze: %f",reactivity/10);
    std::string str("");
    sprintf((char *)str.c_str(),"set tco %2.2f",reactivity/10);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaletime2( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    timeCentroid=adj->value;
    //printf("constant time for the controlGaze2: %f",timeCentroid/10);
    std::string str("");
    sprintf((char *)str.c_str(),"set tce %2.2f",timeCentroid/10);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}
gint timeout_update_CB(gpointer data) {
    //portFpsData.getStats(av, min, max);
    //portFpsData.reset();
    gchar *msg;
    gdk_threads_enter();

    msg=g_strdup_printf("selectiveAttentionInterface");
    updateStatusbar(fpsStatusBar, msg);
    g_free(msg);

    //displayFpsData.getStats(av, min, max);
    //displayFpsData.reset();
    
    //periodToFreq(av, min, max, avHz, minHz, maxHz);

    //msg=g_strdup_printf("Display: %.1f (min:%.1f max:%.1f) fps", avHz, minHz, maxHz);
    //updateStatusbar(fpsStatusBar2, msg);
    //g_free(msg);

    gdk_threads_leave();

    return TRUE;
}

gint timeout_CB (gpointer data) {
    gdk_threads_enter();
    gdk_threads_leave();
    return TRUE;
}


gboolean delete_event( GtkWidget *widget, GdkEvent *event, gpointer data ) {
    // If you return FALSE in the "delete_event" signal handler,
    // GTK will emit the "destroy" signal. Returning TRUE means
    // you don't want the window to be destroyed.
    // This is useful for popping up 'are you sure you want to quit?'
    // type dialogs. 
    cleanExit();
    return TRUE;
}

gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    return TRUE;
}

static gboolean configure_event( GtkWidget *widget, GdkEventConfigure *event ) {
   _resources.configure(widget, 
        widget->allocation.width,
        widget->allocation.height);
  return TRUE;
}
gint menuFileQuit_CB(GtkWidget *widget, gpointer data) {
    cleanExit();
    return TRUE;
}

gint menuHelpAbout_CB(GtkWidget *widget, gpointer data) {
#if GTK_CHECK_VERSION(2,6,0)
    const gchar *authors[] = 
        {
            "Yarp developers",
            NULL
        };
    const gchar *license =
        "Released under the terms of the LGPLv2.1 or later, see LGPL.TXT\n"
        "The complete license description is contained in the\n"
        "COPYING file included in this distribution.\n"
        "Please refer to this file for complete\n"
        "information about the licensing of YARP.\n"
        "\n"
        "DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE\n"
        "SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS\n"
        "DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS\n"
        "EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE\n"
        "SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT\n"
        "OWNERS AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED\n"
        "INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n"
        "FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO\n"
        "EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE\n"
        "LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN\n"
        "ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN\n"
        "ONNECTION WITH THE SOFTWARE.\n";

    gtk_show_about_dialog(GTK_WINDOW(mainWindow),
                          "name", "salBlobFinderInterface",
                          "version", "1.0",
                          "license", license,
                          "website", "http://sourceforge.net/projects/yarp0",
                          "comments", "Interface for selectiveAttentionEngine",
                          "authors", authors,
                          NULL);
#else
    printf("Missing functionality on older GTK version, sorry\n");
#endif

    return TRUE;
}


gint menuImageSize_CB(GtkWidget *widget, gpointer data) {
    int targetWidth, targetHeight;
    targetWidth = _resources.imageWidth();
    targetHeight = _resources.imageHeight();
    if (targetWidth!=0&&targetHeight!=0) {
        unsigned int windowH=mainWindow->allocation.height;
        unsigned int windowW=mainWindow->allocation.width;
        //unsigned int daH=_resources.drawArea->allocation.height;
        //unsigned int daW=_resources.drawArea->allocation.width;

        //trick: we compute the new size of the window by difference
        int daH=200;
        int daW=200;
        unsigned int newHeight=(windowH-daH)+targetHeight;
        unsigned int newWidth=(windowW-daW)+targetWidth;
        gtk_window_resize(GTK_WINDOW(mainWindow), newWidth, newHeight);
    }
    return TRUE;
}

gint menuImageRatio_CB(GtkWidget *widget, gpointer data) {
    double ratio;
    int imgWidth, imgHeight;
    int targetWidth, targetHeight;
    imgWidth = _resources.imageWidth();
    imgHeight = _resources.imageHeight();
    if (imgWidth!=0&&imgHeight!=0) {
        unsigned int windowH=mainWindow->allocation.height;
        unsigned int windowW=mainWindow->allocation.width;
        int daH=200;
        int daW=200;
        //unsigned int daW = _resources.drawArea->allocation.width;
        //unsigned int daH = _resources.drawArea->allocation.height;
        ratio = double(imgWidth) / double(imgHeight);
        targetWidth = int(double(daH) * ratio);
        targetHeight = daH;
 
        //trick: we compute the new size of the window by difference
        unsigned int newHeight=(windowH-daH)+targetHeight;
        unsigned int newWidth=(windowW-daW)+targetWidth;
        gtk_window_resize(GTK_WINDOW(mainWindow), newWidth, newHeight);    }
    return TRUE;
}

gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSetItem), FALSE);
            gtk_widget_show_all (saveSingleDialog);
        } 
    else 
        {
            gtk_widget_hide (saveSingleDialog);
        }

    return TRUE;
}

gint menuFileSet_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data) {
#if GTK_CHECK_VERSION(2,6,0)

    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSingleItem), FALSE);
                    
            gtk_widget_show_all (saveSetDialog);
        } 
    else 
        {
            gtk_widget_hide (saveSetDialog);
        }

#endif

    return TRUE;
}

gint menuSynchroDisplay_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            setSynchroMode(); 
            _options.synch=true;
        } 
    else 
        {
            setTimedMode(_options.refreshTime);
            _options.synch=false;
        }
    return TRUE;
}


gint menuImageFreeze_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            _resources.freeze();    
        } 
    else 
        {
            _resources.unfreeze();
        }
    return TRUE;
}

gint saveSingleDelete_CB (GtkWidget *widget, gpointer data) {
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSingleItem), FALSE);

    return (TRUE);
}

gint saveSetDelete_CB (GtkWidget *widget, gpointer data) {
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSetItem), FALSE);

    return (TRUE);
}

gint saveSingleClicked_CB(GtkWidget *widget, gpointer data) {
    return (TRUE);
}

gint saveSetStartClicked_CB(GtkWidget *widget, gpointer data) {
    _savingSet = true;
        
    return (TRUE);
}

gint saveSetStopClicked_CB(GtkWidget *widget, gpointer data) {
    _savingSet = false;
        
    return (TRUE);
}

gint menuImageFramerate_CB(GtkWidget *widget, gpointer data) {
    GtkWidget *dialog;
    GtkWidget *hbox;
    GtkWidget *stock;

    GtkWidget *label;
    GtkWidget *spinner;
    GtkAdjustment *spinner_adj;
    gint response;

    dialog = gtk_dialog_new_with_buttons ("New Refresh Time",
                                          GTK_WINDOW (mainWindow),
                                          GTK_DIALOG_MODAL,
                                          GTK_STOCK_OK,
                                          GTK_RESPONSE_OK,
                                          GTK_STOCK_CANCEL,
                                          GTK_RESPONSE_CANCEL,
                                          NULL);

    hbox = gtk_hbox_new (FALSE, 8);
    gtk_container_set_border_width (GTK_CONTAINER (hbox), 8);
    stock = gtk_image_new_from_stock (GTK_STOCK_DIALOG_QUESTION, GTK_ICON_SIZE_DIALOG);
    gtk_box_pack_start (GTK_BOX (hbox), stock, FALSE, FALSE, 0);
    label = gtk_label_new_with_mnemonic ("Insert new refresh time (in mSec):");
    gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, FALSE, 0);
    spinner_adj = (GtkAdjustment *) gtk_adjustment_new (_options.refreshTime, 10.0, 1000.0, 1.0, 5.0, 5.0);
    spinner = gtk_spin_button_new (spinner_adj, 1.0, 0);
    gtk_box_pack_start (GTK_BOX (hbox), spinner, FALSE, FALSE, 0);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 0);
    
    gtk_widget_show_all (hbox);
    
    response = gtk_dialog_run (GTK_DIALOG (dialog));

    if (response == GTK_RESPONSE_OK)
        {
            _options.refreshTime = (int) gtk_spin_button_get_value (GTK_SPIN_BUTTON(spinner));
            
            if (!_options.synch)
                setTimedMode(_options.refreshTime);

            gchar *msg;
            msg = g_strdup_printf ("%s",_options.portName);
            updateStatusbar(statusbar,msg);
            g_free(msg);
        }

    gtk_widget_destroy (dialog);

    return (TRUE);
}

gint clickDA_CB (GtkWidget *widget, GdkEventButton *event, gpointer data) {
    int imageX, imageY;
    int clickX, clickY;
    int daWidth, daHeight;
    int imageWidth, imageHeight;
    double ratioX, ratioY;

    imageWidth = _resources.imageWidth();
    imageHeight = _resources.imageHeight();

    if ( (imageWidth != 0) && (imageHeight != 0) )
        {
            daWidth = widget->allocation.width;
            daHeight = widget->allocation.height;
            clickX = (int) event->x;
            clickY = (int) event->y;
            ratioX = double(clickX) / double(daWidth);
            ratioY = double(clickY) / double(daHeight);
            imageX = int(imageWidth * ratioX + 0.5);
            imageY = int(imageHeight * ratioY + 0.5);

            printf("Transmitting click information...\n");
            if (_pOutPort!=NULL) {
                yarp::os::Bottle& bot = _pOutPort->prepare();
                bot.clear();
                bot.addInt(imageX);
                bot.addInt(imageY);
                //_pOutPort->Content() = _outBottle;
                _pOutPort->write();
            }
    
        } else {
            printf("I would send a position, but there's no image for scaling\n");
        }

    return TRUE;
}


void setTimedMode(guint dT) {
   // ptr_portCallback->mustDraw(false);
    if (timeout_ID!=0)
        gtk_timeout_remove(timeout_ID);

    timeout_ID = gtk_timeout_add (dT, timeout_CB, NULL);
}

void setSynchroMode() {
    gtk_timeout_remove(timeout_ID);
    timeout_ID=0;
    //ptr_portCallback->mustDraw(true);
}

//-------------------------------------------------
// Non Modal Dialogs
//-------------------------------------------------
GtkWidget* createSaveSingleDialog(void) {

    GtkWidget *dialog = NULL;
    GtkWidget *button;
    GtkWidget *hbox;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Snapshot");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    //gtk_window_resize(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
    hbox = gtk_hbox_new (TRUE, 8); // parameters (gboolean homogeneous_space, gint spacing);
    button = gtk_button_new_from_stock(GTK_STOCK_SAVE);
    gtk_widget_set_size_request (GTK_WIDGET(button), 150,50);
    gtk_box_pack_start (GTK_BOX (hbox), button, TRUE, TRUE, 16); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_signal_connect (GTK_OBJECT (button), "clicked", GTK_SIGNAL_FUNC (saveSingleClicked_CB), NULL);
    gtk_signal_connect (GTK_OBJECT (dialog), "delete_event", GTK_SIGNAL_FUNC (saveSingleDelete_CB), NULL);
    
    //gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
    
    return dialog;
}

GtkWidget* createSaveSetDialog(void) {
    GtkWidget *dialog = NULL;
    GtkWidget *saveButton;
    GtkWidget *stopButton;
    GtkWidget *hbox;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Image Set");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 190, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
#if GTK_CHECK_VERSION(2,6,0)
    saveButton = gtk_button_new_from_stock(GTK_STOCK_MEDIA_RECORD);
    stopButton = gtk_button_new_from_stock(GTK_STOCK_MEDIA_STOP);
#else
    printf("Missing functionality on older GTK version, sorry\n");
#endif
    gtk_widget_set_size_request (GTK_WIDGET(saveButton), 80,50);
    gtk_widget_set_size_request (GTK_WIDGET(stopButton), 80,50);

    hbox = gtk_hbox_new (TRUE, 8); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_box_pack_start (GTK_BOX (hbox), saveButton, TRUE, TRUE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (hbox), stopButton, TRUE, TRUE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_signal_connect (GTK_OBJECT (saveButton), "clicked", GTK_SIGNAL_FUNC (saveSetStartClicked_CB), NULL);
    gtk_signal_connect (GTK_OBJECT (stopButton), "clicked", GTK_SIGNAL_FUNC (saveSetStopClicked_CB), NULL);
    gtk_signal_connect (GTK_OBJECT (dialog), "delete_event", GTK_SIGNAL_FUNC (saveSetDelete_CB), NULL);

    return dialog;
}

static void cb_draw_value( GtkToggleButton *button )
{
    /* Turn the value display on the scale widgets off or on depending
     *  on the state of the checkbutton */
    printf("callbacks from draw value %s \n",button->button.label_text);
    if(!strcmp(button->button.label_text,"ContrastLP-->")){
        if(button->active){
            contrastLP_flag=true;
            message->assign("set clp");
        }
        else
            contrastLP_flag=false;
    }
    
    else if(!strcmp(button->button.label_text,"MeanColoursLP-->")){
        if(button->active){
            meanColour_flag=true;
            message->assign("set mea");
        }
        else
            meanColour_flag=false;
            
    }
    
    else if(!strcmp(button->button.label_text,"MaxSaliencyBlob-->")){
        if(button->active){
            maxSaliencyBlob_flag=true;
            message->assign("set max");
        }
        else
            maxSaliencyBlob_flag=false;
            
    }
    
    else if(!strcmp(button->button.label_text,"FoveaBlob-->")){
        if(button->active){
            foveaBlob_flag=true;
            message->assign("set fov");
         }
        else
            foveaBlob_flag=false;
            
    }
    
    else if(!strcmp(button->button.label_text,"ColorVQ-->")){
        if(button->active)
            colorVQ_flag=true;
        else
            colorVQ_flag=false;
            
    }
    
    else if(!strcmp(button->button.label_text,"BlobList-->")){
        if(button->active)
            blobList_flag=true;
        else
            blobList_flag=false;
            
    }
    
    else if(!strcmp(button->button.label_text,"Tagged-->")){
        if(button->active){
            tagged_flag=true;
            message->assign("set tag");
        }
        else
            tagged_flag=false;
            
    }
    
    else if(!strcmp(button->button.label_text,"Watershed-->")){
        if(button->active){
            watershed_flag=true;
            message->assign("set wat");
        }
        else
            watershed_flag=false;
            
    }
    
}

//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* createMenubar(void) {
    GtkWidget *menubar;

    menubar =  gtk_menu_bar_new ();
    GtkWidget *menuSeparator;	
    // Submenus Items on menubar
    fileItem = gtk_menu_item_new_with_label ("File");
    imageItem = gtk_menu_item_new_with_label ("Image");
    helpItem = gtk_menu_item_new_with_label ("Help");
    // Submenu: File 
    fileMenu = gtk_menu_new();
    fileSingleItem = gtk_check_menu_item_new_with_label ("Save single image..");
    gtk_menu_append( GTK_MENU(fileMenu), fileSingleItem);
    gtk_signal_connect( GTK_OBJECT(fileSingleItem), "toggled", GTK_SIGNAL_FUNC(menuFileSingle_CB), mainWindow);
    fileSetItem = gtk_check_menu_item_new_with_label ("Save a set of images..");
    gtk_menu_append( GTK_MENU(fileMenu), fileSetItem);
    gtk_signal_connect( GTK_OBJECT(fileSetItem), "toggled", GTK_SIGNAL_FUNC(menuFileSet_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(fileMenu), menuSeparator);
    fileQuitItem = gtk_menu_item_new_with_label ("Quit");
    gtk_menu_append( GTK_MENU(fileMenu), fileQuitItem);
    gtk_signal_connect( GTK_OBJECT(fileQuitItem), "activate", GTK_SIGNAL_FUNC(menuFileQuit_CB), mainWindow);
    // Submenu: Image  
    imageMenu = gtk_menu_new();
    imageSizeItem = gtk_menu_item_new_with_label ("Original size");
    gtk_menu_append( GTK_MENU(imageMenu), imageSizeItem);
    gtk_signal_connect( GTK_OBJECT(imageSizeItem), "activate", GTK_SIGNAL_FUNC(menuImageSize_CB), mainWindow);
    imageRatioItem = gtk_menu_item_new_with_label ("Original aspect ratio");
    gtk_menu_append( GTK_MENU(imageMenu), imageRatioItem);
    gtk_signal_connect( GTK_OBJECT(imageRatioItem), "activate", GTK_SIGNAL_FUNC(menuImageRatio_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    imageFreezeItem = gtk_check_menu_item_new_with_label ("Freeze");
    gtk_menu_append( GTK_MENU(imageMenu), imageFreezeItem);
    gtk_signal_connect( GTK_OBJECT(imageFreezeItem), "toggled", GTK_SIGNAL_FUNC(menuImageFreeze_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    synchroDisplayItem = gtk_check_menu_item_new_with_label ("Synch display");
    gtk_menu_append( GTK_MENU(imageMenu), synchroDisplayItem);
    gtk_signal_connect( GTK_OBJECT(synchroDisplayItem), "toggled", GTK_SIGNAL_FUNC(menuSynchroDisplay_CB), mainWindow);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    imageFramerateItem = gtk_menu_item_new_with_label ("Change refresh interval..");
    gtk_menu_append( GTK_MENU(imageMenu), imageFramerateItem);
    gtk_signal_connect( GTK_OBJECT(imageFramerateItem), "activate", GTK_SIGNAL_FUNC(menuImageFramerate_CB), mainWindow);
    // Submenu: Help
    helpMenu = gtk_menu_new();	
    helpAboutItem = gtk_menu_item_new_with_label ("About..");
    gtk_menu_append( GTK_MENU(helpMenu), helpAboutItem);
    gtk_signal_connect( GTK_OBJECT(helpAboutItem), "activate", GTK_SIGNAL_FUNC(menuHelpAbout_CB), mainWindow);
    // linking the submenus to items on menubar
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(fileItem), fileMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(imageItem), imageMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(helpItem), helpMenu);
    // appending the submenus to the menubar
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), fileItem);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), imageItem);
    gtk_menu_item_set_right_justified (GTK_MENU_ITEM (helpItem), TRUE);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), helpItem);
  
    return menubar;
}

static GtkWidget *xpm_label_box( gchar     *xpm_filename,gchar *label_text ) {
    GtkWidget *box;
    GtkWidget *label;
    GtkWidget *image;

    /* Create box for image and label */
    box = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box), 2);

    /* Now on to the image stuff */
    if(xpm_filename!=NULL)
        image = gtk_image_new_from_file (xpm_filename);

    /* Create a label for the button */
    label = gtk_label_new (label_text);

    /* Pack the image and label into the box */
    if(xpm_filename!=NULL)
        gtk_box_pack_start (GTK_BOX (box), image, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 3);

    if(xpm_filename!=NULL)
        gtk_widget_show (image);
    gtk_widget_show (label);

    return box;
}

static void scale_set_default_values( GtkScale *scale ) {
    gtk_range_set_update_policy (GTK_RANGE (scale),GTK_UPDATE_CONTINUOUS);
    gtk_scale_set_digits (scale, 2);
    gtk_scale_set_value_pos (scale, GTK_POS_TOP);
    gtk_scale_set_draw_value (scale, TRUE);
}

//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
void updateStatusbar(GtkWidget *statusbar, gchar *msg) {
    GtkStatusbar *sb=GTK_STATUSBAR (statusbar);

    gtk_statusbar_pop (sb, 0); // clear any previous message, underflow is allowed 
    gtk_statusbar_push (sb, 0, msg);
}

//-------------------------------------------------
// Main Window 
//-------------------------------------------------
GtkWidget* createMainWindow(void)
{
    //Module=this; //it is necessary to synchronise the static function with this class
    
    GtkRequisition actualSize;
    GtkWidget* window;
    GtkWidget *label;
    GtkWidget *separator;
    
    //gtk_init (&argc, &argv);
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "saliencyBlobFinderInterface");
    gtk_window_set_default_size(GTK_WINDOW (window), 320, 500); 
    gtk_window_set_resizable (GTK_WINDOW (window), TRUE);
    g_signal_connect (G_OBJECT (window), "destroy",
                      G_CALLBACK (cleanExit),
                      NULL);


    // Box for main window in a ordered list
    GtkWidget *box,*box2,*boxA,*box3,*box4;
    box = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box);
    // MenuBar for main window
    menubar = createMenubar();
    gtk_box_pack_start (GTK_BOX (box), menubar, FALSE, TRUE, 0); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    //gtk_widget_size_request(menubar, &actualSize);


    //Toolbox area
    //creates the area as collection of port processes sequence
    box2 = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box2);
    GtkWidget *button,*button2,*buttonCheck;
    GtkWidget *boxButton,*boxButton2;
    GtkWidget *boxButtons;
    GtkWidget *boxSliders;
    boxButtons = gtk_hbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxButtons), 0);
    boxSliders = gtk_hbox_new (TRUE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxSliders), 0);
     /* Create a new button */
    button = gtk_button_new ();
    button2 = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "output1");
    g_signal_connect (G_OBJECT (button2), "clicked",G_CALLBACK (callback), (gpointer) "output2");
    /* This calls our box creating func tion */
    
    boxButton = xpm_label_box (NULL,(gchar*)"output1");
    boxButton2= xpm_label_box (NULL, (gchar*)"output2");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_widget_show (boxButton2);
    gtk_container_add (GTK_CONTAINER (button2), boxButton2);
    gtk_widget_show (button2);
    //gtk_container_add (GTK_CONTAINER (boxButtons), button);
    //gtk_container_add (GTK_CONTAINER (boxButtons), button2);
    
    //---- vSeparator
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);

    
    //-----main section
    GtkWidget *scrollbar;
    
    
    GtkWidget *scale;
    GtkObject *adj1, *adj2,*adj3, *adj4,*adjr, *adjg, *adjb,*adjmin, *adjtime;
    GtkWidget *hscale, *vscale;


    adj1 = gtk_adjustment_new (0.0, 0.0, 101.0, 0.1, 1.0, 1.0);
    vscale = gtk_vscale_new (GTK_ADJUSTMENT (adj1));
    scale_set_default_values (GTK_SCALE (vscale));
    gtk_box_pack_start (GTK_BOX (boxSliders), vscale, TRUE, TRUE, 0);
    gtk_widget_show (vscale);

    /*separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (boxSliders), separator, FALSE, FALSE, 0);
    gtk_widget_show (separator);*/

    

    //----------BOXA SECTION:1
    //boxA is the area that contains the two subsection for watershed and saliency operators
    boxA = gtk_hbox_new (FALSE, 0);
    
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
    gtk_widget_show (boxA);
    
    //---- vSeparator
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (boxA), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
    
    //--box3 section A
    box3 = gtk_hbox_new (FALSE, 0);

    label = gtk_label_new ("Options:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));

    box4 = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box4), 0);

    label = gtk_label_new ("BOTTOM-UP:saliency linear combination Kcoeff.: isolated blobs are salient");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    double maxAdj=100;
    double minAdj=1;
    double stepAdj=1;
    
    adj1 = gtk_adjustment_new(50, minAdj,maxAdj,stepAdj, 1, 1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj1), "value_changed",
                      G_CALLBACK (cb_digits_scale), NULL);


    label = gtk_label_new ("TOP-DOWN:saliency linear combination Kcoeff.: match-colour blob are salient");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    adj2 = gtk_adjustment_new (50, minAdj,maxAdj,stepAdj, 1, 1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj2));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj2), "value_changed",
                      G_CALLBACK (cb_digits_scale2), NULL);
    
    label = gtk_label_new ("MAXBLOB dimension: cut off bigger blobs");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    adj3 = gtk_adjustment_new (4096, 10,6000,100, 1, 1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj3));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj3), "value_changed",
                      G_CALLBACK (cb_digits_scale3), NULL);

    
    label = gtk_label_new ("MINBLOB dimension: cut off smaller blobs");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    adj4 = gtk_adjustment_new (100,1,3000,1,1,1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj4));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj4), "value_changed",
                      G_CALLBACK (cb_digits_scale4), NULL);
    
    label = gtk_label_new ("red intensity target:");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    adjr = gtk_adjustment_new (1,1,255,1,1,1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjr));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjr), "value_changed",
                      G_CALLBACK (cb_digits_scaler), NULL);

    label = gtk_label_new ("green intensity target:");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    adjg = gtk_adjustment_new (1,1,255,1,1,1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjg));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjg), "value_changed",
                      G_CALLBACK (cb_digits_scaleg), NULL);

    label = gtk_label_new ("blue intensity target:");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    adjb = gtk_adjustment_new (1,1,255,1,1,1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjb));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjb), "value_changed",
                      G_CALLBACK (cb_digits_scaleb), NULL);

    label = gtk_label_new ("minBounding area:");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    adjmin = gtk_adjustment_new (225,100,1000,1,1,1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjmin));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjmin), "value_changed",
                      G_CALLBACK (cb_digits_scalemin), NULL);
    
    label = gtk_label_new ("time decimal constant1 (x y z):");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    adjtime = gtk_adjustment_new (10,1,100,1,1,1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjtime));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjtime), "value_changed",
                      G_CALLBACK (cb_digits_scaletime), NULL);


    label = gtk_label_new ("time decimal constant2 (set img x y):");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    adjtime = gtk_adjustment_new (10,1,100,1,1,1);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adjtime));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box4), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adjtime), "value_changed",
                      G_CALLBACK (cb_digits_scaletime2), NULL);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4


    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

    //---- vSeparator
    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
    
    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);

    label = gtk_label_new ("1channel output:");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    GtkWidget *buttonradio;
    GSList *group;

    buttonradio = gtk_radio_button_new_with_label (NULL, "Watershed-->");
    g_signal_connect (G_OBJECT (buttonradio), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Watershed");
    gtk_box_pack_start (GTK_BOX (box4), buttonradio, TRUE, TRUE, 0);
    gtk_widget_show (buttonradio);
    group = gtk_radio_button_group (GTK_RADIO_BUTTON (buttonradio));

    buttonradio = gtk_radio_button_new_with_label(group, "Tagged-->");
    g_signal_connect (G_OBJECT (buttonradio), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "Tagged");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonradio), TRUE);
    gtk_box_pack_start (GTK_BOX (box4), buttonradio, TRUE, TRUE, 0);
    gtk_widget_show (buttonradio);
    group = gtk_radio_button_group (GTK_RADIO_BUTTON (buttonradio));
    buttonradio = gtk_radio_button_new_with_label(group, "ContrastLP-->");
    g_signal_connect (G_OBJECT (buttonradio), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "ContrastLP");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonradio), TRUE);
    gtk_box_pack_start (GTK_BOX (box4), buttonradio, TRUE, TRUE, 0);
    gtk_widget_show (buttonradio);
    group = gtk_radio_button_group (GTK_RADIO_BUTTON (buttonradio));
    buttonradio = gtk_radio_button_new_with_label(group, "FoveaBlob-->");
    g_signal_connect (G_OBJECT (buttonradio), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "FoveaBlob");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonradio), TRUE);
    gtk_box_pack_start (GTK_BOX (box4), buttonradio, TRUE, TRUE, 0);
    gtk_widget_show (buttonradio);
    group = gtk_radio_button_group (GTK_RADIO_BUTTON (buttonradio));
    buttonradio = gtk_radio_button_new_with_label(group, "MaxSaliencyBlob-->");
    g_signal_connect (G_OBJECT (buttonradio), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "MaxSaliencyBlob");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonradio), TRUE);
    gtk_box_pack_start (GTK_BOX (box4), buttonradio, TRUE, TRUE, 0);
    gtk_widget_show (buttonradio);
    group = gtk_radio_button_group (GTK_RADIO_BUTTON (buttonradio));

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4


    box4=  gtk_vbox_new (FALSE, 0);

    label = gtk_label_new ("3channels output:");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);    

    //GtkWidget *buttonradio;
    //GSList *group;

    buttonradio = gtk_radio_button_new_with_label (group, "MeanColoursLP-->");
    g_signal_connect (G_OBJECT (buttonradio), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "MeanColourLP");
    gtk_box_pack_start (GTK_BOX (box4), buttonradio, TRUE, TRUE, 0);
    gtk_widget_show (buttonradio);
    group = gtk_radio_button_group (GTK_RADIO_BUTTON (buttonradio));

    buttonradio = gtk_radio_button_new_with_label(group, "none-->");
    g_signal_connect (G_OBJECT (buttonradio), "toggled",G_CALLBACK (cb_draw_value),(gpointer) "none");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonradio), TRUE);
    gtk_box_pack_start (GTK_BOX (box4), buttonradio, TRUE, TRUE, 0);
    gtk_widget_show (buttonradio);
    group = gtk_radio_button_group (GTK_RADIO_BUTTON (buttonradio));

    
    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4


    //------ HSEPARATOR ---------------
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

    //gtk_container_add (GTK_CONTAINER (box2), boxSliders);
    gtk_box_pack_start(GTK_BOX(box), box2,FALSE,FALSE, 10);
    // StatusBar for main window
    statusbar = gtk_statusbar_new ();
    //updateStatusbar(GTK_STATUSBAR (statusbar));
    gtk_box_pack_start (GTK_BOX (box), statusbar, FALSE, TRUE, 0);
    gtk_widget_size_request(statusbar, &actualSize);
    //_occupiedHeight += 2*(actualSize.height);

    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, 320, 240);
    // TimeOut used to refresh the screen
    timeout_ID = gtk_timeout_add (1000, timeout_CB, NULL);

    mainWindow=window;

    return window;
}



void configure(yarp::os::ResourceFinder rf){
    /* Process all parameters from both command-line and .ini file */
    /* get the module name which will form the stem of all module port names */
    _options.portName      = rf.check("portName", 
                           Value("selAttentionInterface"), 
                           "module name (string)").asString();
    _options.outPortName      = rf.check("outPortName", 
        Value("/selAttentionInterface/command:o"), 
                           "module name (string)").asString();
}

void setOptionsToDefault() {
    // Options defaults
    _options.refreshTime = 100;
    _options.outputEnabled = 0;
    _options.windWidth = 300;
    _options.windHeight = 300;
    _options.posX = 100;
    _options.posY = 100;
    _options.saveOnExit = 0;
}

bool openPorts() {
    _pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    //g_print("Registering port %s on network %s...\n", _options.outPortName, "default");
    bool ok = _pOutPort->open(_options.outPortName.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }
    return true;
}

void closePorts() {
    _pOutPort->close();
    bool ok = true;
    if  (ok)
        g_print("Port %s unregistration succeed!\n", _options.outPortName);
    else 
        g_print("ERROR: Port %s unregistration failed.\n", _options.outPortName);
    delete _pOutPort;
    _pOutPort = NULL;

}

void cleanExit() {
    if (timeout_ID!=0)
        g_source_remove (timeout_ID);
    timeout_ID = 0;
    
    g_source_remove(timeout_update_ID);
    timeout_update_ID=0;

    gtk_main_quit ();
}

//-------------------------------------------------
// Main
//-------------------------------------------------
#undef main //ace leaves a "main" macro defined

int myMain(int argc, char* argv[]) {
    yarp::os::Network yarp;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("selAttentionInterface.ini"); //overridden by --from parameter
    rf.setDefaultContext("logPolarAttention/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    //initialize threads in gtk, copied almost verbatim from
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    g_thread_init (NULL);
    gdk_threads_init ();
    gdk_threads_enter ();
    createObjects();
    _frameN = 0;
    timeout_ID = 0;
    setOptionsToDefault();
    configure(rf);
    
    // Parse command line parameters, do this before
    // calling gtk_init(argc, argv) otherwise weird things 
    // happens
    if (!openPorts())
        goto exitRoutine;

    // This is called in all GTK applications. Arguments are parsed
    // from the command line and are returned to the application.
    gtk_init (&argc, &argv);

    // create a new window
    mainWindow = createMainWindow();
    
    // Non Modal Dialogs
#if GTK_CHECK_VERSION(2,6,0)
    saveSingleDialog = createSaveSingleDialog();
    saveSetDialog = createSaveSetDialog();
#else
    printf("Functionality omitted for older GTK version\n");
#endif
    // Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
    gtk_window_move(GTK_WINDOW(mainWindow), _options.posX, _options.posY);
    // All GTK applications must have a gtk_main(). Control ends here
    // and waits for an event to occur (like a key press or
    // mouse event).

    //ptr_portCallback->attach(&_resources);
    //ptr_portCallback->attach(&portFpsData);
    //ptr_inputPort->useCallback(*ptr_portCallback);

    if (_options.synch)
    {
        setSynchroMode();
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), true);
    }
    else
    {
        setTimedMode(_options.refreshTime);
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), false);
    }

    gtk_main ();

exitRoutine:
    // leave critical section here. From example
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    gdk_threads_leave ();

    closePorts();

    deleteObjects();
    return 0;
}

#ifdef YARP_WIN32_NOCONSOLE
#include <windows.h>
// win32 non-console applications define WinMain as the
// entry point for the linker
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
    return myMain (__argc, __argv);
}
#else
int main(int argc, char* argv[]) {
    return myMain(argc, argv);
}
#endif

void printHelp() {
    g_print("selAttentionInterface usage:\n");
    g_print("--name: input port name (default: /selAttentionInterface)\n");
}

