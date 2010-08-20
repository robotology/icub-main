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


/**
*
@ingroup icub_guis
\defgroup salBlobFinderInterface salBlobFinderInterface

Module graphical interface for saliencyBlobFinder

\section intro_sec Description
This module is purely an interface towards the module saliencyBlobFinder. It sends commands via /command:o port.
The protocol respects the communication directives that the saliencyBlobFinder requires.
In addition this interface is able to draw an image. This useful functionality allows the user to visualise straigh away important information 
and result of his/her interaction

The module does:
-   stream commands to the saliencyBlobFinder module

\image html saliencyBlobFinder.png

In the GUI there are some sliding controls on the left hand side which are in sequence:
<ul>
    <li>coefficients for the algorithm that defines the saliency of the blob </li>
    <li>min and max dimension of the blobs which will be analysed</li>
    <li>controls for the selection of the colour which the system has to look for</li>
</ul>

On the right side of the GUI there is a series of checkbox which select the typology of the output sent on the /image:o port.
The last checkbox that toggle from OFF to ON selects the output. It can be choosen between this possible alternatives:
<ul>
<li> tagged: the image composed by all the blobs filled with a gray scale equal to their list position</li>
<li> watershed: the result of the watershed operation</li>
<li> foveaBlob: the representation of the blob foveated in that moment</li>
<li> colourVQ: image composed by all the blobs filled with quantization colour</li>
<li> meanColour: the composition of all the colour filled with the mean colour of all the pixel which belong to the blob</li>
<li> maxSaliencyBlob: the representation of the max saliency blob</li>
</ul>



\section lib_sec Libraries
YARP
GTK

\section parameters_sec Parameters
--name : name of the module and relative port name

\section portsa_sec Ports Accessed
/blobFinder/cmd


\section portsc_sec Ports Created
Input ports:
- <name>/cmd
Outports:
- <name>/command:o

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
saliencyBlobFinderInterface 


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

#if !defined(SAL_BLOBFINDERINTERFACE_H)
#define SAL_BLOBFINDERINTERFACE_H

//=============================================================================
// YARP Includes - Class Specific
//=============================================================================
#include <yarp/sig/Image.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
//=============================================================================
// GTK Includes 
//=============================================================================
#include <gtk/gtk.h>
#include "viewerResources.h"
#include <string>


// general command vocab's
#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_IS VOCAB2('i','s')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_MAXDB VOCAB3('M','d','b')         // maximum dimension of the blob drawn
#define COMMAND_VOCAB_MINDB VOCAB3('m','d','b')         // minimum dimension of the blob drawn
#define COMMAND_VOCAB_MBA VOCAB3('m','B','A')           // minimum dimension of the bounding area

//-------------------------------------------------
// Callbacks
//-------------------------------------------------
gboolean forceDraw(gpointer data);
// Timeout CB
gint timeout_CB (gpointer data);
// Main Window close CB
gboolean delete_event( GtkWidget *widget, GdkEvent *event, gpointer data );
//  Window redraw CB
gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data);
// Click on Drawinf Area CB
gint clickDA_CB (GtkWidget *widget, GdkEventButton *event, gpointer data);
// Menubar CBs
gint menuFileQuit_CB(GtkWidget *widget, gpointer data);
gint menuHelpAbout_CB(GtkWidget *widget, gpointer data);
gint menuImageInterval_CB(GtkWidget *widget, gpointer data);

void setTimedMode(guint dT);

void setSynchroMode();

void saveImageNow();

//-------------------------------------------------
// Non Modal Dialogs
//-------------------------------------------------
GtkWidget* createSaveSingleDialog();
GtkWidget* createSaveSetDialog();
//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* createMenubar();
//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
void updateStatusbar(GtkWidget *statusbar, gchar *msg);
//-------------------------------------------------
// Main Window
//-------------------------------------------------
GtkWidget* createMainWindow(void);
//-------------------------------------------------
// Service Functions
//-------------------------------------------------
bool getImage();
void parseOptFile(char *fileName);
bool parseParameters(int argc, char *argv[]);
void saveOptFile(char *fileName);
void setOptionsToDefault();
bool openPorts();
void closePorts();
bool setUp();
void cleanExit();

void printHelp();

//-------------------------------------------------
// Global Variables
//-------------------------------------------------
// main window 


//-------------------------------------------------
// Program Options 
//-------------------------------------------------
struct mOptions
{
    unsigned int    refreshTime;
    std::string     portName;
    char            networkName[256];
    int             windWidth;
    int             windHeight;
    int             posX;
    int             posY;
    char            fileName[256];
    int             saveOnExit;
    std::string           outPortName;
    char            outNetworkName[256];
    int             outputEnabled;
    bool            synch;
};
typedef struct mOptions pgmOptions;



#endif // #if !defined(SAL_BLOBFINDERINTERFACE_H)
