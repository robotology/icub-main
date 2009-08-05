/**
 * @ingroup icub_module
 *
 * \defgroup icub_guiDemo guiDemo
 *
 * This module is part of the application \ref icub_drummingEPFL "drummingEPFL"
 *
 *\section intro_sec Description 
 *
 * This module creates a GUI that allows for the online specification of a score to be played. If you want to  use the whole drumming application, please refer to \ref icub_drummingEPFL "drummingEPFL".
 *
 *
 *\section lib_sec Libraries
 *
 * This module requires Qt3 to be installed. 
 *
 *\section parameters_sec Parameters
 *
 * This module does not take any parameters. 
 *
 *\section portssa_sec Ports Accessed 
 *
 * Input ports\n
 * <ul>
 * <li> for each active \a part of the robot: /part/score/in (created by the \ref icub_DrumManager "DrumManager" module)
* <li> for each active \a part of the robot: /part/phase/in (created by the \ref icub_DrumManager "DrumManager" module)
 * <li> a port /interactive/in (created by the \ref icub_DrumManager "DrumManager" module)
 * </ul>
 *  
 *
 *\section portsc_sec Ports Created
 *
 * Output ports\n
 * <ul>
 * <li> For each \a part of the robot, a corresponding port /part/score/out sends to the \ref icub_DrumManager "DrumManager"  a vector of integers corresponding to the target positions at the different time steps;
* <li> For each \a part of the robot, a corresponding port /part/phase/out sends to the \ref icub_DrumManager "DrumManager" a vector of doubles corresponding to the phase shift of each part at the different time steps;
 * <li> A port /interactive/out sends to the \ref icub_DrumManager "DrumManager" a vector corresponding to the frequency at the different time steps. \n
 *</ul>
 *
 *\section in_files_sec Input Data Files
 *
 * This modules browse the folder src/drummingEPFL/guiDemo3/part to find predefined scores (i.e. .csv files). An example of a score file can be found in this directory. The length of the score need to be 16. 
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./guiDemo
 *
 * This file can be edited at \in src/drummingEPFL/guiDemo3/main.cpp 
 *
 *\authors Sarah Degallier, Ludovic Righetti
 *
 **/



// Header for the QApplication class
#include <qapplication.h>


// Header for our dialog
#include "maindlg.h"

// Remember the ( int, char** ) part as the QApplication needs them
int main( int argc, char **argv )
{
  // We must always have an application
  QApplication a( argc, argv );

  mainDlg *m = new mainDlg();   // We create our dialog
  a.setMainWidget( m );         // It is our main widget
  m->show();                    // Show it...

  return a.exec();              // And run!
} 
