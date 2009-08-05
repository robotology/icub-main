/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/gui/QWidgetControlboard.h>


using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::gui;

QWidgetControlboard::QWidgetControlboard(){

}

QWidgetControlboard::QWidgetControlboard(yarp::os::Searchable &config, QWidget* parent, const char* name, bool modal, WFlags fl)
	: QWidgetControlboardBase( parent, name, fl )
{
	
    frmMainLayout = new QVBoxLayout( frmMain, 11, 6, "frmMainLayout"); 
    
        // scroll view
    _scrlView = new QScrollView(frmMain);
    frmMainLayout->addWidget(_scrlView);
    //_scrlView->setVScrollBarMode(QScrollView::AlwaysOn);
    _scrlView->setResizePolicy(QScrollView::AutoOneFit);

    // vbox for sub configurable buttons
    _scrlBox = new QVBox(_scrlView->viewport());
    _scrlBox->setSpacing(0);
    _scrlBox->setMargin(0);
    _scrlView->addChild(_scrlBox);
    
	Property propBoard;
	propBoard.fromString(config.toString());

	// local port
    if (propBoard.check("name")){
        propBoard.put("local", propBoard.find("name").asString().c_str());
    }
    else{
        propBoard.put("local", "/guiControlboard");
    }
    
    // controlboard driver
    propBoard.put("device", "remote_controlboard");

    // create the controlboard
    _dd.open(propBoard);
    
    if(!_dd.view(_ipos))
        cout << "Device does not implement IPositionControl!" << endl;
    else{	
        _ipos->getAxes(&_numAxes);
        for (int i = 0; i < _numAxes; i++){
                new ControlboardAxis(&_dd, i, _scrlBox);
        }	
    }
        
    // position screen center
    QWidget* desk = QApplication::desktop();
    this->move(desk->width()/2 - this->width()/2,desk->height()/2 - this->height()/2);	
	
}

QWidgetControlboard::~QWidgetControlboard()
{
	_dd.close();

}


//void printHelp(){
//
//    cout << endl;
//    cout << "Help:" << endl;
//    cout << "To connect the GuiControlBoard to a server control board"  << endl;
//    cout << "the port of the server control board has to be specified by" << endl;
//    cout << "the commandline option '--remote'" << endl;
//    cout << "Example: " << endl;
//    cout << "$> guiControlBoard --remote /port/of/servercontrolboard" << endl;
//    cout << endl;
//    cout << "Optionally you can specify the local port of the remote control" << endl;
//    cout << "board as well." << endl;
//    cout << "Example: " << endl;
//    cout << "$> guiControlBoard --remote /port/of/servercontrolboard --local /port/of/remotecontrolboard"  << endl;
//    cout << endl;
//}

///**
// * @ingroup icub_module
// *
// * \defgroup icub_qcontrolboardgui qcontrolboardgui
// *
// * Graphical user interface for a yarp controlboard. \n
// * Run without options to see help on usage. \n
// * \n
// * The module runs a remote control board. To connect to any server control board pass '--remote /yarp/port/servercontrolboard'
// * at the command line. If you wish to specify the port of the remote_controlboard of
// * qcontrolboardgui as well specify something like '--local /yarp/port/remotecontrolboard' at the commandline.\n
// * The server control board has to provide the interfaces IPositionControl, IVelocityControl, IControlLimits, IEncoders and IPidControl
// * for full qcontrolboardgui functionality.
// *
// *  \image html qcontrolboardgui.jpg
// *  \image latex qcontrolboardgui.eps "qcontrolboardgui running on Linux" width=10cm
// *
// * \see iCub::contrib::QWidgetControlboard
// *
// * \author Jonas Ruesch
// *
// */
 