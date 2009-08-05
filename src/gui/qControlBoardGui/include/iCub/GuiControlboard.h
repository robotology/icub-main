/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __UZH_GUICONTROLBOARD__
#define __UZH_GUICONTROLBOARD__

// std
#include <iostream>
#include <string>

// uzh
#include <guicontrolboardbase.h>
#include <iCub/GuiControlboardAxis.h>
#include <iCub/GuiAbout.h>

// Qt
#include <qapplication.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <qvalidator.h>
#include <qlineedit.h>
#include <qvariant.h>
#include <qcheckbox.h>
#include <qcombobox.h>
#include <qtimer.h>
#include <qgroupbox.h>
#include <qlayout.h>
#include <qscrollview.h>
#include <qvbox.h>
#include <qmenubar.h>        
#include <qpopupmenu.h>

// yarp
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

namespace iCub {
    namespace contrib {
        class GuiControlboard;
    }
}

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace iCub::contrib;

class iCub::contrib::GuiControlboard : public GuiControlboardBase
{
	Q_OBJECT

public:
    GuiControlboard();
	GuiControlboard( Property propContorlboard, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
	virtual ~GuiControlboard();
	
protected:
    
    GuiAbout            *_guiAbout;

	QVBoxLayout			*frmMainLayout;
    QScrollView         *_scrlView;
    QVBox               *_scrlBox;
	
	PolyDriver			_dd;
	IPositionControl	*_ipos;
	
	int					_numAxes;
    
	virtual void btnQuit_clicked();
    virtual void btnAbout_clicked();
  
   
};

#endif 

