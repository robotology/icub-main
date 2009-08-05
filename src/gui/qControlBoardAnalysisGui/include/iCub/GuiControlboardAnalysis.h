/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __UZH_GUICONTROLBOARDANALYSIS__
#define __UZH_GUICONTROLBOARDANALYSIS__

// std
#include <iostream>
#include <string>

// uzh
#include <guicontrolboardanalysisbase.h>
#include <iCub/GuiControlboardAnalysisAxis.h>
#include <iCub/GuiCBAAbout.h>

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

// Qwt


// yarp
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

namespace iCub {
    namespace contrib {
        class GuiControlboardAnalysis;
    }
}

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace iCub::contrib;

class iCub::contrib::GuiControlboardAnalysis : public GuiControlboardAnalysisBase
{
	Q_OBJECT

public:
    GuiControlboardAnalysis( Property propContorlboard, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~GuiControlboardAnalysis();
    
public slots:
        
    virtual void lneRefreshTime_returnPressed();

protected:
    
    GuiCBAAbout         *_guiAbout;

    QVBoxLayout	        *frmMainLayout;
    QScrollView         *_scrlView;
    QVBox               *_scrlBox;

    GuiControlboardAnalysisAxis **_axisGuis;

    PolyDriver              _dd;
    IEncoders               *_ienc;
    IPidControl             *_ipid;
    	
    int	                    _numAxes;
    int                     _numActiveAxes;
    int                     *_activeAxes;
    double                  *_pos;
    double                  *_vel;
    double                  *_acc;
    double                  *_err;
    double                  *_out;
    

    QTimer                  *_timer;
    int                     _refreshTime;
    double                  _timeStart;

    virtual void btnQuit_clicked();
    virtual void btnAbout_clicked();

protected slots:
    void                timerdone();
  
};

#endif 

