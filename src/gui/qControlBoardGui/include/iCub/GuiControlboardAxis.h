
/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */



#ifndef __UZH_GUICONTROLBOARDAXIS__
#define __UZH_GUICONTROLBOARDAXIS__

// std
#include <iostream>
#include <string>
#include <math.h>        

// uzh
#include <guicontrolboardaxisbase.h>

// Qt
#include <qpushbutton.h>
#include <qwidget.h>
#include <qvalidator.h>
#include <qlineedit.h>
#include <qvariant.h>
#include <qcheckbox.h>
#include <qcombobox.h>
#include <qtimer.h>
#include <qgroupbox.h>
#include <qradiobutton.h>
#include <qslider.h>
#include <qlabel.h>        

//
//// Qwt
//#include <qwt/qwt_wheel.h>

// yarp
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardPid.h>

namespace iCub {
    namespace contrib {
        class GuiControlboardAxis;
    }
}

using namespace std;
using namespace yarp::dev;
using namespace iCub::contrib;

/**
* @ingroup uzhGui
* Gui for one axis of a controlboard
*/

class iCub::contrib::GuiControlboardAxis : public GuiControlboardAxisBase
{
	Q_OBJECT

public:
    GuiControlboardAxis( PolyDriver *dd, int axisIndex, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~GuiControlboardAxis();
	
protected:

    PolyDriver			*_dd;
    IPositionControl    *_ipos;
    IVelocityControl	*_ivel;
    IEncoders			*_ienc;
    IControlLimits      *_ilim;
    IPidControl			*_ipid;
    
    double              _deg2rad;
    double              _rad2deg;
    double				_sliderGain;
    
    int					_axisIndex;
    double              _dblStepSize;
    bool				_inhibitSliderAction;
	
//QTimer				*_timer;
	
    virtual void sldPosAbsolute_valueChanged( int val );
    virtual void rbtPos_toggled( bool flag);
    virtual void rbtVel_toggled( bool flag);
    virtual void btnPlus_clicked();
    virtual void btnMinus_clicked();
    virtual void lnePosAbsolute_returnPressed();
    virtual void lneStepSize_returnPressed();
    virtual void lneSpeed_returnPressed();
    virtual void lneMin_returnPressed();
    virtual void lneMax_returnPressed();
    virtual void lneCtrlP_returnPressed();
    virtual void lneCtrlI_returnPressed();
    virtual void lneCtrlD_returnPressed();
    virtual void btnStop_clicked();
    
//protected slots:

	//virtual void timerDone();
};

#endif 

