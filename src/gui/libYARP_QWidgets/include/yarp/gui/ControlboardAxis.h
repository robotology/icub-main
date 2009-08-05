/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef __CONTROLBOARDAXIS__
#define __CONTROLBOARDAXIS__

// std
#include <iostream>
#include <string>
#include <math.h>        

// uzh
#include <controlboardaxisbase.h>

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

namespace yarp {
    namespace gui {
        class ControlboardAxis;
    }
}


/**
* Gui for one axis of a controlboard
*/

class yarp::gui::ControlboardAxis : public ControlboardAxisBase
{
	Q_OBJECT

public:
	ControlboardAxis( yarp::dev::PolyDriver *dd, int axisIndex, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~ControlboardAxis();
	
protected:

    yarp::dev::PolyDriver			*_dd;
    yarp::dev::IPositionControl		*_ipos;
    yarp::dev::IVelocityControl		*_ivel;
    yarp::dev::IEncoders			*_ienc;
    yarp::dev::IControlLimits		*_ilim;
    yarp::dev::IPidControl			*_ipid;
	yarp::dev::IAmplifierControl	*_iamp;
    
    double              _deg2rad;
    double              _rad2deg;
    double				_sliderGain;
    
    int					_axisIndex;
    double              _dblStepSize;
	
	
    virtual void sldPosAbsolute_valueChanged( int val );
	virtual void sldPosAbsolute_sliderReleased();
    virtual void rbtPos_toggled( bool flag);
    virtual void rbtVel_toggled( bool flag);
    virtual void btnPlus_clicked();
    virtual void btnMinus_clicked();
    virtual void lnePosAbsolute_returnPressed();
    virtual void lneSpeed_returnPressed();
    virtual void lneMin_returnPressed();
    virtual void lneMax_returnPressed();
    virtual void lneCtrlP_returnPressed();
    virtual void lneCtrlI_returnPressed();
    virtual void lneCtrlD_returnPressed();
	virtual void lneCtrlVel_returnPressed();
	virtual void lneCtrlAcc_returnPressed();
	virtual void chbAmp_toggled(bool on);
    virtual void btnStop_clicked();
    
};

#endif 

