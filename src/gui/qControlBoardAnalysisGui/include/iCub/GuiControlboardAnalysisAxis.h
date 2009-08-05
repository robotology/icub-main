
/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */



#ifndef __UZH_GUICONTROLBOARDANALYSISAXIS__
#define __UZH_GUICONTROLBOARDANALYSISAXIS__

// std
#include <iostream>
#include <string>
#include <math.h>        

// uzh
#include <guicontrolboardanalysisaxisbase.h>

// Qt
#include <qlayout.h>
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
#include <qpen.h>
#include <qcolor.h>

// Qwt
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_classes.h>

// yarp
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardPid.h>

namespace iCub {
    namespace contrib {
        class GuiControlboardAnalysisAxis;
    }
}

using namespace std;
using namespace yarp::dev;
using namespace iCub::contrib;

/**
* @ingroup uzhGui
* Position, velocity and acceleration plot of a motor axis
*/

class iCub::contrib::GuiControlboardAnalysisAxis : public GuiControlboardAnalysisAxisBase
{
	Q_OBJECT

public:
    GuiControlboardAnalysisAxis(){};
    GuiControlboardAnalysisAxis(yarp::os::Searchable &config, int axisIndex, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~GuiControlboardAnalysisAxis();
	
    void                put(double time, double pos, double vel, double acc, double err, double out);
    void                togglePosition(bool state);
    void                toggleVelocity(bool state);
    void                toggleAcceleration(bool state);
    void                toggleError(bool state);
    void                toggleOutput(bool state);
    
    void                scalePosition(double scale);
    void                scaleVelocity(double scale);
    void                scaleAcceleration(double scale);
    void                scaleError(double scale);
    void                scaleOutput(double scale);
    
//     void                showPosition(bool state){_showPos = state;}
//     void                showVelocity(bool state){_showVel = state;}
//     void                showAcceleration(bool state){_showAcc = state;}
    
public slots:
    virtual void chbPos_stateChanged( int );
    virtual void chbVel_stateChanged( int );
    virtual void chbAcc_stateChanged( int );
    virtual void chbErr_stateChanged( int );
    virtual void chbOut_stateChanged( int );
    
    virtual void lneScalePos_returnPressed();
    virtual void lneScaleVel_returnPressed();
    virtual void lneScaleAcc_returnPressed();
    virtual void lneScaleErr_returnPressed();
    virtual void lneScaleOut_returnPressed();
    
protected:

    QVBoxLayout         *frmMainLayout;

    //QwtPlotCurve
    QwtPlot*            _plot;
    long                _curvePos;
    long                _curveVel;
    long                _curveAcc;
    long                _curveErr;
    long                _curveOut;

    // buffering for plots
    int                     _bufSize;
    double                  *_bufPos;
    double                  *_bufVel;
    double                  *_bufAcc;
    double                  *_bufErr;
    double                  *_bufOut;
    double                  *_bufTime;

    int                 _axisIndex;
    
     // configuration
    double                  _scalePos;
    double                  _scaleVel;
    double                  _scaleAcc;
    double                  _scaleErr;
    double                  _scaleOut;
    
//     bool                    _showPos;
//     bool                    _showVel;
//     bool                    _showAcc;

};

#endif 

