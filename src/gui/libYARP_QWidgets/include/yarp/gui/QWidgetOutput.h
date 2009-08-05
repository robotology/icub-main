// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_QWIDGETOUTPUT__
#define __ICUB_QWIDGETOUTPUT__

// std
#include <iostream>
#include <string>

// yarp
#include <yarp/os/all.h>

// iCub
#include <qwidgetoutputbase.h>
#include<yarp/gui/QWidgetConnection.h>
#include<yarp/gui/StdoutCallback.h>

// Qt
#include <qtimer.h>
#include <qlayout.h>
#include <qtextedit.h>
#include <qgroupbox.h>
#include <qstring.h>

namespace yarp {
    namespace gui {
        class QWidgetOutput;
    }
}

class yarp::gui::QWidgetOutput : public QWidgetOutputBase
{
	Q_OBJECT

public:
    QWidgetOutput();
	QWidgetOutput(yarp::os::Searchable &config, QWidget *parent, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~QWidgetOutput();

	virtual void updateUI();
	virtual bool close();

protected:

	StdoutCallback _stdoutCallback;
	yarp::os::BufferedPort<yarp::os::Bottle> _prtStdout;
	
	QWidgetConnection *_qwConStdout;

protected slots:
	virtual void timerDone();
};
#endif 

