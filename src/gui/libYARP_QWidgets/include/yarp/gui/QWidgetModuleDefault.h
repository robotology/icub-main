// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_QWIDGETMODULEDEFAULT__
#define __ICUB_QWIDGETMODULEDEFAULT__

// std
#include <iostream>
#include <string>
#include <vector>

// iCub
#include <yarp/gui/QWidgetViewer.h>
#include <yarp/gui/QWidgetConnections.h>
#include <yarp/gui/QWidgetOutput.h>
#include <yarp/gui/QWidgetRPC.h>
#include <yarp/gui/QWidgetYarp.h>

// Qt
#include <qsplitter.h>
#include <qlabel.h>
#include <qapplication.h>
#include <qvaluelist.h>

// yarp
#include <yarp/os/Searchable.h>

namespace yarp {
    namespace gui {
        class QWidgetModuleDefault;
    }
}


class yarp::gui::QWidgetModuleDefault : 
	public QWidget,
	public yarp::gui::QWidgetYarp {

	Q_OBJECT

public:
    QWidgetModuleDefault();
	QWidgetModuleDefault( yarp::os::Searchable& config, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~QWidgetModuleDefault();
	
	virtual QWidget* getQWidget(){return this;}

protected:
    
	QWidgetViewer *_qwViewer;
	QWidgetConnections *_qwConnections;
	QWidgetOutput *_qwOutput;
	QWidgetRPC *_qwRPC;

protected slots:

	void btnCheckAll_clicked();

};
#endif 

