// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_QVIEWER__
#define __ICUB_QVIEWER__

// std
#include <iostream>
#include <string>

// iCub
#include <yarp/gui/QWidgetViewer.h>
#include <qviewerbase.h>

// Qt
#include <qapplication.h>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Property.h>

namespace iCub {
    namespace contrib {
        class QViewer;
    }
}

class iCub::contrib::QViewer : public QViewerBase
{
	Q_OBJECT

public:
    QViewer();
	QViewer(yarp::os::Searchable& config, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~QViewer();
	
protected:
   
	yarp::gui::QWidgetViewer *_qwViewer;
};
#endif 

