// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __YARP_QWIDGETYARP__
#define __YARP_QWIDGETYARP__

// std
#include <string>

// Qt
#include <qwidget.h>

// yarp
#include <yarp/os/Searchable.h>

namespace yarp{
	namespace gui{
		class QWidgetYarp;
	}
}

class yarp::gui::QWidgetYarp {

public:

	virtual QWidget* getQWidget() = 0;

protected:
    
};
#endif 

