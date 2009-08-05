/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __QTGUICONTROLBOARD__
#define __QTGUICONTROLBOARD__

// std
#include <iostream>
#include <string>

// iCub
#include <qwidgetcontrolboardbase.h>
#include <yarp/gui/ControlboardAxis.h>
#include <yarp/gui/QWidgetYarp.h>

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

namespace yarp {
    namespace gui {
        class QWidgetControlboard;
    }
}


class yarp::gui::QWidgetControlboard : 
	public QWidgetControlboardBase,
	public yarp::gui::QWidgetYarp {

	Q_OBJECT

public:
    QWidgetControlboard();
	QWidgetControlboard(yarp::os::Searchable &config, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
	virtual ~QWidgetControlboard();
	
	virtual QWidget* getQWidget(){return this;}

protected:

	QVBoxLayout			*frmMainLayout;
    QScrollView         *_scrlView;
    QVBox               *_scrlBox;
	
	yarp::dev::PolyDriver			_dd;
	yarp::dev::IPositionControl		*_ipos;
	
	int					_numAxes;
  
};

#endif 

