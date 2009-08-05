/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __UZH_GUISALIENCE__
#define __UZH_GUISALIENCE__

// std
#include <iostream>
#include <string>
#include <vector>

// iCub
#include <guisaliencebase.h>
#include <iCub/GuiSalienceFilter.h>
#include <iCub/RemoteSalience.h>
#include <yarp/gui/QWidgetViewer.h>
#include <yarp/gui/QWidgetConnections.h>
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
#include <yarp/os/Network.h>
#include <yarp/os/Searchable.h>

namespace iCub {
    namespace contrib {
        class GuiSalience;
    }
}

class iCub::contrib::GuiSalience : 
	public GuiSalienceBase,
	public yarp::gui::QWidgetYarp {

	Q_OBJECT

public:
    GuiSalience();
	GuiSalience( yarp::os::Searchable& config, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~GuiSalience();
	
	virtual void updateUI();

	virtual QWidget* getQWidget(){return this;}

protected:
    
	iCub::contrib::RemoteSalience      _remoteSalience;

    QVBoxLayout	        *frmMainLayout;
    QScrollView         *_scrlView;
    QFrame              *_scrlFrame;
	QVBoxLayout			*_scrlFrameLayout;
	QSpacerItem			*_spacer;
	yarp::gui::QWidgetConnection	*_qwConfConnection;

	std::vector<iCub::contrib::GuiSalienceFilter*> _lstFilterWidgets;

    virtual void lneThreshold_returnPressed();
    virtual void lneBlur_returnPressed();
	virtual void btnReinitialize_clicked();

	virtual void initializeUI();
};
#endif 

