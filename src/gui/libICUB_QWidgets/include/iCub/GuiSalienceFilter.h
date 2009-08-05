
/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */



#ifndef __UZH_GUISALIENCEFILTER__
#define __UZH_GUISALIENCEFILTER__

// std
#include <iostream>
#include <string>
#include <math.h>        

// iCub
#include <guisaliencefilterbase.h>
#include <iCub/RemoteSalience.h>

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

namespace iCub {
    namespace contrib {
        class GuiSalienceFilter;
    }
}

using namespace std;
using namespace iCub::contrib;

/**
* Graphical user interface for a salience filter
*/

class iCub::contrib::GuiSalienceFilter : public GuiSalienceFilterBase
{
	Q_OBJECT

public:
    GuiSalienceFilter(int index, RemoteSalience *remote, QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~GuiSalienceFilter();
	
protected:
    int					_index;
    RemoteSalience      *_remote;
    double              _sldSteps;

    virtual void sldWeight_sliderReleased();
};

#endif 

