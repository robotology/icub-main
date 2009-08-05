/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __UZH_GUICBAABOUT__
#define __UZH_GUICBAABOUT__

// std
#include <iostream>
#include <string>     

// uzh
#include <guicbaaboutbase.h>

// Qt
#include <qapplication.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <qvalidator.h>
#include <qlabel.h> 
#include <qstring.h>       

namespace iCub {
    namespace contrib {
        class GuiCBAAbout;
    }
}

using namespace std;
using namespace iCub::contrib;

/**
* @ingroup uzhGui
* About dialog box.
*/

class iCub::contrib::GuiCBAAbout : public GuiCBAAboutBase
{
	Q_OBJECT

public:
    GuiCBAAbout(QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~GuiCBAAbout();

    virtual void setText(QString text);
};

#endif 

