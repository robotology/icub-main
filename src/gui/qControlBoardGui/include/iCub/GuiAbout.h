/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __UZH_GUIABOUT__
#define __UZH_GUIABOUT__

// std
#include <iostream>
#include <string>     

// uzh
#include <guiaboutbase.h>

// Qt
#include <qapplication.h>
#include <qpushbutton.h>
#include <qwidget.h>
#include <qvalidator.h>
#include <qlabel.h> 
#include <qstring.h>       

namespace iCub {
    namespace contrib {
        class GuiAbout;
    }
}

using namespace std;
using namespace iCub::contrib;

/**
* @ingroup uzhGui
* About dialog box.
*/

class iCub::contrib::GuiAbout : public GuiAboutBase
{
	Q_OBJECT

public:
    GuiAbout(QWidget* parent = 0, const char* name = 0, bool modal = FALSE, WFlags fl = 0 );
    virtual ~GuiAbout();

    virtual void setText(QString text);
};

#endif 

