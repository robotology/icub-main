// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "GuiRawData.h"

Glib::RefPtr<Gdk::Pixbuf> GuiRawData::mIconWarning;
Glib::RefPtr<Gdk::Pixbuf> GuiRawData::mIconError;
Glib::RefPtr<Gdk::Pixbuf> GuiRawData::mIconEmpty;

Gdk::Color GuiRawData::mColorWarning("#FFCC44");
Gdk::Color GuiRawData::mColorError("#FF5555");
Gdk::Color GuiRawData::mColorEmpty("#FFFFFF");
Gdk::Color GuiRawData::mColorBlack("#000000");

