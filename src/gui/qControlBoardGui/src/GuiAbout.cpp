/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/GuiAbout.h>        
        
        
GuiAbout::GuiAbout(QWidget* parent, const char* name, bool modal, WFlags){
    
    // position screen center
    QWidget* desk = QApplication::desktop();
    this->move(desk->width()/2 - this->width()/2,desk->height()/2 - this->height()/2);	
}

GuiAbout::~GuiAbout(){
    
}
        
void GuiAbout::setText(QString text){
    lblAbout->setText(text);
}
