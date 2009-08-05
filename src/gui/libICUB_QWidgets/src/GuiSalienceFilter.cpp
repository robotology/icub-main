/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/GuiSalienceFilter.h>


GuiSalienceFilter::GuiSalienceFilter(int index, RemoteSalience *remote, QWidget* parent, const char* name, bool modal, WFlags fl)
	: GuiSalienceFilterBase( parent, name, fl )
{
	_remote = remote;
    _index = index;
    _sldSteps = 100.0;

    sldWeight->setMinValue(0);
    sldWeight->setMaxValue((int)_sldSteps);

    QVariant varIndex(_index);
    //string filterLabel = varIndex.asString() + string(": ") + _remote->getChildFilterName(_index);
    string filterLabel(remote->getChildFilterName(_index));
    cout << filterLabel << endl;
    //lblIndex->setText(string( string(varIndex.asString()) + string(": ") + string(_remote->getChildFilterName(_index))).c_str());
    lblIndex->setText(filterLabel.c_str());

    // set initial slider position and labels
    double weight = _remote->getChildWeight(_index);
    weight = floor(weight * pow( 10.0, 2.0) + 0.5) * pow(10.0,-2.0);
    // slider
    sldWeight->setValue((int)(weight*_sldSteps)); 
    QVariant varWeight(weight);
    lblWeight->setText(varWeight.toString());
}

GuiSalienceFilter::~GuiSalienceFilter()
{

}

void GuiSalienceFilter::sldWeight_sliderReleased()
{
    double weight = ((double)sldWeight->value()) / _sldSteps;
    QVariant varWeight(weight);
    lblWeight->setText(varWeight.toString());
    _remote->setChildWeight(_index, weight);
}
