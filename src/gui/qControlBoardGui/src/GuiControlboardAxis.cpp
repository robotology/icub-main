/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/GuiControlboardAxis.h>


GuiControlboardAxis::GuiControlboardAxis(PolyDriver *dd, int axisIndex, QWidget* parent, const char* name, bool modal, WFlags fl)
	: GuiControlboardAxisBase( parent, name, fl )
{
            
    _sliderGain = 100.0;
    _inhibitSliderAction = false;
    
    _dd = dd;
    _ipos = NULL;
    _ivel = NULL;
    _ienc = NULL;
    _ilim = NULL;
    _ipid = NULL;
	
    if (!(dd->view(_ipos)))
        cout << "device does not implement IPositionControl!" << endl;
    if (!(dd->view(_ivel)))
        cout << "device does not implement IVelocityControl!" << endl;
    if (!(dd->view(_ienc)))
    	cout << "device does not implement IEncoders!" << endl;
    if (!(dd->view(_ilim)))
    	cout << "device does not implement IControlLimits!" << endl;
    if (!(dd->view(_ipid)))
    	cout << "device does not implement IPidControl!" << endl;
    
    _axisIndex = axisIndex;
    QVariant varIndex(_axisIndex);
    gbxAxis->setTitle("Axis: " + varIndex.asString());
    
    // set modus
    if (_ipos != NULL){
        _ipos->setPositionMode();
        //_ipos->setRefSpeed(_axisIndex,110); // TODO check this
        rbtPos->setChecked(true);
    }
    else{
       if(_ivel !=NULL){
            _ivel->setVelocityMode();
            rbtVel->setChecked(true);
       }
    }
    
    
    // limits
    if (_ilim != NULL){
        double min, max;
        _ilim->getLimits(_axisIndex, &min, &max);
        // round to 1/100 and rad2deg
		min = floor(min * pow( 10.0, 2.0) + 0.5) * pow(10.0, -2.0);
		max = floor(max * pow( 10.0, 2.0) + 0.5) * pow(10.0, -2.0);
        QVariant varmin(min);
        QVariant varmax(max);
        // labels
        lblMin->setText(varmin.asString());
        lblMax->setText(varmax.asString());
        // limit line edits
        lneMin->setText(varmin.asString());
        lneMax->setText(varmax.asString());
        // slider
        sldPosAbsolute->setMinValue((int)(min * _sliderGain));
        sldPosAbsolute->setMaxValue((int)(max * _sliderGain));
    }
    
    if (_ipid != NULL){
    	
    	double p,i,d;
    	yarp::dev::Pid pid;
    	_ipid->getPid(_axisIndex, &pid);
    	p = floor(pid.kp * pow( 10.0, 4.0) + 0.5) * pow(10.0, -4.0);
    	i = floor(pid.ki * pow( 10.0, 4.0) + 0.5) * pow(10.0, -4.0);
    	d = floor(pid.kd * pow( 10.0, 4.0) + 0.5) * pow(10.0, -4.0);
    	QVariant varp(p);
    	QVariant vari(i);
    	QVariant vard(d);
    	lneCtrlP->setText(varp.asString());
    	lneCtrlI->setText(vari.asString());
    	lneCtrlD->setText(vard.asString());
    }
    
    // current position
     if (_ienc != NULL){
         double enc;
         _ienc->getEncoder(_axisIndex, &enc);
         enc = floor(enc * pow( 10.0, 4.0) + 0.5) * pow(10.0,-4.0);
         // slider
         _inhibitSliderAction = true;
        sldPosAbsolute->setValue((int)(enc));
         _inhibitSliderAction = false;   
        QVariant varenc(enc);
        // line edit position
        lnePosAbsolute->setText(varenc.toString());
     }
    
    // defaults
    _dblStepSize = 1.0;
    QVariant stepSize(_dblStepSize);
    lneStepSize->setText(stepSize.asString());
    
    // line edit validators
    lneSpeed->setValidator(new QDoubleValidator(this));
    lnePosAbsolute->setValidator(new QDoubleValidator(this));
    lneStepSize->setValidator(new QDoubleValidator(this));
    lneMin->setValidator(new QDoubleValidator(this));
    lneMax->setValidator(new QDoubleValidator(this));
    lneCtrlP->setValidator(new QDoubleValidator(this));
    lneCtrlI->setValidator(new QDoubleValidator(this));
    lneCtrlD->setValidator(new QDoubleValidator(this));
    
	
    /*
	// timer for data acquisition
    _timer = new QTimer();
    connect( _timer, SIGNAL(timeout()), this, SLOT(timerDone()) );
    _timer->start(200);*/

	//rbtPos->setDown(true);	
	//rbtPos_toggled(true);
}

GuiControlboardAxis::~GuiControlboardAxis()
{
	//_timer->stop();
	//delete _timer;
}



void GuiControlboardAxis::sldPosAbsolute_valueChanged( int val )
{
	//cout << "slider value changed... to: " << val << endl;
        
    if (!_inhibitSliderAction){
	   if (_ipos != NULL){
		   _ipos->positionMove(_axisIndex, (((double)val)/_sliderGain));
	    }
		
		QVariant varVal(((double)val)/_sliderGain);
		lnePosAbsolute->setText(varVal.asString());
    }
}



void GuiControlboardAxis::rbtPos_toggled( bool flag)
{
	if (flag){
            if (_ipos != NULL){
                    //cout << "position mode set" << endl;
                    _ipos->setPositionMode();
                    lnePosAbsolute->setEnabled(true);
                    sldPosAbsolute->setEnabled(true);
                    lneStepSize->setEnabled(true);
                    btnPlus->setEnabled(true);
                    btnMinus->setEnabled(true);
                    lneSpeed->setEnabled(false);
            }
	}
}


void GuiControlboardAxis::rbtVel_toggled( bool flag)
{
	if (flag){
        if (_ivel != NULL){
            //cout << "velocity mode set" << endl;
            _ivel->setVelocityMode();
            lnePosAbsolute->setEnabled(false);
            sldPosAbsolute->setEnabled(false);
            lneStepSize->setEnabled(false);
            btnPlus->setEnabled(false);
            btnMinus->setEnabled(false);
            lneSpeed->setEnabled(true);
        }
	}
}


void GuiControlboardAxis::lnePosAbsolute_returnPressed()
{
	QVariant pos(lnePosAbsolute->text());
    if (_ipos != NULL){
        _ipos->positionMove(_axisIndex, pos.asDouble());
        _inhibitSliderAction = true;
        sldPosAbsolute->setValue((int)(pos.asDouble() * _sliderGain));
        _inhibitSliderAction = false;
    }
}

 void GuiControlboardAxis::lneStepSize_returnPressed()
{
    QVariant stepSize(lneStepSize->text());
    _dblStepSize = stepSize.asDouble();
}


void GuiControlboardAxis::lneSpeed_returnPressed()
{
	QVariant speed(lneSpeed->text());
    if (_ivel != NULL){
	   _ivel->velocityMove(_axisIndex, speed.asDouble());
    }
}

void GuiControlboardAxis::lneMin_returnPressed(){
    QVariant varmin(lneMin->text());
    QVariant varmax(lneMax->text());
    if (_ilim != NULL){
    	//cout << "guiControlboard axis: " << _axisIndex << " setting limits to: " << varmin.asDouble() * _deg2rad << " " << varmax.asDouble() * _deg2rad << endl;  
        _ilim->setLimits(_axisIndex, varmin.asDouble() , varmax.asDouble());
        
     	// adjust slider
        double min = floor(varmin.asDouble() * pow( 10.0, 2.0) + 0.5) * pow(10.0, -2.0);
		double max = floor(varmax.asDouble() * pow( 10.0, 2.0) + 0.5) * pow(10.0, -2.0);
        varmin = min;
        varmax = max;
        // labels
        lblMin->setText(varmin.asString());
        lblMax->setText(varmax.asString());
        // slider
        sldPosAbsolute->setMinValue((int)(min * _sliderGain));
        sldPosAbsolute->setMaxValue((int)(max * _sliderGain));
    }
}

void GuiControlboardAxis::lneMax_returnPressed(){
    QVariant varmin(lneMin->text());
    QVariant varmax(lneMax->text());
    if (_ilim != NULL){
    	//cout << "guiControlboard axis: " << _axisIndex << " setting limits to: " << varmin.asDouble() * _deg2rad << " " << varmax.asDouble() * _deg2rad << endl;  
        _ilim->setLimits(_axisIndex, varmin.asDouble() , varmax.asDouble() );
        
       // adjust slider
        double min = floor(varmin.asDouble() * pow( 10.0, 2.0) + 0.5) * pow(10.0, -2.0);
		double max = floor(varmax.asDouble() * pow( 10.0, 2.0) + 0.5) * pow(10.0, -2.0);
        varmin = min;
        varmax = max;
        // labels
        lblMin->setText(varmin.asString());
        lblMax->setText(varmax.asString());
        // slider
        sldPosAbsolute->setMinValue((int)(min * _sliderGain));
        sldPosAbsolute->setMaxValue((int)(max * _sliderGain));
    }
}
    
void GuiControlboardAxis::btnPlus_clicked(){
    if (_ipos != NULL){
        if(_ipos->relativeMove(_axisIndex, _dblStepSize)){
        
	        // set psoition to absolute position line edit
	        QVariant absPos(lnePosAbsolute->text());
	        double rounded = floor(_dblStepSize * pow( 10.0, 2.0) + 0.5) * pow(10.0,-2.0);
	        
	        int sldValue = (int)(rounded * _sliderGain + absPos.asDouble());
	        QVariant newPos(absPos.asDouble() + rounded);
	        lnePosAbsolute->setText(newPos.asString());
            _inhibitSliderAction = true;
            sldPosAbsolute->setValue((int)(newPos.asDouble() * _sliderGain));
            _inhibitSliderAction = false;
        }
    }
}
    
void GuiControlboardAxis::btnMinus_clicked(){
    if (_ipos != NULL){
        if(_ipos->relativeMove(_axisIndex, -_dblStepSize)){
        
        	// set position to absolute position line edit
	        QVariant absPos(lnePosAbsolute->text());
	        double rounded = floor(_dblStepSize * pow( 10.0, 2.0) + 0.5) * pow(10.0,-2.0);
	        QVariant newPos(absPos.asDouble() - rounded);
	        lnePosAbsolute->setText(newPos.asString());
            _inhibitSliderAction = true;
            sldPosAbsolute->setValue((int)(newPos.asDouble() * _sliderGain));
            _inhibitSliderAction = false;
        }
    }
}


void GuiControlboardAxis::lneCtrlD_returnPressed()
{
	if (_ipid != NULL){
		QVariant varp(lneCtrlP->text());
		QVariant vari(lneCtrlI->text());
		QVariant vard(lneCtrlD->text());
		yarp::dev::Pid pid;
		pid.setKp(varp.asDouble()); 
		pid.setKi(vari.asDouble()); 
		pid.setKd(vard.asDouble());
		//cout << "Setting pid to: " << pid.kp << " " << pid.ki << " " << pid.kd << endl;
		_ipid->setPid(_axisIndex, pid);
	}
}


void GuiControlboardAxis::lneCtrlI_returnPressed()
{
	if (_ipid != NULL){
		QVariant varp(lneCtrlP->text());
		QVariant vari(lneCtrlI->text());
		QVariant vard(lneCtrlD->text());
		yarp::dev::Pid pid;
		pid.setKp(varp.asDouble()); 
		pid.setKi(vari.asDouble()); 
		pid.setKd(vard.asDouble());
		//cout << "Setting pid to: " << pid.kp << " " << pid.ki << " " << pid.kd << endl;
		_ipid->setPid(_axisIndex, pid);
	}

}


void GuiControlboardAxis::lneCtrlP_returnPressed()
{
	if (_ipid != NULL){
		QVariant varp(lneCtrlP->text());
		QVariant vari(lneCtrlI->text());
		QVariant vard(lneCtrlD->text());
		yarp::dev::Pid pid;
		pid.setKp(varp.asDouble()); 
		pid.setKi(vari.asDouble()); 
		pid.setKd(vard.asDouble());
		//cout << "Setting pid to: " << pid.kp << " " << pid.ki << " " << pid.kd << endl;
		_ipid->setPid(_axisIndex, pid);
	}

}

void GuiControlboardAxis::btnStop_clicked()
{
	_ipos->stop(_axisIndex);
	_ivel->stop(_axisIndex);
}

// void GuiControlboardAxis::timerDone(){
// 	double dblVal;
// 	_ienc->getEncoder(_axisIndex, &dblVal);
// 	QVariant varVal((int)dblVal);
// 	lneEnc->setText(varVal.asString());
// }

