/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/gui/ControlboardAxis.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::gui;

ControlboardAxis::ControlboardAxis(yarp::dev::PolyDriver *dd, int axisIndex, QWidget* parent, const char* name, bool modal, WFlags fl)
	: ControlboardAxisBase( parent, name, fl )
{
            
    _sliderGain = 100.0;
    
    _dd = dd;
    _ipos = NULL;
    _ivel = NULL;
    _ienc = NULL;
    _ilim = NULL;
    _ipid = NULL;
	_iamp = NULL;
	
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
	if (!(dd->view(_iamp)))
    	cout << "device does not implement IAmplifierControl!" << endl;
    
    _axisIndex = axisIndex;
    QVariant varIndex(_axisIndex);
    gbxAxis->setTitle("Axis: " + varIndex.asString());
    
	// enable amplifier
	if(_iamp != NULL){
		//_iamp->enableAmp(_axisIndex);
		chbAmp->setChecked(true);
	}

    // position interface init
    if (_ipos != NULL){
        _ipos->setPositionMode();
		double acc, vel;
        _ipos->setRefSpeed(_axisIndex,100); 
		_ipos->setRefAcceleration(_axisIndex, 100);
		_ipos->getRefAcceleration(_axisIndex, &acc);
		_ipos->getRefSpeed(_axisIndex, &vel);
		acc = floor(acc * pow( 10.0, 2.0) + 0.5) * pow(10.0, -2.0);
		vel = floor(vel * pow( 10.0, 2.0) + 0.5) * pow(10.0, -2.0);
        QVariant varAcc(acc);
        QVariant varVel(vel);
        lneCtrlAcc->setText(varAcc.asString());
        lneCtrlVel->setText(varVel.asString());
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
        // limit line edits
        lneMin->setText(varmin.asString());
        lneMax->setText(varmax.asString());
        // slider
        sldPosAbsolute->setMinValue((int)(min * _sliderGain));
        sldPosAbsolute->setMaxValue((int)(max * _sliderGain));
    }
    
    if (_ipid != NULL){    	
    	double p,i,d;
    	p = 0.0;
		i = 0.0;
		d = 0.0;
		// some problem with jrkerr driver on windows VS9 (so, we don't get the values)
		/*yarp::dev::Pid pid;
    	_ipid->getPid(_axisIndex, &pid);
    	p = floor(pid.kp * pow( 10.0, 4.0) + 0.5) * pow(10.0, -4.0);
    	i = floor(pid.ki * pow( 10.0, 4.0) + 0.5) * pow(10.0, -4.0);
    	d = floor(pid.kd * pow( 10.0, 4.0) + 0.5) * pow(10.0, -4.0);*/
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
        sldPosAbsolute->setValue((int)(enc*_sliderGain));
        QVariant varenc(enc);
        // line edit position
        lnePosAbsolute->setText(varenc.toString());
     }
    
    // defaults
    _dblStepSize = 1.0;
    
    // line edit validators
    lneSpeed->setValidator(new QDoubleValidator(this));
    lnePosAbsolute->setValidator(new QDoubleValidator(this));
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

ControlboardAxis::~ControlboardAxis()
{
	//_timer->stop();
	//delete _timer;
}



void ControlboardAxis::sldPosAbsolute_valueChanged( int val ){   

	QVariant varVal(((double)val)/_sliderGain);
	lnePosAbsolute->setText(varVal.asString());
}

void ControlboardAxis::sldPosAbsolute_sliderReleased(){
	if (_ipos != NULL){
		int val = sldPosAbsolute->value();
	   _ipos->positionMove(_axisIndex, (((double)val)/_sliderGain));
    }
}


void ControlboardAxis::rbtPos_toggled( bool flag)
{
	if (flag){
            if (_ipos != NULL){
                    //cout << "position mode set" << endl;
                    _ipos->setPositionMode();
                    lnePosAbsolute->setEnabled(true);
                    sldPosAbsolute->setEnabled(true);
                    btnPlus->setEnabled(true);
                    btnMinus->setEnabled(true);
                    lneSpeed->setEnabled(false);
            }
	}
}


void ControlboardAxis::rbtVel_toggled( bool flag)
{
	if (flag){
        if (_ivel != NULL){
            //cout << "velocity mode set" << endl;
            _ivel->setVelocityMode();
            lnePosAbsolute->setEnabled(false);
            sldPosAbsolute->setEnabled(false);
            btnPlus->setEnabled(false);
            btnMinus->setEnabled(false);
            lneSpeed->setEnabled(true);
        }
	}
}


void ControlboardAxis::lnePosAbsolute_returnPressed()
{
	QVariant pos(lnePosAbsolute->text());
    if (_ipos != NULL){
        _ipos->positionMove(_axisIndex, pos.asDouble());
        sldPosAbsolute->setValue((int)(pos.asDouble() * _sliderGain));
    }
}


void ControlboardAxis::lneSpeed_returnPressed()
{
	QVariant speed(lneSpeed->text());
    if (_ivel != NULL){
	   _ivel->velocityMove(_axisIndex, speed.asDouble());
    }
}

void ControlboardAxis::lneMin_returnPressed(){
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
        // slider
        sldPosAbsolute->setMinValue((int)(min * _sliderGain));
        sldPosAbsolute->setMaxValue((int)(max * _sliderGain));
    }
}

void ControlboardAxis::lneMax_returnPressed(){
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
        // slider
        sldPosAbsolute->setMinValue((int)(min * _sliderGain));
        sldPosAbsolute->setMaxValue((int)(max * _sliderGain));
    }
}
    
void ControlboardAxis::btnPlus_clicked(){
    if (_ipos != NULL){
        if(_ipos->relativeMove(_axisIndex, _dblStepSize)){
        
	        // set psoition to absolute position line edit
	        QVariant absPos(lnePosAbsolute->text());
	        double rounded = floor(_dblStepSize * pow( 10.0, 2.0) + 0.5) * pow(10.0,-2.0);
	        
	        int sldValue = (int)(rounded * _sliderGain + absPos.asDouble());
	        QVariant newPos(absPos.asDouble() + rounded);
	        lnePosAbsolute->setText(newPos.asString());
            sldPosAbsolute->setValue((int)(newPos.asDouble() * _sliderGain));
        }
    }
}
    
void ControlboardAxis::btnMinus_clicked(){
    if (_ipos != NULL){
        if(_ipos->relativeMove(_axisIndex, -_dblStepSize)){
        
        	// set position to absolute position line edit
	        QVariant absPos(lnePosAbsolute->text());
	        double rounded = floor(_dblStepSize * pow( 10.0, 2.0) + 0.5) * pow(10.0,-2.0);
	        QVariant newPos(absPos.asDouble() - rounded);
	        lnePosAbsolute->setText(newPos.asString());
            sldPosAbsolute->setValue((int)(newPos.asDouble() * _sliderGain));
        }
    }
}


void ControlboardAxis::lneCtrlD_returnPressed()
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


void ControlboardAxis::lneCtrlI_returnPressed()
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


void ControlboardAxis::lneCtrlP_returnPressed()
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



void ControlboardAxis::lneCtrlVel_returnPressed(){
	if(_ipos != NULL){
		QVariant varVel(lneCtrlVel->text());
		_ipos->setRefSpeed(_axisIndex, varVel.asDouble());
	}
}


void ControlboardAxis::lneCtrlAcc_returnPressed(){
	if(_ipos != NULL){
		QVariant varAcc(lneCtrlAcc->text());
		_ipos->setRefAcceleration(_axisIndex, varAcc.asDouble());
	}
}


void ControlboardAxis::chbAmp_toggled(bool on){
	if(_iamp != NULL){
		if(on){
			_iamp->enableAmp(_axisIndex);
		}
		else{
			_iamp->disableAmp(_axisIndex);
		}
	}
}

void ControlboardAxis::btnStop_clicked(){
	if(_ipos != NULL){
		_ipos->stop(_axisIndex);
	}
	if(_ivel != NULL){
		_ivel->stop(_axisIndex);
	}
}

// void ControlboardAxis::timerDone(){
// 	double dblVal;
// 	_ienc->getEncoder(_axisIndex, &dblVal);
// 	QVariant varVal((int)dblVal);
// 	lneEnc->setText(varVal.asString());
// }

