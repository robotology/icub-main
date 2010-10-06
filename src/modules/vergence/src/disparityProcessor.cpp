// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#include <iCub/disparityProcessor.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace yarp::math;
const int THREAD_RATE = 100;

disparityProcessor::disparityProcessor():RateThread(THREAD_RATE){
    cout<< "initialisation process "<<endl;
	ratio = 4.00;
	
	robotHead = 0;
	robotTorso = 0;
    imgInL=0;
    imgInR=0;

	fb.resize(9);
	fb = 0;
	_q.resize(3);
	_it.resize(3);
	_o.resize(3);
	_epx.resize(3);
	_tmp.resize(3);
	_tmpEl.resize(3);

	_fixationPoint.resize(3);
	_fixationPolar.resize(3);

	_leftJoints.resize(9);
	_rightJoints.resize(9);
    dispInit = false;
}

disparityProcessor::~disparityProcessor(){

}


void disparityProcessor::setName(string name, string robotName) {
    this->moduleName = name;
    this->robotName = robotName;
}

bool disparityProcessor::threadInit(){


    Property optGaze("(device gazecontrollerclient)");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/gaze_client");

    clientGaze=new PolyDriver;
    if (!clientGaze->open(optGaze))
    {
        delete clientGaze;    
        return false;
    }

    // open the view
    clientGaze->view(igaze);


    string torsoPort, headPort;
  
    torsoPort = "/" + robotName + "/torso";
    headPort = "/" + robotName + "/head";

	optionsTorso.put("device", "remote_controlboard");
   	optionsTorso.put("local", "/localTorso");
   	optionsTorso.put("remote", torsoPort.c_str() );

   	robotTorso = new PolyDriver(optionsTorso);

   	if (!robotTorso->isValid()) {
     	printf("Cannot connect to robot torso\n");
   	}
   	robotTorso->view(encTorso);
   	if ( encTorso==NULL) {
    	printf("Cannot get interface to robot torso\n");
     	robotTorso->close();
   	}

	optionsHead.put("device", "remote_controlboard");
	optionsHead.put("local", "/localhead");
	optionsHead.put("remote", headPort.c_str());

	robotHead = new PolyDriver (optionsHead);

	if (!robotHead->isValid()){
		printf("cannot connect to robot head\n");
	}
	robotHead->view(encHead);
	if (encHead == NULL) {
		printf("cannot get interface to the head\n");
		robotHead->close();
	}

    string imageInLeftName = moduleName+"/left:i";
    imageInLeft.open(imageInLeftName.c_str());

    string imageInRightName = moduleName+"/right:i";
    imageInRight.open(imageInRightName.c_str());

    string imageHistoName = moduleName+"/histo:o";
    histoOutPort.open(imageHistoName.c_str());

    string outputCommand = moduleName+"/cmd:o";
    cmdOutput.open(outputCommand.c_str());

    string shiftCommand = moduleName+"/shift:o";
    shiftOutput.open(shiftCommand.c_str());

    rightEye = new iCubEye("right");
    leftEye = new iCubEye("left");
	
    for (int i = 0; i < 8; i++ ){
        leftEye->releaseLink(i);
        rightEye->releaseLink(i);
    }
  
	chainRightEye=rightEye->asChain();
  	chainLeftEye =leftEye->asChain();
    
	//cout << "chainRightEye " << (*chainRightEye)[0].getAng() << endl;
	(*chainRightEye)[0].setAng( 1.0 );
	//cout << "chainRightEye " << (*chainRightEye)[0].getAng() << endl;
	
    
	/*fprintf(stderr,"Left Eye kinematic parameters:\n");
  	for (unsigned int i=0; i<chainLeftEye->getN(); i++){
    	fprintf(stderr,"#%d: %g, %g, %g, %g\n",i,
        (*chainLeftEye)[i].getA(),
        (*chainLeftEye)[i].getD(),
        (*chainLeftEye)[i].getAlpha(),
        (*chainLeftEye)[i].getOffset());
  	}
	
	fprintf(stderr,"Right Eye kinematic parameters:\n");
  	for (unsigned int i=0; i<chainRightEye->getN(); i++){
    	fprintf(stderr,"#%d: %g, %g, %g, %g\n",i,
    	(*chainRightEye)[i].getA(),
        (*chainRightEye)[i].getD(),
        (*chainRightEye)[i].getAlpha(),
        (*chainRightEye)[i].getOffset());
  	}*/
     
    _nFrame = chainRightEye->getN(); //HL.rows();
    _joints.resize( _nFrame );
    _head.resize(6); 
    _torso.resize(3);
    gazeVect.resize(3);
    tmpPos.resize(9);
    tempV.zero();
    tempV.resize(3);
	
	return true;
}

void disparityProcessor::threadRelease(){
	histoOutPort.close();
	imageInLeft.close();
	imageInRight.close();
    cmdOutput.close();
    delete leftEye;
    delete rightEye;   
	delete robotHead;
	delete robotTorso;
    delete clientGaze;
}

void disparityProcessor::onStop() 
{
    histoOutPort.close();
	imageInLeft.close();
	imageInRight.close();
    cmdOutput.close();
    delete leftEye;
    delete rightEye;   
	delete robotHead;
	delete robotTorso;
    delete clientGaze;
}

void disparityProcessor::run(){	
   
    int disparityVal = 0;
    double corrVal = 0.0;

    //create vector thast contains head encoders
    
        for (int i=0; i<3; i++)
            fb[i]=_torso[2-i];    // reversed order

    if (encHead->getEncoders(_head.data()))
        for (int i=0; i<6; i++)
            fb[3+i]=_head[i];

    Vector q = (M_PI/180.0)*fb; // to radians

    q[7]=(M_PI/180.0)*(fb[7]+fb[8]/2);
    //HL = leftEye->getH(q);
    q[7]=(M_PI/180.0)*(fb[7]-fb[8]/2);
    //HR = rightEye->getH(q);

    // computes ray that intesects with image plane
    computeRay( KIN_RIGHT_PERI , tempV, _centerX, _centerY);

    //------------------------------------------------------------------compute min 
    tmpPos = q;

    tmpPos(7)  = - ( _maxVerg + tmpPos(8) );
    computeDirect( tmpPos );
    int x, y;
    intersectRay( KIN_RIGHT_PERI, tempV, x, y );
    float min;
    min = (float)(_centerX - x);

    // ------------------------------------------------------------------compute max 
    tmpPos = q;
    tmpPos(7)  = - ( _minVerg + tmpPos(8) );
    computeDirect( tmpPos );
    intersectRay( KIN_RIGHT_PERI, tempV, x, y );
    float max;
    max = (float)(_centerX - x);

    int hWidth = 0;		
    int hHeight = 0;
    double _minDisp = Disp.shiftToDisparity(Disp.getLimitsMin());
    double _maxDisp = Disp.shiftToDisparity(Disp.getLimitsMax());

    _minDisp = 2.0 * 206.026 * tan( fb[8] * M_PI/180.0 - _maxVerg ) + 0.5; 
    _maxDisp = 2.0 * 206.026 * tan( fb[8] * M_PI/180.0 - _minVerg ) - 0.5;

    Disp.setInhibition((int)_maxDisp, (int)_minDisp);

    needLeft =  ( imageInLeft.getInputCount() > 0 );
    needRight = ( imageInRight.getInputCount() > 0 );

    if( needLeft + needRight > 1 ) {

        imgInL = imageInLeft.read(false);
		imgInR = imageInRight.read(false);

        if( ( imgInL != NULL ) && ( imgInR != NULL ) ) {

            if( !dispInit ) {
                Disp.setSize( imgInR );
                dispInit = true;            
            }
            disparityVal = Disp.computeDisparityCorrRGBsum(*imgInR, *imgInL, 4);
            //disparityVal = Disp.computeMono(*imgInR, *imgInL, 4.0);

            hWidth = Disp.getShiftLevels();
            hHeight = hWidth/2;
            histo.resize(hWidth,hHeight);

           // cout << "disparity Val " << disparityVal  << endl;

            if ( histoOutPort.getOutputCount() > 0 ) { 
                Disp.makeHistogram(histo);
                histoOutPort.prepare() = histo;	
                histoOutPort.write();
            }

           angle = fb[8]-(180/M_PI)*atan(disparityVal/(2*206.026));
            if(angle<0)
                angle=0;		

           encHead->getEncoders( _head.data() );
           double relangle = 0;
           relangle = angle - _head[5];

          // cout << "2 atan " <<(180/M_PI)*atan(disparityVal/(2*206.026))<< " angle " << angle <<" current " << fb[8] << " " << relangle << endl;

           if ( cmdOutput.getOutputCount() > 0 ) { 
                Bottle in,bot;
                bot.clear();
                char verg[100];
                sprintf(verg, "set pos 5 %f", angle);
                bot.addString("set");
                bot.addString("pos");		
                bot.addInt(5);
                bot.addDouble(angle);
                cmdOutput.write(bot,in);
                bot.clear();
            }
            
            gazeVect[0] = 0.0;
            gazeVect[1] = 0.0;
            gazeVect[2] = relangle;
            igaze->lookAtRelAngles(gazeVect);

            //cout << "relangle " << relangle << endl;

            //if (relangle < 0.01)
            //suspend();
            
            //send shifts on shift port
            shift_Struct test;
            test = Disp.getMax();
            int pixels = 0;
            pixels = (180 - test.index) * (252/180);

            if (shiftOutput.getOutputCount()>0){
                Bottle bot;
                bot.clear();
                bot.addInt(pixels);
                shiftOutput.write(bot);
                bot.clear();
            }
        }
    }
} 	

void disparityProcessor::suspend(){

    cout << endl;
    cout << "Vergence has been suspended!" << endl;
    cout << endl;

    RateThread::suspend();
}

void disparityProcessor::release(){

    cout << endl;
    cout << "Vergence has been resumed!" << endl;
    cout << endl;

    RateThread::resume();
}

void disparityProcessor::computeRay(__kinType k, Vector& v, int x, int y){
	if (k == KIN_LEFT)
	{
		x -= CenterFoveaX;
		y -= CenterFoveaY;

		const Matrix &ep = (*chainLeftEye).getH();

		/// pixels -> mm
		double dx = double(x) / PixScaleX;
		double dy = double(y) / PixScaleY;

		v(0) = F * ep (0, 0) - dx * ep (0, 1) - dy * ep (0, 2);
		v(1) = F * ep (1, 0) - dx * ep (1, 1) - dy * ep (1, 2);
		v(2) = F * ep (2, 0) - dx * ep (2, 1) - dy * ep (2, 2);

		v = yarp::math::operator* (v, 1.0/sqrt( dot (v,v) ) );
	}
	else
	if (k == KIN_LEFT_PERI)
	{
		x -= CenterPeripheryX;
		y -= CenterPeripheryY;

		int rx, ry;
		peripheryToFovea (x, y, rx, ry);

		const Matrix &ep = (*chainLeftEye).getH();

		/// pixels -> mm
		double dx = double(rx) / PixScaleX;
		double dy = double(ry) / PixScaleY;

		v(0) = F * ep (0, 0) - dx * ep (0, 1) - dy * ep (0, 2);
		v(1) = F * ep (1, 0) - dx * ep (1, 1) - dy * ep (1, 2);
		v(2) = F * ep (2, 0) - dx * ep (2, 1) - dy * ep (2, 2);
		
		v = yarp::math::operator* (v, 1.0/sqrt( dot (v,v) ) );
	}
	else
	if (k == KIN_RIGHT)
	{
		x -= CenterFoveaX;
		y -= CenterFoveaY;

		const Matrix &ep = (*chainRightEye).getH();

		/// pixels -> mm
		double dx = double(x) / PixScaleX;
		double dy = double(y) / PixScaleY;

		v(0) = F * ep (0, 0) - dx * ep (0, 1) - dy * ep (0, 2);
		v(1) = F * ep (1, 0) - dx * ep (1, 1) - dy * ep (1, 2);
		v(2) = F * ep (2, 0) - dx * ep (2, 1) - dy * ep (2, 2);

		v = yarp::math::operator* (v, 1.0/sqrt( dot (v,v) ) );
	}
	else
	if (k == KIN_RIGHT_PERI)
	{
		x -= CenterPeripheryX;
		y -= CenterPeripheryY; 

		int rx, ry;
		peripheryToFovea (x, y, rx, ry);

		const Matrix &ep = (*chainRightEye).getH();

		/// pixels -> mm
		double dx = double(rx) / PixScaleX;
		double dy = double(ry) / PixScaleY;

		v(0) = F * ep (0, 0) - dx * ep (0, 1) - dy * ep (0, 2);
		v(1) = F * ep (1, 0) - dx * ep (1, 1) - dy * ep (1, 2);
		v(2) = F * ep (2, 0) - dx * ep (2, 1) - dy * ep (2, 2);

		v = yarp::math::operator* (v, 1.0/sqrt( dot (v,v) ) );
	}
	else
		v = 0;
}

// given an up to date kin matrix, it computes the x,y point where a given ray v intersects the img plane.
void disparityProcessor::intersectRay (__kinType k, const Vector& v, int& x, int& y)
{
	if (k == KIN_LEFT || k == KIN_LEFT_PERI)
	{
		Vector &q = _q;
		Vector &it= _it;
		const Matrix &ep = (*chainLeftEye).getH();

		Vector &o = _o;
		Vector &epx = _epx;

		o(0) = ep(0,3);
		o(1) = ep(1,3);
		o(2) = ep(2,3);
		epx(0) = ep(0,0);
		epx(1) = ep(1,0);
		epx(2) = ep(2,0);

		q = o + F * epx;

		/// intersect plane w/ old ray v.
		/// normal vector to plane is ep(1/2/3, 1)
		double t = F / (ep(0,0)*v(0) + ep(1,0)*v(1) + ep(2,0)*v(2));
		it = v * t + o - q;

		Vector &tmp = _tmp;;
		///tmp(1) = ep(1,1) * it(1) + ep(2,1) * it(2) + ep(3,1) * it(3);
		tmp(1) = ep(0,1) * it(0) + ep(1,1) * it(1) + ep(2,1) * it(2);
		tmp(2) = ep(0,2) * it(0) + ep(1,2) * it(1) + ep(2,2) * it(2);

		/// mm -> pixels
		x = int (-tmp(1) * PixScaleX + .5);
		y = int (-tmp(2) * PixScaleY + .5);

		if (k == KIN_LEFT_PERI)
		{
			int rx = x, ry = y;
			foveaToPeriphery (rx, ry, x, y);

			x += CenterPeripheryX;
			y += CenterPeripheryY;
		}
		else
		{
			x += CenterFoveaX;
			y += CenterFoveaY;
		}
	}
	else
	if (k == KIN_RIGHT || k == KIN_RIGHT_PERI)
	{
		Vector &q = _q;
		Vector &it = _it;
		const Matrix &ep = (*chainRightEye).getH();

		Vector &o = _o;
		Vector &epx = _epx;

		o(0) = ep(0,3);
		o(1) = ep(1,3);
		o(2) = ep(2,3);
		epx(0) = ep(0,0);
		epx(1) = ep(1,0);
		epx(2) = ep(2,0);

		q = o + F * epx;

		double t = F / ( ep(0,0)*v(0) + ep(1,0)*v(1) + ep(2,0)*v(2));
		
		it = v * t + o - q;

		Vector &tmp = _tmp;
		//tmp(0) = ep(0,0) * it(0) + ep(1,0) * it(1) + ep(2,0) * it(2); //
		tmp(1) = ep(0,1) * it(0) + ep(1,1) * it(1) + ep(2,1) * it(2);
		tmp(2) = ep(0,2) * it(0) + ep(1,2) * it(1) + ep(2,2) * it(2);

		//cout << "tmp(n) " << tmp(1) << " " << tmp(2) << endl;

		/// mm -> pixels
		x = int (-tmp(1) * PixScaleX + 0.5);
		y = int (-tmp(2) * PixScaleY + 0.5);

		if (k == KIN_RIGHT_PERI)
		{
			int rx = x, ry = y;
			foveaToPeriphery (rx, ry, x, y);
			x += CenterPeripheryX;
			y += CenterPeripheryY;
		}
		else
		{
			x += CenterFoveaX;
			y += CenterFoveaY;
		}
	}
	else
	{
		x = y = 0;
	}

	
}

void disparityProcessor::computeDirect (const Vector &joints)
{
	// the joint vector is devided into right and left
	// I *KNOW* this is awful because doesn't use n_ref_frame
	// we don't need joints copy because we don't use computeDirect(Joints)

    for (int i = 0; i < 9; i++ ){
        _leftJoints(i) = joints(i);
        _rightJoints(i) = joints(i);
    }
	
	//cout << "rightJoints " << _rightJoints.toString().c_str() << endl;
	(*chainLeftEye).setAng( _leftJoints );
	(*chainRightEye).setAng( _rightJoints );

	//compute the new fixation point starting from the the reference frames of the left and right camera
	Matrix RE = rightEye->getH();
	Matrix LE = leftEye->getH();	
	computeFixation ( RE, LE );
}

// check out 'cause I have modified the index numeration (Fra)
// as input two roto-traslation matrices 4x4
void disparityProcessor::computeFixation (const Matrix &T1, const Matrix &T2) {

	double tmp1;
	double tmp2;
	double tmp3;

	double u;
	
	if (T2(0,0) != 0)
		tmp1 = T2(1,3)-T1(1,3)-(T2(1,0)/T2(0,0))*(T2(0,3)-T1(0,3));
	else
		tmp1 = 0;
	
	tmp3 = T2(1,0)*T1(0,0)-T1(1,0)*T2(0,0);

	if (tmp3 != 0)
		tmp2 = -T2(0,0)/tmp3;
	else
		tmp2 = 0;
	
	u = tmp2*tmp1;

	_fixationPoint(0) = T1(0,3) + T1(0,0) * u;
	_fixationPoint(1) = T1(1,3) + T1(1,0) * u;
	_fixationPoint(2) = T1(2,3) + T1(2,0) * u;

	_cartesianToPolar(_fixationPoint, _fixationPolar);	
}
//left blank