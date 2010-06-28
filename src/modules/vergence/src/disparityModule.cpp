
#include <iCub/disparityModule.h>
#include <ippi.h>
#include <ippcc.h>

using namespace yarp::math;

disparityModule::disparityModule(){
    	printf("initialization of the module \n");

	//create the imageIn ports
	imageInLeft.open( "/vergence/left:i" );
	imageInRight.open( "/vergence/right:i" );	
	cmdOutput.open("/vergence/command:o");
	targetPort.open("/vergence/target:o");

	ratio = 4.00;
	//Disp.init(84,ratio);
	
	robotHead = 0;
	robotTorso = 0;
	leftEye = new iCubEye("left");
	rightEye = new iCubEye("right");
	
	leftEye->releaseLink(0);rightEye->releaseLink(0);
	leftEye->releaseLink(1);rightEye->releaseLink(1);
	leftEye->releaseLink(2);rightEye->releaseLink(2);
	leftEye->releaseLink(3);rightEye->releaseLink(3);
	leftEye->releaseLink(4);rightEye->releaseLink(4);
	leftEye->releaseLink(5);rightEye->releaseLink(5);
	leftEye->releaseLink(6);rightEye->releaseLink(6);
	leftEye->releaseLink(7);rightEye->releaseLink(7);

	chainRightEye=rightEye->asChain();
  	chainLeftEye =leftEye->asChain();
	cout << "chainRightEye " << (*chainRightEye)[0].getAng() << endl;

	(*chainRightEye)[0].setAng( 1.0 );
	//}
	cout << "chainRightEye " << (*chainRightEye)[0].getAng() << endl;
	
	fprintf(stderr,"Left Eye kinematic parameters:\n");
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
  	}

	cout << "GET N: " << chainRightEye->getN() << endl;
	cout << "GET DOF: " << chainRightEye->getDOF() << endl;

/*	// define the links in standard D-H convention
	//A,  D, alpha, offset(*), min theta, max theta
	leftLink = new iKinLink( (*chainLeftEye)[7].getA(), (*chainLeftEye)[7].getD(), (*chainLeftEye)[7].getAlpha(), (*chainLeftEye)[7].getOffset(), 0, 0);
	rightLink = new iKinLink( (*chainRightEye)[7].getA(), (*chainRightEye)[7].getD(), (*chainRightEye)[7].getAlpha(), (*chainRightEye)[7].getOffset(), 0, 0);
*/

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

	imgOut=new ImageOf<PixelMono>;
	imgOut->resize(252,152);
}

disparityModule::~disparityModule(){

	delete leftEye;
	delete rightEye;
	delete robotTorso;
	delete robotHead;
}

bool disparityModule::open( Searchable& config ){
	printf("opening all the ports");

	imageOutputPort.open("/vergence/image:o");
	histoOutPort.open("/vergence/histo:o");

	printf("\n opening .....");
	fout = fopen("data/disp_data.txt", "wa");
	//fflush( stdout );

	

	optionsTorso.put("device", "remote_controlboard");
   	optionsTorso.put("local", "/local1");
   	optionsTorso.put("remote", "/icubSim/torso"); // CHANGE FOR THE ROBOT

   	robotTorso = new PolyDriver(optionsTorso);

   	if (!robotTorso->isValid()) {
     	printf("Cannot connect to robot torso\n");
     	return 1;
   	}
   	robotTorso->view(encTorso);
   	if ( encTorso==NULL) {
    	printf("Cannot get interface to robot torso\n");
     	robotTorso->close();
    	return 1;
   	}

	optionsHead.put("device", "remote_controlboard");
	optionsHead.put("local", "/local");
	optionsHead.put("remote", "/icubSim/head");

	robotHead = new PolyDriver (optionsHead);

	if (!robotHead->isValid()){
		printf("cannot connect to robot head\n");
		return 1;
	}
	robotHead->view(encHead);
	if (encHead == NULL) {
		printf("cannot get interface to the head\n");
		robotHead->close();
		return 1;
	}
	
	return true;
}

bool disparityModule::close(){
    	printf("closing all the ports");
	imageOutputPort.close();
	targetPort.close();
	cmdOutput.close();
	histoOutPort.close();
	printf("saving left \n");
	imageInLeft.close();
	printf("saving right \n");
	imageInRight.close();
	printf("fclosing.... \n");
    	if(fout)
        	fclose (fout);
	printf("stouting .... \n");
	//fflush( stdout );    
	return true;
}

bool disparityModule::interruptModule(){
	imageOutputPort.interrupt();
	histoOutPort.interrupt();
	imageInLeft.interrupt();
	imageInRight.interrupt();

	return true;
}

bool disparityModule::updateModule(){	
	
	
	
	needLeft =  ( imageInLeft.getInputCount() > 0 );
	needRight = ( imageInRight.getInputCount() > 0 );

	

	int disparityVal = 0;
	double corrVal = 0.0;
    	
	//create vector that contains head encoders
	Vector _head(6), _torso(3);
	if (encTorso->getEncoders(_torso.data()))
	    for (int i=0; i<3; i++)
        	fb[i]=_torso[2-i];    // reversed order
	
 	if (encHead->getEncoders(_head.data()))    		
		for (int i=0; i<6; i++)
        		fb[3+i]=_head[i];
	
	Vector q = (M_PI/180.0)*fb; // to radians

	q[7]=(M_PI/180.0)*(fb[7]+fb[8]/2);
	HL = leftEye->getH(q);
    	q[7]=(M_PI/180.0)*(fb[7]-fb[8]/2);
	HR = rightEye->getH(q);

	_nFrame = chainRightEye->getN(); //HL.rows();
	_joints.resize( _nFrame );

	Vector tempV;
	tempV.resize(3);
	// computes ray that intesects with image plane
	
	computeRay( KIN_RIGHT_PERI , tempV, _centerX, _centerY);

	// ------------------------------------------------------------------compute min 
	Vector tmpPos;
	tmpPos.resize(9);
	tmpPos = q;

	tmpPos(7)  = - ( _maxVerg + tmpPos(8) );
	computeDirect( tmpPos );
	int x, y;
	intersectRay( KIN_RIGHT_PERI, tempV, x, y );
	float min;
	min = _centerX - x;

	// ------------------------------------------------------------------compute max 
	tmpPos = q;
	tmpPos(7)  = - ( _minVerg + tmpPos(8) );
	computeDirect( tmpPos );
	intersectRay( KIN_RIGHT_PERI, tempV, x, y );
	float max;
	max = _centerX - x;
	
	//cout << "min: " << min << " max: " << max << " " << "fixation " << _fixationPoint.toString().c_str() << endl;
	
	int hWidth = 0;		
	int hHeight = 0;
	double _minDisp = Disp.shiftToDisparity(Disp.getLimitsMin());
	double _maxDisp = Disp.shiftToDisparity(Disp.getLimitsMax());

	if ( needLeft + needRight > 1)
	{
		_minDisp = 2.0*206.026*tan(fb[8]*M_PI/180.0 - _maxVerg) + 0.5;
		_maxDisp = 2.0*206.026*tan(fb[8]*M_PI/180.0 - _minVerg) - 0.5;
		Disp.setInhibition((int)_maxDisp, (int)_minDisp);

		imgInL = imageInLeft.read(true);
		imgInR = imageInRight.read(true);
		/*Limg.resize(imgInL->width()/ratio,imgInL->height()/ratio);
		Rimg.resize(imgInR->width()/ratio,imgInR->height()/ratio);
		Disp.downSample(*imgInL, Limg);
		Disp.downSample(*imgInR, Rimg);*/

		//send to port
		//Disp._shiftFunction[Disp._shifts[0]]
		//Calcolo la funzione di correlazione e le disp dei massimi e scrivo i risultati su file
		disparityVal = Disp.computeDisparityCorrRGBsum(*imgInR, *imgInL, 4);

		hWidth = Disp.getShiftLevels();
		hHeight = hWidth/2;	
		histo.resize(hWidth, hHeight);

		for (int k = 0; k < 4; k++)
			maxes[k] = Disp.getMax(k);

		printf( "\n\tMax Shift:\n\t\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", maxes[0].index, (int)maxes[0].disp, maxes[0].corr);
		printf( "\n\tSecond Max:\n\t\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", maxes[1].index,(int) maxes[1].disp, maxes[1].corr);
		printf( "\n\tThird Max:\n\t\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", maxes[2].index, (int)maxes[2].disp, maxes[2].corr);
		printf( "\n\tZero Shift:\n\t\tdisp index: %d\tdisp value: %d\tcorr value: %f\n\n\n", maxes[3].index, (int)maxes[3].disp, maxes[3].corr);
		
		//cout << "disparity Val " << disparityVal << " Corrval " << corrVal << " DISPARITY " << Disp._shiftFunction[Disp._shifts[0]] << endl;
		cout << "disparity Val " << disparityVal  << endl;

		Disp.makeHistogram(histo);
		if(histoOutPort.getOutputCount()>0){
			cout<< "histo out"<<endl;
			histoOutPort.prepare() = histo;	
			histoOutPort.write();
		}
		
		
		//calculating the fusion map of the input images (@shift, @preShift, @afterShift )	
		merge(imgInL, imgInR, imgOut,disparityVal);
		if ( imageOutputPort.getOutputCount() > 0 ){ 		  
			imageOutputPort.prepare() = *imgOut;	
			imageOutputPort.write();
		}


		/*
		//vergence control from disparity calculations\
					theta = -K * di 
		//theta -> first derivative of the vergence angle\
		K     ->constant gain\
		di    ->disparity index	

		double K  = 0.25;
		double theta = -K * disparityVal;


		double angle  = fb[8] + theta;/*q[8]
		Bottle& b = cmdOutput.prepare(); 
 		char verg[100];
		sprintf(verg, "Vergence (position %lf)", angle);
		
		
		//cout << verg << endl;
		b.addString(verg);
		//b.addString(verg);
		//b.addDouble(angle);
		//b.addString(verg2);
		cmdOutput.write();
		b.clear();

		cout << "theta " << theta << " angle " << angle <<" current " << fb[8] << endl;

		*/

		// calculating the angolar displacement for the controller
		double angle=fb[8]-(180/M_PI)*atan(disparityVal/(2*206.026));
		if(angle<0)
			angle=0;		
		cout << "2 atan " <<(180/M_PI)*atan(disparityVal/(2*206.026))<< " angle " << angle <<" current " << fb[8] << endl;
		

		//sending position to the controller of the vergence
		Bottle in,bot;
        	//Bottle &bot = triangulationPort.prepare(); 
        	bot.clear();

		//Bottle& b = cmdOutput.prepare(); 
 		char verg[100];
		sprintf(verg, "set pos 5 %f", angle);
		
//		bot.addString(verg);
		bot.addString("set");
		bot.addString("pos");		
		bot.addInt(5);
		bot.addDouble(angle);
		cmdOutput.write(bot,in);
		bot.clear();


		//sending the salient position
		if ( targetPort.getOutputCount() > 0 ){ 		  
		  Bottle targetBottle;
		  targetBottle.addDouble(10*tan(6.28-angle)+0.1);
		  targetBottle.addDouble(10);
		  targetBottle.addDouble(10);
		  targetPort.prepare() = targetBottle;	
		  targetPort.write();
		}
		
	}
    
	fflush( stdout );
	return true;
} 	


void disparityModule::merge(ImageOf<PixelRgb> *imgInL,ImageOf<PixelRgb> *imgInR,ImageOf<PixelMono> *imgOut, int disparityVal){
	//initialisation
	IppiSize sizesrc={252,152};
	Ipp32f coeffs[3]={0.33,0.33,0.33};
	ImageOf<PixelMono>* imgInLGray=new ImageOf<PixelMono>;
	ImageOf<PixelMono>* imgInRGray=new ImageOf<PixelMono>;
	ImageOf<PixelMono>* imgInRGrayShift=new ImageOf<PixelMono>;
	imgInLGray->resize(252,152);
	imgInRGray->resize(252,152);
	imgInRGrayShift->resize(252,152);

	// copying the input image into grayscale image
	ippiColorToGray_8u_C3C1R(imgInL->getRawImage(),imgInL->getRowSize(),imgInLGray->getRawImage(),imgInLGray->getRowSize(), sizesrc, coeffs);
	ippiColorToGray_8u_C3C1R(imgInR->getRawImage(),imgInR->getRowSize(),imgInRGray->getRawImage(),imgInRGray->getRowSize(), sizesrc, coeffs);
	ippiCopy_8u_C1R(imgInRGray->getRawImage(),imgInRGray->getRowSize(), imgOut->getRawImage(),imgOut->getRowSize(),sizesrc);
	// shift the image
	unsigned char *p_right=(unsigned char *)imgInRGray->getRawImage();
	unsigned char *p_rightShift=(unsigned char *)imgInRGrayShift->getRawImage();
	
	for (int r=0;r<152;r++){
	  int disparity=0;
	  for(int c=0; c<252; c++){	    
	    if(disparityVal>0){
	      //cout<<"disparityVal>0"<<endl;
	      if(disparity>=disparityVal){
		*p_rightShift=*p_right;
		p_right++;
	      }
	      else{
		*p_rightShift=0;
	      }
	      p_rightShift++;
	      disparity++; 
	    }
	    else{
	      if(disparity>=-disparityVal){
		//cout<<" *"<<endl;
		*p_rightShift=*p_right;
		p_rightShift++;
	      } 
	      
	      disparity++;
	      p_right++;
	    }	    
	  }

	  
	  //alignment
	  for(int i=252-abs(disparityVal);i<252;i++){
	    if(disparityVal>0){
	      
	      p_right++;
	    }
	    else{
	      *p_rightShift=0;
	      p_rightShift++;	         
	    }
	  }
       
	  
	  
      	  //padding
	  for(int i=252;i< imgInLGray->getRowSize();i++){
	    //printf(".");	    
	    p_rightShift++;	    
	    p_right++;	         
	  }
	  
	}

	
	// fuse the two images
	unsigned char *p_left=(unsigned char *)imgInLGray->getRawImage();
	p_right=(unsigned char *)imgInRGrayShift->getRawImage();
	unsigned char *p_out=(unsigned char *)imgOut->getRawImage();
	for (int r=0;r<152;r++){
	  for(int c=0; c<252; c++){
	    //p_out[c+r*152]=((p_left[c+r*152]+p_right[c+r*152])/512)*255;
	    unsigned char value;
	    double subvalue=(double)(*p_left+*p_right)/512;
	    value=(unsigned char)(subvalue*255);
	    //value=(unsigned char)((*p_right+*p_left)/512)*255;
	    *p_out=value;
	    p_right++;
	    p_left++;
	    p_out++;	   
	  }
	  //padding
	  for(int i=252;i< imgInLGray->getRowSize();i++){
	    p_right++;
	    p_left++;
	    p_out++;	   
	  }
	}
	
	
	delete imgInLGray;
	delete imgInRGray;
	delete imgInRGrayShift;

}


void disparityModule::computeRay(__kinType k, Vector& v, int x, int y){
	
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

/// given an up to date kin matrix, it computes the x,y point where a given ray v intersects the img plane.
void disparityModule::intersectRay (__kinType k, const Vector& v, int& x, int& y)
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

void disparityModule::computeDirect (const Vector &joints)
{

	// the joint vector is devided into right and left
	// I *KNOW* this is awful because doesn't use n_ref_frame
	// we don't need joints copy 'cause we don't use computeDirect(Joints)


	_leftJoints(0) = joints(0);
	_leftJoints(1) = joints(1);
	_leftJoints(2) = joints(2);
	_leftJoints(3) = joints(3);
	_leftJoints(4) = joints(4);
	_leftJoints(5) = joints(5);
	_leftJoints(6) = joints(6);
	_leftJoints(7) = joints(8);


	_rightJoints(0) = joints(0);
	_rightJoints(1) = joints(1);
	_rightJoints(2) = joints(2);
	_rightJoints(3) = joints(3);
	_rightJoints(4) = joints(4);
	_rightJoints(5) = joints(5);
	_rightJoints(6) = joints(6);
	_rightJoints(7) = joints(7);
	
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
void disparityModule::computeFixation (const Matrix &T1, const Matrix &T2)
{
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



