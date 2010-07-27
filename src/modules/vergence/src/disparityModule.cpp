
#include <iCub/disparityModule.h>

using namespace yarp::math;

disparityModule::disparityModule(){
    init_flag=false;
    currentProcessor=0;
    
	ratio = 4.00;
	//Disp.init(84,ratio);
	
	
    robotHead = 0;
	robotTorso = 0;


	/*leftEye = new iCubEye("left");
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
  	chainLeftEye =leftEye->asChain();*/

	
    //cout << "chainRightEye " << (*chainRightEye)[0].getAng() << endl;

    /*
	(*chainRightEye)[0].setAng( 1.0 );
	cout << "chainRightEye " << (*chainRightEye)[0].getAng() << endl;
    */

    /*
	
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

    */
    

/*	// define the links in standard D-H convention
	//A,  D, alpha, offset(*), min theta, max theta
	leftLink = new iKinLink( (*chainLeftEye)[7].getA(), (*chainLeftEye)[7].getD(), (*chainLeftEye)[7].getAlpha(), (*chainLeftEye)[7].getOffset(), 0, 0);
	rightLink = new iKinLink( (*chainRightEye)[7].getA(), (*chainRightEye)[7].getD(), (*chainRightEye)[7].getAlpha(), (*chainRightEye)[7].getOffset(), 0, 0);
*/

    /*
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
    */
}

disparityModule::~disparityModule(){

	/*delete leftEye;
	delete rightEye;
	delete robotTorso;
	delete robotHead;*/
}


bool disparityModule::configure(ResourceFinder &rf) {
    //create the imageIn ports
	imageInLeft.open(getName( "/left" ));
	imageInRight.open(getName( "/right" ));	
	cmdOutput.open(getName("/output"));
	imageOutputPort.open(getName("/image:o"));
	histoOutPort.open(getName("/histo:o"));

    Time::turboBoost();
    cmdPort.open(getName("/cmd:i"));
    attach(cmdPort);

    /*interThread=new interactionThread();
    interThread->setName(this->getName().c_str());
    printf("name:%s \n",this->getName().c_str());
    interThread->start();*/

    printf("\n waiting for connection of the input port \n");

	//fout = fopen("disp_data.txt", "wa");
	//fflush( stdout );

	/*optionsTorso.put("device", "remote_controlboard");
   	optionsTorso.put("local", "/local1");
   	optionsTorso.put("remote", "/icubSim/torso"); 

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

    leftEye = new iCubEye("left");
	rightEye = new iCubEye("right");*/
   
    return true;
}


/**
* ------------- DEPRECATED ------------------
*/
bool disparityModule::open( Searchable& config ){
	

	imageOutputPort.open("/vergence/image:o");
	histoOutPort.open("/vergence/histo:o");

	printf("opening .....");
	fout = fopen("data/disp_data.txt", "wa");
	//fflush( stdout );

	optionsTorso.put("device", "remote_controlboard");
   	optionsTorso.put("local", "/local1");
   	optionsTorso.put("remote", "/icub/torso"); // CHANGE FOR THE ROBOT

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
	optionsHead.put("remote", "/icub/head");

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

void disparityModule::setOptions(yarp::os::Property opt){
	//options	=opt;
    // definition of the name of the module
    ConstString name=opt.find("name").asString();
    if(name!=""){
        printf("|||  Module named as :%s \n", name.c_str());
        this->setName(name.c_str());
    }
    ConstString value=opt.find("mode").asString();
    if(value!=""){
    }
}

bool disparityModule::close(){
	imageOutputPort.close();
	histoOutPort.close();
	imageInLeft.close();
	imageInRight.close();
	
	//fclose (fout);
	//printf("stouting .... \n");
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

    if((needLeft)&&(needRight))
    {
        imgInL = imageInLeft.read(true);
        imgInR = imageInRight.read(true);
        if(!init_flag){
            currentProcessor=new disparityProcessor();
            currentProcessor->imgInL=this->imgInL;
            currentProcessor->imgInR=this->imgInR;
            currentProcessor->histo=this->histo;
            currentProcessor->start();
            init_flag=true;
        }
    }

    
	//int disparityVal = 0;
	//double corrVal = 0.0;
    
    /*
	//create vector thast contains head encoders
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
		//Limg.resize(imgInL->width()/ratio,imgInL->height()/ratio);
		//Rimg.resize(imgInR->width()/ratio,imgInR->height()/ratio);
		//Disp.downSample(*imgInL, Limg);
		//Disp.downSample(*imgInR, Rimg);

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

        */
		
		//cout << "disparity Val " << disparityVal << " Corrval " << corrVal << " DISPARITY " << Disp._shiftFunction[Disp._shifts[0]] << endl;
        //int disparityVal=8;
		//cout << "disparity Val " << disparityVal  << endl;

		//Disp.makeHistogram(histo);
		if ( histoOutPort.getOutputCount() > 0 ){ 
			histoOutPort.prepare() = currentProcessor->histo;	
			histoOutPort.write();
		}
		

		if ( imageOutputPort.getOutputCount() > 0 ){ 
			imageOutputPort.prepare() = *imgInL;	
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
		//double angle=fb[8]-(180/M_PI)*atan(disparityVal/(2*206.026));
        //double angle=2;
		//if(angle<0)
		//	angle=0;		
		//cout << "2 atan " <<(180/M_PI)*atan(disparityVal/(2*206.026))<< " angle " << angle <<" current " << fb[8] << endl;

        if(currentProcessor!=0){
		
		    Bottle in,bot;
            //Bottle &bot = triangulationPort.prepare(); 
            bot.clear();

		    //Bottle& b = cmdOutput.prepare(); 
 		    char verg[100];
		    sprintf(verg, "set pos 5 %f", currentProcessor->angle);
    		
            if(cmdOutput.getOutputCount()){
                //bot.addString(verg);
		        bot.addString("set");
		        bot.addString("pos");		
		        bot.addInt(5);
		        bot.addDouble(currentProcessor->angle);
		        cmdOutput.write(bot,in);
		        bot.clear();
            }
        }//end if(currentProcessor!=0)
	//}
	
    
//	fflush( stdout );
	return true;
} 	







