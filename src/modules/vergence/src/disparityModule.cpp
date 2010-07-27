
#include <iCub/disparityModule.h>

using namespace yarp::math;

disparityModule::disparityModule() {
    init_flag=false;
    currentProcessor=0;
    ratio = 4.00;
    robotHead = 0;
    robotTorso = 0;
}

disparityModule::~disparityModule() { } 

bool disparityModule::configure(ResourceFinder &rf) {
    //create the imageIn ports
    
   

    Time::turboBoost();
    cmdPort.open(getName("/cmd:i"));
    attach(cmdPort);

    currentProcessor=new disparityProcessor();
    currentProcessor->start();

    printf("\n waiting for connection of the input port \n");
    return true;
}


/**
* ------------- DEPRECATED ------------------
*/
/*
bool disparityModule::open( Searchable& config ) {
	
	imageOutputPort.open("/vergence/image:o");
	histoOutPort.open("/vergence/histo:o");

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
*/

void disparityModule::setOptions(yarp::os::Property opt) {
    // definition of the name of the module

    ConstString name=opt.find("name").asString();
    if(name!="") {
        printf("|||  Module named as :%s \n", name.c_str());
        this->setName(name.c_str());
    }

    ConstString value=opt.find("mode").asString();
    if(value!="") {
    }
}

bool disparityModule::close(){
	
	return true;
}

bool disparityModule::interruptModule(){
	

	return true;
}

bool disparityModule::updateModule(){	
/*	
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

  */  
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
		/*
        if ( histoOutPort.getOutputCount() > 0 ){ 
			histoOutPort.prepare() = currentProcessor->histo;	
			histoOutPort.write();
		}
		

		if ( imageOutputPort.getOutputCount() > 0 ){ 
			imageOutputPort.prepare() = *imgInL;	
			imageOutputPort.write();
		}
        */

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

        /*

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

    */
	
    

	return true;
} 	







