#include <iCub/BMLEngine.h>
#include <string.h>


#define T_TOUCH 1000
#define COOLING_RATE 4

//
static BMLEngine *engineModule;

// Image Receiver
static YARPImgRecv *ptr_imgRecv;
#define _imgRecv (*(ptr_imgRecv))

//Image been read
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImg;
#define _inputImg (*(ptr_inputImg))

// Semaphore
static yarp::os::Semaphore *ptr_semaphore;
#define _semaphore (*(ptr_semaphore))


void BMLEngine::openCommandPort(){
	_pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    printf("Registering port %s on network %s...\n", "/rea/BMLEngine/outCommand","dafult");
    bool ok = _pOutPort->open("/rea/BMLEngine/outCommand");
    if  (ok)
        printf("Port registration succeed!\n");
    else 
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
        }
}

void BMLEngine::closeCommandPort(){
	//_pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    printf("Closing port %s on network %s...\n", "/rea/BMLEngine/outCommand","dafult");
    _pOutPort->close();//("/rea/BMLEngine/outCommand");
}

/**
* function that opens the port where the inputImage is read
*/
bool BMLEngine::openPortImage(){
	bool ret = false;
	//int res = 0;
	// Registering Port(s)
    //reduce verbosity --paulfitz
	printf("Registering port %s on network %s...\n", "/rea/BMLEngine/inputImage","default");
	ret = _imgRecv.Connect("/rea/BMLEngine/inputImage","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
	}
}

/**
* function that opens the port where the inputImage is read
*/
bool  BMLEngine::closePortImage(){
	bool ret = true;
	//int res = 0;
	// Registering Port(s)
    //reduce verbosity --paulfitz
	printf("Closing port %s on network %s...\n", "/rea/BMLEngine/inputImage","default");
	_imgRecv.Disconnect();//("/rea/BMLEngine/inputImage","default");
	return ret;
}


bool getImage(){
	bool ret = false;
	ret = _imgRecv.Update();

	if (ret == false){
		return false;
	}

	_semaphore.wait();
	ret = _imgRecv.GetLastImage(&_inputImg);
	engineModule->ptr_inputImage=&_inputImg;
	_semaphore.post();
	
	//printf("GetImage: out of the semaphore \n");
	return ret;
}


void createObjects() {
	ptr_imgRecv = new YARPImgRecv;
	ptr_inputImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
	//ptr_inputImg2= new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
	ptr_semaphore = new yarp::os::Semaphore;
}

/*BMLEngine::BMLEngine(){
	
}*/

bool BMLEngine::outCommandPort(){
	if(strcmp(outCommand->c_str(),"")){	
		Bottle& outBot1=_pOutPort->prepare();
		//bOptions.addString("to");
		//bOptions.addString("Layer0");
		printf("outPort %s \n",outCommand->c_str());
		outBot1.fromString(outCommand->c_str());
		outBot1.addList()=bOptions;
		this->_pOutPort->writeStrict();
		outCommand->clear();
		bOptions.clear();
	}
	return true;
}

bool BMLEngine::open(Searchable& config) {
        count=0;
		scaleFactorX=20;
		scaleFactorY=20;
		clampingThreshold=1;
		countLayer=0;
		currentLayer=0;
		
		createObjects();
		openPortImage();
		openCommandPort();
		this->outCommand=new string("");

		port.open(getName("in")); 
		port0.open(getName("out0"));
		port1.open(getName("out1"));
        port2.open(getName("out2")); 
		portCmd.open(getName("inCmd"));
		portCmd.setStrict();
		
		img0=new ImageOf<PixelRgb>;
		img0->resize(320,240);
		img2=new ImageOf<PixelRgb>;
		img2->resize(320,240);
		ptr_inputImage2=new ImageOf<PixelRgb>;
		ptr_inputImage2->resize(320,240);

		//setting of the flags
		enableDraw=false;
		runFreely=false;
		runClamped=false;
		probClamped_flag=false;
		probFreely_flag=false;

		im_out = ippiMalloc_8u_C1(320,240,&psb);		
		im_tmp_tmp= ippiMalloc_8u_C1(320,240,&psb);
		im_tmp[0]=im_out;
		im_tmp[1]=im_out;
		im_tmp[2]=im_out;
		red_tmp= ippiMalloc_8u_C1(320,240,&psb);
		blue_tmp= ippiMalloc_8u_C1(320,240,&psb);
		green_tmp= ippiMalloc_8u_C1(320,240,&psb);
		for(int i=0;i<320*240;i++){
			red_tmp[i]=255;
			blue_tmp[i]=255;
			green_tmp[i]=255;
		}
			

		engineModule=this;
        return true;
    }

bool  BMLEngine::interruptModule() {
		port.interrupt();
		port2.interrupt();
		return true;
	}

bool BMLEngine::close() {
		port.close();
		port2.close();
		port0.close();//(getName("out0"));
		port1.close();//(getName("out1"));
        port2.close();//(getName("out2")); 
		portCmd.close();//(getName("inCmd"));
		closePortImage();
		closeCommandPort();
		layer0Image->stop();
		delete layer0Image;
		layer1Image->stop();
		delete layer1Image;
		//mb->saveConfiguration();		
		return true;
	}


/** 
 *Function updateModule is executed when the Module is updated, 
 * 1.It reads the command from a port, convertes it into parameters
 * 2.It evolves the Boltzmann machine 
 * 3.Extracts the output images of the active layers of the Boltzmann machine
 *
 * @param none
 * @return true if there were no errors in the funciton
 *
*/
bool BMLEngine::updateModule() {

	getImage();

	Bottle *bot=portCmd.read(false);
	if(bot!=NULL){
		string *commandTOT=new string(bot->toString().c_str());
		string command,option;
		string optionName1,optionValue1,optionName2, optionValue2;
		printf("Bottle  is: %s\n",commandTOT->c_str());
		unsigned int parOpen=commandTOT->find("(");
		if(parOpen==-1){
			printf("Simple command \n ");
			command=commandTOT->substr(0,commandTOT->size());
		}
		else
		{
			//first parameter list
			command=commandTOT->substr(0,parOpen-1);
			option=commandTOT->substr(parOpen+1,commandTOT->size()-parOpen);
			
			
			unsigned int  parPos1=option.find("(");
			unsigned int parPos2=option.find(")");
			unsigned int spacePos=option.find(" ");
			
			if(spacePos!=-1){
				printf("Presence of a space detected \n");
				optionName1=option.substr(parPos1+1,spacePos-parPos1);
				optionValue1= option.substr(spacePos+1,parPos2-spacePos-1);
				unsigned int dim=option.size();
				option=option.substr(parPos2+2,dim-2-parPos2);

				parPos1=option.find("(");
				if(parPos1!=-1){
					printf("found the second parentesis \n");
					parPos2=option.find(")");
					spacePos=option.find(" ");
					optionName2=option.substr(parPos1+1,spacePos-parPos1);
					optionValue2= option.substr(spacePos+1,parPos2-spacePos-1);
					option=option.substr(parPos2,option.size()-parPos2);
				}
			
				//string name=option.substr(1,spacePos-1);
				//string value=option.substr(spacePos+1,option.size()-spacePos);
			}
			
			printf("option: |%s| \n",option.c_str());
			printf("name1: |%s| \n",optionName1.c_str());
			printf("value1: |%s| \n",optionValue1.c_str());
			
			
			printf("option: |%s| \n",option.c_str());
			printf("name2: |%s| \n",optionName2.c_str());
			printf("value2: |%s| \n",optionValue2.c_str());
		}

		printf("command: |%s| \n",command.c_str());

		if(!strcmp(command.c_str(),"EvolveFreely")){
			printf("ExecuteFreely \n");
			mb->setTemperature(T_TOUCH);
			runFreely=true;
			runClamped=false;
		}
		else if(!strcmp(command.c_str(),"EvolveClamped")){
			printf("ExecuteClamped \n");
			mb->setTemperature(T_TOUCH);
			runFreely=false;
			runClamped=true;
		}
		else if(!strcmp(command.c_str(),"ConnectLayer")){
			int value1,value2;
			printf("ConnectLayer \n");


			if(!strcmp(optionValue1.c_str(),"layer0")){
				value1=0;
			}
			else if(!strcmp(optionValue1.c_str(),"layer1")){
				value1=1;
			}

			if(!strcmp(optionValue2.c_str(),"layer0")){
				value2=0;
			}
			else if(!strcmp(optionValue2.c_str(),"layer1")){
				value2=1;
			}

			map<std::string,Layer>::iterator iterK;
			map<std::string,Layer>::iterator iterL;
			map<std::string,Layer>::iterator iterE1,iterE2;
			
			int j=0;
			for(iterE1=mb->elementList.begin(); iterE1!=mb->elementList.end()&&j<value1;iterE1++){
				j++;
			}
			j=0;
			for(iterE2=mb->elementList.begin(); iterE2!=mb->elementList.end()&&j<value2;iterE2++){
				j++;
			}
			
			mb->interconnectLayers(iterE1->second,iterE2->second);
			
			
		}
		else if(!strcmp(command.c_str(),"Stop")){
			printf("ExecuteStop \n");
			runFreely=false;
			runClamped=false;
		}
		else if(!strcmp(command.c_str(),"Learn")){
			printf("Learn \n");
			mb->learn();
		}
		else if(!strcmp(command.c_str(),"setProbabilityFreely")){
			printf("setProbabilityFreely \n");
			this->probFreely_flag=true;
			this->probClamped_flag=false;
			
		}
		else if(!strcmp(command.c_str(),"setProbabilityClamped")){
			printf("setProbabilityClamped \n");
			this->probFreely_flag=false;
			this->probClamped_flag=true;
			
		}
		else if(!strcmp(command.c_str(),"setProbabilityNull")){
			printf("setProbabilityNull \n");
			this->probFreely_flag=false;
			this->probClamped_flag=false;
			
		}
		else if(!strcmp(command.c_str(),"ClampLayer")){
			printf("ClampLayer \n");
			clampLayer(currentLayer);
		}
		else if(!strcmp(command.c_str(),"ClampPattern")){
			printf("ClampPattern \n");
			printf("OptionValue1 %s \n" , optionValue1.c_str() );
			int bit;
			unsigned int comma_pos=optionValue1.find(",");
			string bit_str=optionValue1.substr(1,comma_pos-1);
			bit=atoi(bit_str.c_str());
			printf("bit_str %d \n", bit);
			mb->addClampedUnit(288+6+1+0,bit);
			optionValue1=optionValue1.substr(comma_pos+1,optionValue1.size()-comma_pos);
			printf("optionValue1 %s \n", optionValue1.c_str());
			comma_pos=optionValue1.find(",");
			bit_str=optionValue1.substr(1,comma_pos-1);
			bit=atoi(bit_str.c_str());
			optionValue1=optionValue1.substr(comma_pos+1,optionValue1.size()-comma_pos);
			printf("bit_str %d \n", bit);
			printf("optionValue1 %s \n", optionValue1.c_str());
			mb->addClampedUnit(288+6+1+1,bit);
		}
		else if(!strcmp(command.c_str(),"outHeadBehaviour")){
			printf("outHeadBehaviour \n");
			list<Unit>::iterator iter;
			int totUnits=6;
			bool end_loop=false;
			if(mb->_unitList.size()==0){
				return true;
			}
			for(iter=mb->_unitList.begin(); !end_loop;iter++){
				printf("unit name:%s,%d \n",iter->getName().c_str(),iter->getState());
				unsigned int posR=iter->getName().find('R');
				string layerName=iter->getName().substr(1,posR-1);
				int layerNumber=2;
				printf("layer name:%s where as looking for %d \n",layerName.c_str(), layerNumber);
				int extractNumber=atoi(layerName.c_str());
				if(extractNumber==layerNumber)
					end_loop=true;
			}
			Bottle tmp;
			iter--;
			//layer row of dimension 6 not considered
			for(int i=0;i<totUnits;i++)
				iter++;
			//first unit not considered
			iter++;
			printf("%d,",iter->getState());
			//behaviour0
			if(iter->getState()){
				tmp.addString("behaviour0");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			printf("%d,",iter->getState());
			//behaviour1
			if(iter->getState()){
				tmp.addString("behaviour1");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			printf("%d,",iter->getState());
			//behaviour2
			if(iter->getState()){
				
				tmp.addString("behaviour2");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			printf("%d,",iter->getState());
			//behaviour3
			if(iter->getState()){
				
				tmp.addString("behaviour3");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			printf("%d,",iter->getState());
			//behaviour4
			if(iter->getState()){
				
				tmp.addString("behaviour4");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			printf("%d,",iter->getState());
			//behaviour4
			if(iter->getState()){
				
				tmp.addString("behaviour4");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			outCommand->assign("outHeadBehaviour");
			outCommand->assign("outHeadBehaviour");
		}
		else if(!strcmp(command.c_str(),"outEyesBehaviour")){
			printf("outEyesBehaviour \n");
			list<Unit>::iterator iter;
			int totUnits;
			bool end_loop=false;
			for(iter=mb->_unitList.begin(); !end_loop;iter++){
				printf("unit name:%s,%d \n",iter->getName().c_str(),iter->getState());
				unsigned int posR=iter->getName().find('R');
				string layerName=iter->getName().substr(1,posR-1);
				int layerNumber=2;
				printf("layer name:%s where as looking for %d \n",layerName.c_str(), layerNumber);
				int extractNumber=atoi(layerName.c_str());
				if(extractNumber==layerNumber)
					end_loop=true;
			}
			Bottle tmp;
			iter--;
			printf("%d,",iter->getState());
			//behaviour0
			if(iter->getState()){
				tmp.addString("behaviour0");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			printf("%d,",iter->getState());
			//behaviour1
			if(iter->getState()){
				tmp.addString("behaviour1");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			printf("%d,",iter->getState());
			//behaviour2
			if(iter->getState()){
				
				tmp.addString("behaviour2");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			printf("%d,",iter->getState());
			//behaviour3
			if(iter->getState()){
				
				tmp.addString("behaviour3");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			printf("%d,",iter->getState());
			//behaviour4
			if(iter->getState()){
				
				tmp.addString("behaviour4");
				tmp.addInt(10);
				bOptions.addList()=tmp;
				tmp.clear();
			}
			iter++;
			outCommand->assign("outHeadBehaviour");
		}
		else if((!strcmp(command.c_str(),"Learn")))
			printf("Learn \n");
		else if(!strcmp(command.c_str(),"Save")){
			printf("save Configuration \n");
			this->mb->saveConfiguration();
		}
		else if((!strcmp(command.c_str(),"Load"))){
			printf("loadConfiguration \n");
			this->loadConfiguration("configuration.ini");
		}
		else if((!strcmp(command.c_str(),"AddLayer"))){
			printf("addLayer \n");
			int valueRow=atoi(optionValue1.c_str());
			int valueCol=atoi(optionValue2.c_str());
			addLayer(countLayer,valueRow+2,valueCol+2);
			countLayer++;
		}
		else if((!strcmp(command.c_str(),"CurrentLayer"))){
			
			int valueLayer=atoi(optionValue1.c_str());
			printf("CurrentUnit %d \n",valueLayer);
			//int valueCol=atoi(optionValue2.c_str());
			this->setCurrentLayer(valueLayer);
		}	
	}	

	count=(count+1)%40;
	//cout<<count;
	//cout<<".";		
	list<Unit>::iterator iter;
	list<Unit>::iterator iter2;

	//extract every unit present in the BoltzmannMachine
	//int psb;
	IppiSize srcsize={320,240};
	
	
	int k=0;
	/*
	if(enableDraw){
		img.resize(320,240);
		//img2->resize(320,240);
		img_tmp.resize(320,240);
		int countLayer=-1;
		//for(iterE=mb->elementList.begin(); iterE!=mb->elementList.end();iterE++){	//iterE=mb->elementList.begin();

			

			//Layer layer=iterE->second;
			int totUnits;
			int countUnit=0;
			int ycount=-1;
			string layerName("");
			string layerName_pr("");
			
			for(iter=mb->_unitList.begin(); iter!=mb->_unitList.end();iter++){
				//countUnit++;
				ct = ((countUnit%10)-1)*24;
				
				//printf("unit name:%s,%d \n",iter->getName().c_str(),iter->getState());
				unsigned int posR=iter->getName().find('R');
				layerName=iter->getName().substr(0,posR-0);
				//printf("layer name:%s \n",layerName.c_str());
				
				if(strcmp(layerName_pr.c_str(),layerName.c_str())){
					//count the number of element in the row
					bool end_loop=false;
					totUnits=-1;
					iter2=iter;
					//count the number of units
					for(;!end_loop;iter2++){
						if(iter2==mb->_unitList.end()){
							totUnits++;
							break;
						}
						unsigned int posR=iter2->getName().find('R');
						unsigned int posU=iter2->getName().find('U');
						string number_str=iter2->getName().substr(posR+1,posU-posR-1);
						int number=atoi(number_str.c_str());
						if(number!=0){
							end_loop=true;
						}
						totUnits++;
					}
					//printf("totUnits: %d of layer %s \n",totUnits, layerName.c_str());
					//printf("countUnits: %d", totUnits);
					//printf("Change!");
					//_______  EXIT CONDITION _________________
					if(strcmp(layerName_pr.c_str(),"")){
						string countLayer_str=layerName_pr.substr(1,1);
						countLayer=atoi(countLayer_str.c_str());
						//printf("number Layer %d \n",countLayer);
						// output the image
						//ippiCopy_8u_C1R(red_tmp,psb,im_tmp[0],psb,srcsize);
						//ippiCopy_8u_C1R(green_tmp,psb,im_tmp[1],psb,srcsize);
						//ippiCopy_8u_C1R(blue_tmp,psb,im_tmp[2],psb,srcsize);
						im_tmp[0]=red_tmp;
						im_tmp[1]=green_tmp;
						im_tmp[2]=blue_tmp;
						

						//ippiCopy_8u_C1R(green_tmp,psb,im_tmp[1],psb,srcsize);
						//ippiCopy_8u_C1R(blue_tmp,psb,im_tmp[2],psb,srcsize);
						//ippiCopy_8u_C1R(red_tmp,psb,img2->getPixelAddress(0,0),320,srcsize);
						
						// output the image
						if(countLayer==0){
							ippiCopy_8u_P3C3R(im_tmp,psb,img2->getPixelAddress(0,0),320*3,srcsize);
							port0.prepare() = *img2;
							//if((unsigned int)ptr_inputImage2!=0xcccccccc)
							//	port0.prepare()= *ptr_inputImage2;
							port0.write();
						}
						else if(countLayer==1){
							ippiCopy_8u_P3C3R(im_tmp,psb,img2->getPixelAddress(0,0),320*3,srcsize);
							port1.prepare() = *img2;
							port1.write();
						}
						else if(countLayer==2){
							ippiCopy_8u_P3C3R(im_tmp,psb,img0->getPixelAddress(0,0),320*3,srcsize);
							port2.prepare() = *img2;
							port2.write();
						}
						else if(countLayer==3){
							ippiCopy_8u_P3C3R(im_tmp,psb,img0->getPixelAddress(0,0),320*3,srcsize);
							port3.prepare() = *img2;
							port3.write();
						}
					}


					//_______
					
					
					for(int i=0;i<320*240;i++){
						red_tmp[i]=255;
						blue_tmp[i]=255;
						green_tmp[i]=255;
					}
					
					layerName_pr.assign(layerName);
					countUnit=0;
					end_loop=false;
					ycount=0;
				} // close if(strcmp(layerName_pr.c_str(),layerName.c_str()))
				
				
				

				if(countUnit%totUnits==0){
					ycount++;
				}
				
				//produces the image binary image
				if(iter->getState()){
					for(int x=0;x<scaleFactorX;x++)
						for(int y=0;y<scaleFactorY;y++){
							red_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=255;
							blue_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=255;
							green_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=255;
						}
				}
				else{
					for(int x=0;x<scaleFactorX;x++)
						for(int y=0;y<scaleFactorY;y++){
							red_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=50;
							blue_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=50;
							green_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=50;
						}
				}
				countUnit++;
			} // close  for(iter=mb->_unitList.begin(); iter!=mb->_unitList.end();iter++)
			


			//printf(" Final picture : totUnits: %d of layer %s \n",totUnits, layerName.c_str());

			string countLayer_str=layerName.substr(1,1);
			countLayer=atoi(countLayer_str.c_str());
			//printf("number Layer %d \n",countLayer);
			// output the image
			//ippiCopy_8u_C1R(red_tmp,psb,im_tmp[0],psb,srcsize);
			//ippiCopy_8u_C1R(green_tmp,psb,im_tmp[1],psb,srcsize);
			//ippiCopy_8u_C1R(blue_tmp,psb,im_tmp[2],psb,srcsize);
			im_tmp[0]=red_tmp;
			im_tmp[1]=green_tmp;
			im_tmp[2]=blue_tmp;

			//ippiCopy_8u_C1R(green_tmp,psb,im_tmp[1],psb,srcsize);
			//ippiCopy_8u_C1R(blue_tmp,psb,im_tmp[2],psb,srcsize);
			//ippiCopy_8u_C1R(red_tmp,psb,img2->getPixelAddress(0,0),320,srcsize);
			ippiCopy_8u_P3C3R(im_tmp,psb,img2->getPixelAddress(0,0),320*3,srcsize);
			// output the image
			if(countLayer==0){
				port0.prepare() = *img2;
				//if((unsigned int)ptr_inputImage2!=0xcccccccc)
				//	port0.prepare()= *ptr_inputImage2;
				port0.write();
			}
			else if(countLayer==1){
				port1.prepare() = *img2;
				port1.write();
			}
			else if(countLayer==2){
				port2.prepare() = *img2;
				port2.write();
			}
			else if(countLayer==3){
				port3.prepare() = *img2;
				port3.write();
			}

		
			for(int i=0;i<320*240;i++){
				red_tmp[i]=255;
				blue_tmp[i]=255;
				green_tmp[i]=255;
			}

			layerName_pr.assign(layerName);
			countUnit=0;
			ycount=0;
		//}
	}

	*/
	
	


	//---------------------EVOLVE the Boltzmann Machine
	if(runFreely){
		mb->evolveFreely(2,0);
		int value=mb->getTemperature()+COOLING_RATE;
		mb->setTemperature(value);
		count++;
	}
	else if(runClamped){
		mb->evolveClamped(2,1);
		int value=mb->getTemperature()+COOLING_RATE;
		mb->setTemperature(value);
		count++;
	}
	else if(probFreely_flag){
		mb->setProbabilityFreely();
	}
	else if(probClamped_flag){
		mb->setProbabilityClamped();
	}
	if((count>10)&&(count<20)){
		//mb->evolveFreely(2);
	}
	if(false){
			enableDraw=true;
			//equilibrium of the freely mode or clamped mode
			//it is now possible of set the probability either of freely mode or clamped modo
			if(runFreely){
				mb->setProbabilityFreely();
				count=0;
			}
			else if(runClamped){
				mb->setProbabilityClamped();
				count=0;
			}
			//mb->setProbabilityFreely();
	}

	if((count>20)&&(count<30)){
		//mb->evolveClamped(2);
	}
	if(count==30){
		//equilibrium of clampled mode
		//it is now possible of set the probability of clamped mode
		//mb->setProbabilityClamped();
		//LEARNING COMMAND
		//mb->learn();
	}
	if((count>30)&&(count<40)){
		//run freely with input layer clamped
	}

	outCommandPort();

    return true;
}

/** 
* function that sets the scaleFactorX
* @param value new value of the scaleFactorX
*/
void BMLEngine::setScaleFactorX(int value){
	this->scaleFactorX=value;
}
/** 
* function that sets the scaleFactorY
* @param value new value of the scaleFactorY
*/
void BMLEngine::setScaleFactorY(int value){
	this->scaleFactorY=value;
}



/** 
* function that set the number of the layer active 
* @param value number of the layer actually active
*/
void BMLEngine::setCurrentLayer(int value){
	this->currentLayer=value;
}

void BMLEngine::loadConfiguration(string filename){
	//attach(port); // process input to this port
	string fileName("configuration.ini");
	FILE * pFile;
	int n;
	char name [100];
	cout<<"Checking for the Boltzmann Machine file "<<fileName<<"............"<<endl;
	pFile = fopen (fileName.c_str(),"r");
	if(pFile==NULL){
		cout<<"Creating the Boltzmann Machine from scratch "<<endl;
		mb=new MachineBoltzmann();
	}
	else{
		cout<<"Loading the Boltzmann Machine from file"<<endl;
		cout<<"Creating a default Boltzmann Machine...."<<endl;
		mb=new MachineBoltzmann();
		cout<<"Loading configuration from file"<<endl;
		mb->loadConfiguration();
	}	
	
	//cout<<"Number of allocated elements:"<< mb->getCountElements()<<endl;
	if(this->countLayer>0)
		enableDraw=true;
	
}

void BMLEngine::addLayer(int number,int colDimension, int rowDimension){
	char number_str[3];
	int n=sprintf(number_str,"%d",number);
	string n_string(number_str,n);
	string layerName("L");
	layerName.append(n_string);
	printf("layerName:%s",layerName.c_str());
	Layer *layer=new Layer(layerName,colDimension,rowDimension);
	mb->addLayer(*layer);
	mb->migrateLayer(*layer);
	//mb->interconnectLayer(number);
	enableDraw=true;
	// allocate the relative thread
	if(number==0){
		layer0Image=new imageThread(50,"out00");
		layer0Image->setLayer(layer);
		layer0Image->start();
	}
	else if(number==1){
		layer1Image=new imageThread(50,"out11");
		layer1Image->setLayer(layer);
		layer1Image->start();
	}
}

void BMLEngine::clampLayer(int layerNumber){
	int width=320;
	int height=240;
	IppiSize srcsize={width,height};
	//1.extracts 3 planes
	Ipp8u* im_out = ippiMalloc_8u_C1(320,240,&psb);
	Ipp8u* im_tmp[3];
	Ipp8u* im_tmp_tmp[3];
	im_tmp_tmp[0]=ippiMalloc_8u_C1(320,240,&psb);
	im_tmp_tmp[1]=ippiMalloc_8u_C1(320,240,&psb);
	im_tmp_tmp[2]=ippiMalloc_8u_C1(320,240,&psb);
	Ipp8u* im_tmp_red=ippiMalloc_8u_C1(320,240,&psb);
	Ipp8u* im_tmp_green=ippiMalloc_8u_C1(320,240,&psb);
	Ipp8u* im_tmp_blue=ippiMalloc_8u_C1(320,240,&psb);
	Ipp8u* im_tmp_tmp2=ippiMalloc_8u_C1(320,240,&psb);
	im_tmp[0]=ippiMalloc_8u_C1(320,240,&psb);
	im_tmp[1]=ippiMalloc_8u_C1(320,240,&psb);
	im_tmp[2]=ippiMalloc_8u_C1(320,240,&psb);
	
	//if((unsigned int)ptr_inputImage==0xcccccccc)
	//	return;
	ippiCopy_8u_C3P3R(this->ptr_inputImage->getPixelAddress(0,0),320*3,im_tmp,psb,srcsize);
	ippiCopy_8u_C1R(im_tmp[0],psb,im_tmp_red,psb,srcsize);
	ippiCopy_8u_C1R(im_tmp[1],psb,im_tmp_green,psb,srcsize);
	ippiCopy_8u_C1R(im_tmp[2],psb,im_tmp_blue,psb,srcsize);
	//2. gets the maximum value between planes
	for(int i=0;i<320*240;i++){
		if(im_tmp_red[i]<im_tmp_green[i])
			if(im_tmp_green[i]<im_tmp_blue[i])
				im_out[i]=im_tmp_blue[i];
			else
				im_out[i]=im_tmp_green[i];
		else
			if(im_tmp_red[i]<im_tmp_blue[i])
				im_out[i]=im_tmp_blue[i];
			else
				im_out[i]=im_tmp_red[i];
	}
	ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[0],psb,srcsize);
	ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[1],psb,srcsize);
	ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[2],psb,srcsize);
	//ippiCopy_8u_C1C3R(im_out,psb,this->ptr_inputImage2->getPixelAddress(0,0),320*3,srcsize);
	ippiCopy_8u_P3C3R(im_tmp_tmp,psb,this->ptr_inputImage2->getPixelAddress(0,0),320*3,srcsize);
	
	
	//im_tmp_tmp[0]=im_out;
	//im_tmp_tmp[1]=im_out;
	//im_tmp_tmp[2]=im_out;

	//2.Extract the necessary information
	//finds the starting unit of the layer
	string layerName;
	list<Unit>::iterator iter;
	list<Unit>::iterator iter2;
	int totUnits;
	bool end_loop=false;
	for(iter=mb->_unitList.begin(); !end_loop;iter++){
				printf("unit name:%s,%d \n",iter->getName().c_str(),iter->getState());
				unsigned int posR=iter->getName().find('R');
				layerName=iter->getName().substr(1,posR-1);
				printf("layer name:%s where as looking for %d \n",layerName.c_str(), layerNumber);
				int extractNumber=atoi(layerName.c_str());
				if(extractNumber==layerNumber)
					end_loop=true;
	}

	//count the number of element in the row
	end_loop=false;
	bool end_loop_layer=false;
	totUnits=0;
	int previous_number=0;
	int totRows=0;
	iter2=iter;
	for(;!end_loop_layer;){
		totUnits=0;
		printf("Counting number of units and number of rows");
		for(;((!end_loop)&&(iter2!=mb->_unitList.end()));iter2++){
			unsigned int posR=iter2->getName().find('R');
			unsigned int posU=iter2->getName().find('U');
			string number_str=iter2->getName().substr(posR+1,posU-posR-1);
			int number=atoi(number_str.c_str());
			if(number!=previous_number){
				end_loop=true;
				previous_number=number;
			}
			totUnits++;
		}
		if(iter2==mb->_unitList.end()){
			totRows++;
			totUnits++;
			break;
	}
	printf("countUnits: %d", totUnits);
	unsigned int posR=iter2->getName().find('R');
	layerName=iter2->getName().substr(1,posR-1);
	printf("layer name:%s \n",layerName.c_str());
	int extractNumber=atoi(layerName.c_str());
	if(extractNumber!=layerNumber)
		end_loop_layer=true;
	end_loop=false;
	totRows++;
	
	}

	printf("rowDimension %d \n",totRows);


	//3. maps the intensity on the layer
	// it does not consider the hidden units of the layer
	int rectDimX=320/(totUnits-2);
	int rectDimY=240/(totRows-2);
	int sum=0;
	for(int boltzmannMachineRow=0;boltzmannMachineRow<totRows-2;boltzmannMachineRow++)
		for(int boltzmannMachineCol=0;boltzmannMachineCol<totUnits-2;boltzmannMachineCol++){
			sum=0;
			for(int y=0;y<rectDimY;y++){
				for(int x=0;x<rectDimX;x++){
					sum+=im_out[boltzmannMachineRow*rectDimY*320+boltzmannMachineCol*rectDimX+x+320*y];
				}
			}
			float mean=sum/(rectDimX*rectDimY);
			printf("mean of the unit %f ----> ",mean);
			//set the threashold to decide whether the unit has to be elicited
			if(mean>clampingThreshold){
				mb->addClampedUnit(boltzmannMachineCol+1+(boltzmannMachineRow+1)*totUnits+layerNumber*totUnits*totRows,1);
			}
			else{
				mb->addClampedUnit(boltzmannMachineCol+1+(boltzmannMachineRow+1)*totUnits+layerNumber*totUnits*totRows,0);
			}
		
		}
	


	//4. free memory
	ippiFree(im_out);
	ippiFree(im_tmp);
	ippiFree(im_tmp_tmp);
}
