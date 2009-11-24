// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/cRefinementModule.h>


/**
*function that opens the module
*/
bool cRefinementModule::open(Searchable& config) {
    ct = 0;
    portInImage.open(getName("inImage"));
	portInProbClassA.open(getName("inProbClassA"));
	portInProbOther.open(getName("inProbOther"));
	portOut.open(getName("out"));
    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal

	//istantiate the MultiClass object pointed by m
	
	isize.width  = 320;
	isize.height = 240;
	int psb_i;
	int psb_i_4;

	edge_map = ippiMalloc_8u_C1(isize.width,isize.height,&psb_i);
	in       = ippiMalloc_8u_C4(isize.width,isize.height,&psb_i_4);
	imgOut=new ImageOf<PixelMono>;
	imgOut->resize(isize.width,isize.height);
	imageOther=new ImageOf<PixelMono>;
	imageOther->resize(isize.width,isize.height);
	imageClassA=new ImageOf<PixelMono>;
	imageClassA->resize(isize.width,isize.height);

	int nclasses=2; //class a and other.
	m = new MultiClass(isize,psb_i,nclasses,&this->properties);

	//istantiate the converter
	ci = new Convert_RGB(isize);

    return true;
}

/** 
* tries to interrupt any communications or resource usage
*/
bool cRefinementModule::interruptModule() {
    portInImage.interrupt();
	portInProbClassA.interrupt();
	portInProbOther.interrupt();
	portOut.interrupt();
	cmdPort.interrupt();
    return true;
}


bool cRefinementModule::close(){
	portInImage.close();
	portInProbClassA.close();
	portInProbOther.close();
	portOut.close();
	cmdPort.close();
    return true;
}

void cRefinementModule::setProperties(MultiClass::Parameters prop){
	   properties=prop;
}

void cRefinementModule::setOptions(yarp::os::Property opt ){
	options	=opt;
	// definition of the mode
	
	// definition of the name of the module
	ConstString name=opt.find("name").asString();
	printf("Module named as :%s \n", name.c_str());
	if(name.c_str()!=NULL)
		this->setName(name.c_str());
	printf("\n");
}

bool cRefinementModule::updateModule() {
	//set in grays the image
	imgInput = portInImage.read(false);
	if(imgInput==NULL){
		return true;	
	}
	imgProbClassA = portInProbClassA.read(false);
	if(imgProbClassA==NULL){
		return true;	
	}
	/*imgProbOther = portInProbOther.read(false);
	if(imgProbOther==NULL){
		return true;	
	}*/
	imgProbOther=inverse(imgProbClassA);


	// calling the classifier
	int psb_i,psb_i_4;
	Ipp8u *in       = ippiMalloc_8u_C4(isize.width,isize.height,&psb_i_4);
	Ipp8u* pDst[4];
	pDst[0]=ippiMalloc_8u_C1(320,240,&psb_i);
	pDst[1]=ippiMalloc_8u_C1(320,240,&psb_i);
	pDst[2]=ippiMalloc_8u_C1(320,240,&psb_i);
	pDst[3]=ippiMalloc_8u_C1(320,240,&psb_i);
	ippiCopy_8u_C3P3R(imgInput->getPixelAddress(0,0),320*3,pDst,320,isize);
	pDst[3]=pDst[0];
	ippiCopy_8u_P4C4R(pDst,320,in,imgInput->width()*4,isize);
	ci->proc(in,psb_i_4);
	Ipp8u* pr_other=ippiMalloc_8u_C1(320,240,&psb_i);
	Ipp8u* pr_classA=ippiMalloc_8u_C1(320,240,&psb_i);
	
	ippiCopy_8u_C1R(imgProbClassA->getPixelAddress(0,0),320,pr_classA,320,isize);
	imgProbOther=inverse(imgProbClassA);
	ippiCopy_8u_C1R(imgProbOther->getPixelAddress(0,0),320,pr_other,320,isize);

	// parsing the output of the classifier
	Ipp8u**p_pr = (Ipp8u**) malloc(2*sizeof(Ipp8u*));
	p_pr[0] = pr_other;
	p_pr[1] = pr_classA;
	Ipp8u* r=ci->get_y();
	m->proc(r,p_pr);  
    
	//prepare the output to the ports
	ippiCopy_8u_C1R(p_pr[1],320,imgOut->getPixelAddress(0,0),320,isize);
	portOut.prepare() = *imgOut;		
	portOut.write();
	ippiFree(p_pr);
    return true;
}


ImageOf<PixelMono>* cRefinementModule::inverse(ImageOf<PixelMono>* input){
	ImageOf<PixelMono>* ret=new ImageOf<PixelMono>;
	ret->resize(320,240);
	
	for(int c=0;c<input->width();c++){
		for(int r=0;r<input->height();r++){
			(*ret)(c,r)=abs((*input)(c,r)-255);
		}
	}
	return ret;
}