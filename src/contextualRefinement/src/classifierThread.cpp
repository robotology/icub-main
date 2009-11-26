#include <iCub/classifierThread.h>

/**
	* return the 255 complementary image of the input
	*/
static ImageOf<PixelMono>* inverse(ImageOf<PixelMono>* input){
	ImageOf<PixelMono>* ret=new ImageOf<PixelMono>;
	ret->resize(320,240);
	
	for(int c=0;c<input->width();c++){
		for(int r=0;r<input->height();r++){
			(*ret)(c,r)=abs((*input)(c,r)-255);
		}
	}
	return ret;
}

/**
* Constructor of the thread
* @param Property &op is a parameter passed as reference of the thread property
*/
classifierThread::classifierThread(Property &op):RateThread(THREAD_RATE)
{
	printf("Thread allocation ..... \n");
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
        
}

classifierThread::~classifierThread(){
	printf("Thread destruction ..... \n");
	delete imgOut;
	delete imageOther;
	delete imageClassA;
	delete m;
	delete ci;
}

bool classifierThread::threadInit()
{
    printf("Thread initialisation ..... \n");
    return true;
}

void classifierThread::run()
{
	
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
	//imgProbOther=inverse(imgProbClassA);
	ippiCopy_8u_C1R(imgProbOther->getPixelAddress(0,0),320,pr_other,320,isize);

	// parsing the output of the classifier
	Ipp8u**p_pr = (Ipp8u**) malloc(2*sizeof(Ipp8u*));
	p_pr[0] = pr_other;
	p_pr[1] = pr_classA;
	Ipp8u* r=ci->get_y();
	m->proc(r,p_pr);  
	ippiFree(p_pr);
}

void classifierThread::threadRelease()
{
	printf("Thread released ..... \n");
}