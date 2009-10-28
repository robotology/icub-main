// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/imageThread.h>


/**
* initialise the thread
*/
bool imageThread::threadInit(){
	scaleFactorX=25;
	scaleFactorY=25;
	cvImage= cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3 );
	image2=new ImageOf<PixelRgb>;
	image2->resize(320,240);
	printf("Image Thread initialising.....");
	//string str=getName();
	string str("/rea/BMLEngine/");
	str.append(this->name);
	port.open(str.c_str()); 
	return true;
}

/**
* code that is executed after the thread starts
* @param s is true if the thread started
*/
void imageThread::afterStart(bool s){
	if(s){
		printf("Image Thread after start.....\n");		
	}

}

/**
* running code of the thread
*/
void imageThread::run(){
	printf("Image Thread running..... \n");
	//1. produces the image
	printf("|||||||||||||||||||||||||||| \n");
	for(int i=0;i<plottedLayer->getCol();i++){
		for(int j=0;j<plottedLayer->getRow();j++)
		{
			int pos=i*plottedLayer->getRow()+j;
			Vector v=*(plottedLayer->stateVector);
			printf("%d %f \n",pos,v(pos));
			for (int scaleX=0; scaleX<scaleFactorX; scaleX++){
				for(int scaleY=0;scaleY<scaleFactorY;scaleY++){
					PixelRgb &pix=image2->pixel(i+scaleX,j+scaleY);
					pix.r=v(pos)*255;
					pix.b=v(pos)*255;
					pix.g=v(pos)*255;
				}
			}
		}
	}
	//2. force the image out on the port
	port.prepare()=*(image2);
	port.write();
}
void imageThread::setLayer(Layer* layer){
	this->plottedLayer=layer;
}
/**
* code executed when the thread is released
*/
void imageThread::threadRelease(){
	printf("Image Thread releasing..... \n");	
}

ImageOf<PixelRgb>* imageThread::getYarpImage(){
	return image2;
}

void imageThread::setName(string n){
	this->name=n.substr(0,n.size());
}
