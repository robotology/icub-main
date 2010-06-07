// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/imageThread.h>


/**
* initialise the thread
*/
bool imageThread::threadInit(){
    scaleFactorX=4;
    scaleFactorY=4;
    int sizeX=plottedLayer->getCol()*scaleFactorX;
    int sizeY=plottedLayer->getRow()*scaleFactorY;
    cvImage= cvCreateImage(cvSize(sizeX,sizeY), IPL_DEPTH_8U, 3 );
    image2=new ImageOf<PixelRgb>;
    image2->resize(sizeX,sizeY);
    //sizeX=plottedLayer->getColWeights()*scaleFactorX;
    //sizeY=plottedLayer->getRowWeights()*scaleFactorX;
    //imageWeights=new ImageOf<PixelRgb>;
    //imageWeights->resize(sizeX,sizeY);
    imageWeights=0;
    for (int x=0; x<sizeX; x++){
                for(int y=0;y<sizeY;y++){
                    PixelRgb &pix=image2->pixel(x,y);
                    //PixelRgb &pixW=imageWeights->pixel(x,y);
                    pix.r=0;
                    pix.b=0;
                    pix.g=0;
                    //pixW.r=0;
                    //pixW.b=0;
                    //pixW.g=0;
                }
            }
    printf("Image Thread initialising..... \n");
    //printf("opening image port..... \n");
    
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
    //1. produces the image of the state
    //printf("|||||||||||||||||||||||||||| \n");
    Vector v=*(plottedLayer->stateVector);
    for(int i=0;i<plottedLayer->getCol();i++){
        for(int j=0;j<plottedLayer->getRow();j++)
        {
            int pos=j*plottedLayer->getCol()+i;
            
            //printf("%d %f \n",pos,v(pos));
            for (int scaleX=0; scaleX<scaleFactorX; scaleX++){
                for(int scaleY=0;scaleY<scaleFactorY;scaleY++){
                    PixelRgb &pix=image2->pixel(i*scaleFactorX+scaleX,j*scaleFactorY+scaleY);
                    pix.r=v(pos)*255;
                    pix.b=v(pos)*255;
                    pix.g=v(pos)*255;
                }
            }
        }
    }
    
    //2. preduces the image of the weights to the upper level
    if(0!=plottedLayer->vishid){
        if(imageWeights==0){
            imageWeights=new ImageOf<PixelRgb>;
            imageWeights->resize(plottedLayer->getColWeights()*scaleFactorX,plottedLayer->getRowWeights()*scaleFactorY);
        }
        Matrix matVisHid=*(plottedLayer->vishid);
        double* w= matVisHid.data();
        for(int i=0;i<plottedLayer->getColWeights();i++){
            for(int j=0;j<plottedLayer->getRowWeights();j++)
            {
                int pos=j*plottedLayer->getColWeights()+i;
                
                //printf("%d %f \n",pos,v(pos));
                for (int scaleX=0; scaleX<scaleFactorX; scaleX++){
                    for(int scaleY=0;scaleY<scaleFactorY;scaleY++){
                        PixelRgb &pix=imageWeights->pixel(i,j);
                        pix.r=matVisHid(j,i)*255;
                        pix.b=matVisHid(j,i)*255;
                        pix.g=matVisHid(j,i)*255;
                    }
                }
            }
        }
     }
}

void imageThread::setLayer(Layer* layer){
    this->plottedLayer=layer;
}

/**
* code executed when the thread is released
*/
void imageThread::threadRelease(){
    printf("Image Thread releasing..... \n");
   // port.close();
}

ImageOf<PixelRgb>* imageThread::getYarpImage(){
    return image2;
}

void imageThread::setName(string n){
    this->name=n.substr(0,n.size());
}
