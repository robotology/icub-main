// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
 
#include <iCub/SalienceModule.h>
#include <iCub/vis/SalienceFactory.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::vis;

SalienceModule::SalienceModule(){

}

SalienceModule::~SalienceModule(){

}

double SalienceModule::getPeriod(){
	return 0.0;
}

bool SalienceModule::configure(yarp::os::ResourceFinder &rf){
    
	ConstString str = rf.check("name", Value("/salience"), "module name (string)").asString();
	setName(str.c_str()); // modulePortName  
	attachTerminal();

	// framerate
	_intFPS = rf.check("fps", Value(20), "Try to achieve this number of frames per second (int).").asInt();
	_intPrintFPSAfterNumFrames = rf.check("fpsOutputFrequency", Value(20), "Print the achieved framerate after this number of frames (int).").asInt();
	_dblTPF = 1.0f/((float)_intFPS);
	_intFPSAchieved = 0;
	_intFC = 0;
	_dblTPFAchieved = 0.0;
	_dblStartTime = 0.0;

    numBlurPasses = rf.check("numBlurPasses",
                                    Value(0),
                                    "Blur the output map numBlurPasses times with a gaussian 3x3 kernel (int).").asInt();
    drawSaliencePeak = rf.check("drawSaliencePeak",
                                Value(1),
                                "Draw a crosshair at salience peak onto the output visualization image (int [0|1]).").asInt()!=0;
	thresholdSalience = rf.check("thresholdSalience",
                                    Value(0.0),
                                    "Set salience map values < threshold to zero (double).").asDouble();
    activateIOR = rf.check("activateInhibitionOfReturn",
                           Value(0),
                           "Use IOR (int [0|1]).").asInt()!=0;
    if (activateIOR){
        ior.open(rf);
    }  

    filter = NULL;
    ConstString filterName = rf.check("filter",
                                          Value(""),
                                          "filter to use (string [group|intensity|color|directional|motion|emd|ruddy|face])").asString();

    if (filterName=="") {
        printf("*** Please specify a filter, e.g. --filter motion\n");
        vector<string> names = SalienceFactories::getPool().getNames();
        printf("*** Filters available: ");
        for (unsigned int i=0; i<names.size(); i++) {
            printf("%s ", names[i].c_str());
        }
        printf("\n");
        return false;
    }

    filter = SalienceFactories::getPool().get(filterName);
    if (filter!=NULL) {
        bool ok = filter->open(rf);
        if (!ok) {
            delete filter;
            filter = NULL;
            return false;
        }
    }

    imgPort.open(getName("/view"));
	peakPort.open(getName("/peak")); //For streaming saliency peak coordinates (Alex, 31/05/08)
    filteredPort.open(getName("/map"));
    configPort.open(getName("/conf"));
    attach(configPort);

    oldSizeX = -1;
    oldSizeY = -1;
    needInit = true;

	fflush(stdout);
    return true;
}

bool SalienceModule::close() {
    if (filter!=NULL) delete filter;
    filter = NULL;
    //for (int i = 0; i < tmpBuffer.size(); i++){
    //    //cout << "deleting buffered image: " << i << endl;
    //    delete tmpBuffer[i];
    //}
    imgPort.close();
	peakPort.close(); //(Alex, 31/05/08)
    filteredPort.close();
    configPort.close();
	fflush(stdout);
    return true;
}

bool SalienceModule::interruptModule() {
    imgPort.interrupt();
	peakPort.interrupt(); //(Alex, 31/05/08)
    filteredPort.interrupt();
    configPort.interrupt();
	fflush(stdout);
    return true;
}

bool SalienceModule::updateModule(){
	
	// framerate stuff
	_intFC++;
	if(_intPrintFPSAfterNumFrames <= _intFC && _intPrintFPSAfterNumFrames > 0){
		std::cout << "FPS: " << _intFPSAchieved << std::endl;
		_intFC = 0;
	}
	_dblTPFAchieved = ((float)(yarp::os::Time::now() - _dblStartTime));
	if(_dblTPFAchieved < _dblTPF){
		yarp::os::Time::delay(_dblTPF-_dblTPFAchieved);
		_intFPSAchieved = _intFPS;
	}
	else{
		_intFPSAchieved = (int)::floor((1.0 / _dblTPFAchieved) + 0.5);
	}
	_dblStartTime = yarp::os::Time::now();

    // read image from port
    ImageOf<PixelRgb> *img = imgPort.read(false);
	if (img != NULL){

		// if image size changes, need to resize buffered images
		if (img->width() != oldSizeX || img->height() != oldSizeY || needInit){
			//resizeBufferedImages(img->width(), img->height());
			needInit = false;
		}

		ImageOf<PixelRgb> &imgView = imgPort.prepare();
		ImageOf<PixelFloat> &imgMap = filteredPort.prepare();
		mutex.wait();

		// apply the specified filter
		filter->apply(*img, imgView, imgMap);

		// smooth output if requested
		for (int i = 0; i < numBlurPasses; i++)
			cvSmooth(imgMap.getIplImage(), imgMap.getIplImage());

		// threshold output map: if src(x,y) < threshold src(x,y) = 0
		if (thresholdSalience > 0.0)
			cvThreshold(imgMap.getIplImage(), imgMap.getIplImage(), thresholdSalience, 0.0, CV_THRESH_TOZERO);
	    
		// most salient location
		int peakX = -1;
		int peakY = -1;
		float peakXNorm = -2.0f; // normalized to a range of [-1.0,1.0]
		float peakYNorm = -2.0f; // normalized to a range of [-1.0,1.0]
		float peakV = -1.0f;

		if (activateIOR){
			ior.applyIOR(imgMap);
		}

		//Always compute peak to stream peak coordinates (Alex, 31/05/08)
		getPeak(imgMap, peakX, peakY, peakV);
		// Compute normalized peak positions (Jonas, 09/07/09)
		peakXNorm = 2.0f*((float)peakX)/((float)imgMap.width()) - 1.0f;
		peakYNorm = 2.0f*((float)peakY)/((float)imgMap.height()) - 1.0f;

		if (activateIOR){
			ior.updateIOR(peakX, peakY);
		}

		// create image view from current image map
		drawRgbFromFloat(imgMap, imgView, 1.0f);
		//imgView.zero();

		// draw crosshair at most salient location to imgView if requestd
		if (drawSaliencePeak){
			if( peakX >= 0 && peakY >= 0)
			{
				PixelRgb pix = PixelRgb(255,255,255);
				yarp::sig::draw::addCrossHair(imgView, pix, peakX, peakY, 10);
			}
		}

		mutex.post();

		// write peak to port (Alex, 31/05/08)
		Bottle &peak = peakPort.prepare();
		peak.clear();
		peak.addInt(peakX);
		peak.addInt(peakY);
		// add normalized peak positions (Jonas, 09/07/09)
		peak.addDouble(peakXNorm);
		peak.addDouble(peakYNorm); 
		peakPort.write();

		// write images to ports
		filteredPort.write();
		imgPort.write();

		oldSizeX = img->width();
		oldSizeY = img->height();
	}	

    return true;
}


double SalienceModule::getSalienceThreshold(){
    return thresholdSalience;
}

bool SalienceModule::setSalienceThreshold(double thr){
    thresholdSalience = thr;
    return true;
}

int SalienceModule::getNumBlurPasses(){
    return numBlurPasses;
}

bool SalienceModule::setNumBlurPasses(int num){
    numBlurPasses = num;
    return true;
}   

//int SalienceModule::getTemporalBlur(){
//    return temporalBlurSteps;
//}

//bool SalienceModule::setTemporalBlur(int size){
//    setTemporalBufferSize(size);
//    temporalBlurSteps = size;
//    return true;
//}

bool SalienceModule::respond(const Bottle &command,Bottle &reply){
        
    bool ok = false;
    bool rec = false; // is the command recognized?

    mutex.wait();
    switch (command.get(0).asVocab()) {
	case SALIENCE_VOCAB_HELP:
		rec = true;
		{
			reply.addString("help");

			reply.addString("\n");
			reply.addString("get fn \t: get the name of the top filter in the filter hierarchy");
			reply.addString("get nb \t: get the number of blur passes (gaussian 3x3) applied to the final salience map");
			reply.addString("get th \t: get the threshold applied to the final salience map");
			reply.addString("get cn <i> \t: get the name of the i-th sub-filter");
			reply.addString("get cw <i> \t: get the weight of the i-th sub-filter");
			reply.addString("get cc \t: get the number of sub-filters");
			reply.addString("get cws \t: get the weights of all the sub-filters");

			reply.addString("\n");
			reply.addString("set fn <s> \t: set the name of the top-filter ((string)s)");
			reply.addString("set nb \t: set the number of blur passes (gaussian 3x3) applied to the final salience map (int[>=0])");
			reply.addString("set th \t: set the threshold applied to the final salience map (int[0-255])");
			reply.addString("set w <d> \t: set the weight of the top-filter");
			reply.addString("set cn <i> <s> \t: set the name of the i-th sub-filter ((string)s)");
			reply.addString("set cw <i> <d> \t: set the weight of the i-th sub-filter ((double)d[0.0-1.0])");
			reply.addString("set cws <d> ... <d> \t: set weights to all the sub-filters ((double)d[0.0-1.0])");

			reply.addString("\n");
			reply.addString("Experimental Syntax:");
			reply.addString("fn <topFilterName>.<subFilterName> set w <double> \t: set the weighting of a filter (double[0.0-1.0])");
			reply.addString("fn <topFilterName>.<subFilterName> get w <double> \t: get the weighting of a filter (double[0.0-1.0])");

			ok = true;
		}
		break;
    case SALIENCE_VOCAB_NAME:
        rec = true;
        {
            // check and change filter name to pass on to the next filter
            string fName(command.get(1).asString());
            string subName;
            Bottle subCommand;
            int pos = fName.find_first_of(filter->getFilterName());
            if (pos == 0){
                pos = fName.find_first_of('.');
                if (pos  > -1){ // there is a subfilter name
                    subName = fName.substr(pos + 1, fName.size()-1);
                    subCommand.add(command.get(0));
                    subCommand.add(Value(subName.c_str()));
                }
                for (int i = 2; i < command.size(); i++)
                    subCommand.add(command.get(i));
                ok = filter->respond(subCommand, reply);
            }
            else{
                cout << "filter name " << fName << " does not match top filter name " << filter->getFilterName() << endl;
                ok = false;
            }
        }
        break;
    case SALIENCE_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case SALIENCE_VOCAB_SALIENCE_THRESHOLD:{
                double thr = command.get(2).asDouble();
                ok = this->setSalienceThreshold(thr);
            }
                break;
            case SALIENCE_VOCAB_NUM_BLUR_PASSES:{
                int nb = command.get(2).asInt();
                ok = this->setNumBlurPasses(nb);
            }
                break;
            /*case SALIENCE_VOCAB_TEMPORAL_BLUR:{
                int size = command.get(2).asInt();
                ok = this->setTemporalBlur(size);
            }*/
                break;
            case SALIENCE_VOCAB_NAME:{
                string s(command.get(2).asString().c_str());
                ok = filter->setFilterName(s);
            }
                break;
            case SALIENCE_VOCAB_CHILD_NAME:{
                int j = command.get(2).asInt();
                string s(command.get(3).asString().c_str());
                ok = filter->setChildFilterName(j,s);
            }
                break;
            case SALIENCE_VOCAB_WEIGHT:{
		        double w = command.get(2).asDouble();
                ok = filter->setWeight(w);
            }
                break;
            case SALIENCE_VOCAB_CHILD_WEIGHT:{
                int j = command.get(2).asInt();
                double w = command.get(3).asDouble();
                ok = filter->setChildWeight(j,w);
            }
                break;
            case SALIENCE_VOCAB_CHILD_WEIGHTS:{
                Bottle weights;
                for (int i = 2; i < command.size(); i++)
                    weights.addDouble(command.get(i).asDouble());
                ok = filter->setChildWeights(weights);
            }
                break;
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_SET" << endl;
                break;
            }
        }
        break;
    case SALIENCE_VOCAB_GET:
        rec = true;
        {
            reply.addVocab(SALIENCE_VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case SALIENCE_VOCAB_SALIENCE_THRESHOLD:{
                double thr = this->getSalienceThreshold();
                reply.addDouble(thr);
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_NUM_BLUR_PASSES:{
                int nb = this->getNumBlurPasses();
                reply.addInt(nb);
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_NAME:{
                string s(filter->getFilterName());
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_CHILD_NAME:{
                int j = command.get(2).asInt();
                string s(filter->getChildFilterName(j));
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_CHILD_COUNT:{
                int count = filter->getChildCount();
                reply.addInt(count);
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_WEIGHT:{
                double w = filter->getWeight();
                reply.addDouble(w);
		        ok = true;
            }
                break;
            case SALIENCE_VOCAB_CHILD_WEIGHT:{
                int j = command.get(2).asInt();
                double w = filter->getChildWeight(j);
                reply.addDouble(w);
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_CHILD_WEIGHTS:{
                Bottle weights;
                ok = filter->getChildWeights(&weights);
                for (int k = 0; k < weights.size(); k++)
                    reply.addDouble(weights.get(k).asDouble());
            }
                break;
		    default:
                cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(SALIENCE_VOCAB_FAILED);
    }
    else
        reply.addVocab(SALIENCE_VOCAB_OK);

    return ok;
} 	



//void SalienceModule::resizeBufferedImages(int w, int h){
//    for (int i = 0; i < tmpBuffer.size(); i++)
//        tmpBuffer[i]->resize(w,h);
//}

void SalienceModule::getPeak(ImageOf<PixelFloat> &img, int &i, int &j, float &v){
    //v = -FLT_MAX;
    v = 0.0f;
    //i = img.width()/2;
    //j = img.height()/2;
	// i and j should be left invalid if there is no clear peak (Alex 31/05/08)
	i = -1;
	j = -1; 
    IplImage *image = (IplImage*)img.getIplImage();
    float *data = (float*)image->imageData;
    for (int y = 0; y < image->height; y++){
        for (int x = 0; x < image->width; x++){
            if (v < *data){
                v = *data;
                i = x;
                j = y;
            }
            data++;
        }
    }
}

void SalienceModule::drawRgbFromFloat(ImageOf<PixelFloat> &imgFloat, 
                                      ImageOf<PixelRgb> &imgRgb, 
                                      float scale){
    // TODO make faster
    float flValue = 0.0f;
    IMGFOR(imgFloat,x,y) {
        flValue = scale * imgFloat(x,y);
        if (flValue >= 0.0f){
            if (flValue > 255.0f){
                flValue = 255.0f;
                //cout << "salience value > 255.0: " << flValue << endl;
            }
            imgRgb(x,y).r = (uchar)0;
            imgRgb(x,y).g = (uchar)flValue;
            imgRgb(x,y).b = (uchar)0;
        }
        else{
            //cout << "salience value < 0: " << flValue << endl;
            imgRgb(x,y).r = (uchar)0;
            imgRgb(x,y).g = (uchar)0;//flValue;
            imgRgb(x,y).b = (uchar)0;
        }
    }             
}
