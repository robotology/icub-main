/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrea Del Prete, Alexander Schmitz
 * email:   andrea.delprete@iit.it, alexander.schmitz@iit.it
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
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include "math.h"
#include <algorithm>
#include "iCub/skinDriftCompensation/Compensator.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDriftCompensation;
using namespace iCub::skinDynLib;

Compensator::Compensator(string name, string robotName, string outputPortName, string inputPortName, BufferedPort<Bottle>* _infoPort, 
                         double _compensationGain, double _contactCompensationGain, int addThreshold, float _minBaseline, bool _zeroUpRawData, 
                         bool _binarization, bool _smoothFilter, float _smoothFactor, unsigned int _linkNum)
									   : 
										compensationGain(_compensationGain), contactCompensationGain(_contactCompensationGain),
                                            addThreshold(addThreshold), infoPort(_infoPort),
                                            minBaseline(_minBaseline), binarization(_binarization), smoothFilter(_smoothFilter), 
                                            smoothFactor(_smoothFactor), robotName(robotName), name(name), linkNum(_linkNum)
{
    this->zeroUpRawData = _zeroUpRawData;
    _isWorking = init(name, robotName, outputPortName, inputPortName);
}

Compensator::~Compensator(){
    if(tactileSensorDevice){
		tactileSensorDevice->close();
        delete tactileSensorDevice;
    }

    compensatedTactileDataPort.interrupt();
    compensatedTactileDataPort.close();
}

bool Compensator::init(string name, string robotName, string outputPortName, string inputPortName){
    skinPart = UNKNOWN_SKIN_PART;
    bodyPart = UNKNOWN_BODY_PART;

    if (!compensatedTactileDataPort.open(outputPortName.c_str())) {
	    stringstream msg; msg<< "Unable to open output port "<< outputPortName;
        sendInfoMsg(msg.str());
	    return false;  // unable to open
    }

    Property options;
    stringstream localPortName;
    localPortName<< "/"<< name<< "/input";
    options.put("robot",  robotName.c_str());
    options.put("local",  localPortName.str().c_str());
    options.put("remote",  inputPortName.c_str());	
    options.put("device", "analogsensorclient");
     
    // create a new device driver
    tactileSensorDevice = new PolyDriver(options);
    if (!tactileSensorDevice->isValid()){
	    printf("Device not available.  Here are the known devices:\n");
	    printf("%s", Drivers::factory().toString().c_str());
	    return false;
    }
    // open the sensor interface	
    bool ok = tactileSensorDevice->view(tactileSensor);
    if (!ok) {
	    printf("Problems acquiring interfaces\n");
	    return false;
    }
    
    skinDim = tactileSensor->getChannels();
    if(skinDim<=0){
		fprintf(stderr, "Error while reading the number of channels of the tactile sensor device. Using 192 as default value.\n");
		skinDim = 192;
	}
    readErrorCounter = 0;
    rawData.resize(skinDim);
    baselines.resize(skinDim);
    touchThresholds.resize(skinDim);
    touchDetected.resize(skinDim);
    subTouchDetected.resize(skinDim);
    touchDetectedFilt.resize(skinDim);
    compensatedData.resize(skinDim);
    compensatedDataOld.resize(skinDim);    
    taxelPos.resize(skinDim, zeros(3));
    taxelOri.resize(skinDim, zeros(3));
    // by default every taxel is neighbor with all the other taxels
    vector<int> defaultNeighbors(skinDim);
    for(unsigned int i=0;i<skinDim;i++) defaultNeighbors[i]=i;
    neighborsXtaxel.resize(skinDim, defaultNeighbors);    

    // test read to check if the skin is broken (all taxel output is 0)
    if(readInputData(compensatedData)){
        bool skinBroken = true;
        for(unsigned int i=0; i<skinDim; i++){
            if(compensatedData[i]!=0){
                skinBroken = false;
                break;
            }
        }
        if(skinBroken)
            sendInfoMsg("The output of all the taxels is 0. Probably there is a hardware problem.");
        return !skinBroken;
    }

    return true;
}

void Compensator::calibrationInit(){   
	// take the semaphore so that the touchThreshold can't be read during the calibration phase
	touchThresholdSem.wait();

	// send a command to the microcontroller for calibrating the skin sensors
	if(robotName!="icubSim"){	// this feature isn't implemented in the simulator and causes a runtime error
		tactileSensor->calibrateSensor();
	}

	// initialize
    calibrationRead = 0;
    saturatedTaxels.resize(0);
	start_sum.assign(skinDim, 0);
	skin_empty.assign(skinDim, vector<int>(MAX_SKIN+1, 0));
}

void Compensator::calibrationDataCollection(){	
    Vector skin_values(skinDim);
    if(!readInputData(skin_values))
        return;
	calibrationRead++;
		
	for (unsigned int j=0; j<skinDim; j++) {
		if (zeroUpRawData==false)
			skin_values[j] = MAX_SKIN - skin_values[j];
		
		if(skin_values[j]<0 || skin_values[j]>MAX_SKIN){
			fprintf(stderr, "Error while reading the tactile data! Data out of range: %d\n", (int)skin_values[j]);
		}
		else{
			skin_empty[j][int(skin_values[j])]++;
			start_sum[j] += int(skin_values[j]);
		}
	}	
}

void Compensator::calibrationFinish(){
	
	//vector<float> standard_dev(skinDim, 0);
	//get percentile
	for (unsigned int i=0; i<skinDim; i++) {
		//avg start value
		baselines[i] = start_sum[i]/calibrationRead;
		
		//cumulative values
		for (int j=1; j<=MAX_SKIN; j++) {
			//standard_dev[i] += fabs(j-baselines[i]) * skin_empty[i][j];
			skin_empty[i][j] += skin_empty[i][j-1] ;			
		}
		//standard_dev[i] /= (CAL_TIME*PERIOD);

		//when do we cross the threshold?
		for (int j=0; j<=MAX_SKIN; j++) {
			if (skin_empty[i][j] > (calibrationRead*0.95)) {
                // the threshold can not be less than MIN_TOUCH_THR
				touchThresholds[i] = (double)j - baselines[i];
				j = MAX_SKIN;   // break
			}
		}
	}
    // store the initial baseline to compute the drift compensated later
    initialBaselines = baselines;
	
	// set the "old output value" for the smoothing filter to the baseline value to get a smooth start
	compensatedDataOld = baselines;	

    // test to check if the skin is broken (all taxel baselines are 255 OR thresholds are 0)
    bool baseline255 = true, thresholdZero = true;
    for(unsigned int i=0; i<skinDim; i++){
        if(baselines[i]!=255){
            baseline255 = false;
        }
        if(touchThresholds[i]>0.00001){
            thresholdZero = false;
        }
    }
    if(baseline255 || thresholdZero){
        _isWorking = false;
        sendInfoMsg("Eithre the baselines of all the taxels are 255 or the noises are 0. Probably there is a hardware problem.");
    }

    // print to console
    if(_isWorking){
	    fprintf(stderr, "\n[%s] Baselines:\n", name.c_str());
	    for (unsigned int i=0; i<skinDim; i++) {
		    if(!(i%12)) fprintf(stderr, "\n");
		    fprintf(stderr,"%4.1f ", baselines[i]);		
	    }
	    fprintf(stderr,"\n[%s] Thresholds (95 percentile):\n", name.c_str());
	    for (unsigned int i=0; i<skinDim; i++) {
		    if(!(i%12)) fprintf(stderr, "\n");
            touchThresholds[i] = max<double>(MIN_TOUCH_THR, touchThresholds[i]);
		    fprintf(stderr,"%3.1f ", touchThresholds[i]);		
	    }
	    fprintf(stderr,"\n");
    }

    // release the semaphore so that as of now the touchThreshold can be read
	touchThresholdSem.post();
}

bool Compensator::readInputData(Vector& skin_values){
    int err;
    if((err=tactileSensor->read(skin_values))!=IAnalogSensor::AS_OK){
        readErrorCounter++;        

        stringstream msg;
        if(err == IAnalogSensor::AS_TIMEOUT)            
            msg<< "Timeout error reading tactile sensor.";
        else if(err == IAnalogSensor::AS_OVF)
            msg<< "Ovf error reading tactile sensor.";
        else if(err == IAnalogSensor::AS_ERROR)
            msg<< "Generic error reading tactile sensor.";
        sendInfoMsg(msg.str());

        if(readErrorCounter>MAX_READ_ERROR){
            _isWorking = false;
            sendInfoMsg("Too many errors in a row. Stopping the compensator.");
        }
	    return false;
    }

    if(skin_values.size() != skinDim){
        readErrorCounter++;        

        stringstream msg;
        msg<< "Unexpected size of the input array (raw tactile data): "<< skin_values.size();
        sendInfoMsg(msg.str());

        if(readErrorCounter>MAX_READ_ERROR){
            _isWorking = false;
            sendInfoMsg("Too many errors in a row. Stopping the compensator.");
        }
        return false;
    }
    

    readErrorCounter = 0;
    return true;
}

bool Compensator::readRawAndWriteCompensatedData(){    
    if(!readInputData(rawData))
        return false;
	
	Vector& compensatedData2Send = compensatedTactileDataPort.prepare();
    compensatedData2Send.clear();   // local variable with data to send
	compensatedData.clear();        // global variable with data to store
	
	double d;
	for(unsigned int i=0; i<skinDim; i++){
	    // baseline compensation
	    if( zeroUpRawData == false){
		    d = (double)( MAX_SKIN - rawData(i) - baselines[i]);
	    }else{
		    d = (double)(rawData(i) - baselines[i]);
	    }
	    compensatedData.push_back(d);	// save the data before applying filtering

        // detect touch (before applying filtering, so the compensation algorithm is not affected by the filters)
        if(d > touchThresholds[i] + addThreshold){
		    touchDetected[i] = true;
	    }else{
		    touchDetected[i] = false;
        }

        // detect subtouch
        if(d < -touchThresholds[i] - addThreshold){
		    subTouchDetected[i] = true;
	    }else{
		    subTouchDetected[i] = false;
        }

        // smooth filter
        if(smoothFilter){
		    smoothFactorSem.wait();
		    d = (1-smoothFactor)*d + smoothFactor*compensatedDataOld(i);
		    smoothFactorSem.post();
		    compensatedDataOld(i) = d;	// update old value
	    }

	    // binarization filter
        // here we don't use the touchDetected array because, if the smooth filter is on,
        // we want to use the filtered values
        if(d > touchThresholds[i] + addThreshold){
            touchDetectedFilt[i] = true;
            if(binarization)
		        d = BIN_TOUCH;
        }else{
            touchDetectedFilt[i] = false;
            if(binarization)
	            d = BIN_NO_TOUCH;
        }
        
      //  if(d<0) // if negative, set it to zero
		    //d=0;
	    compensatedData2Send.push_back(d);
	}

	compensatedTactileDataPort.write();
	return true;
}

void Compensator::updateBaseline(){
	double mean_change = 0, change, gain;
    unsigned int non_touching_taxels = 0;
	double d; 

    for(unsigned int j=0; j<skinDim; j++) {
        // *** Algorithm 1
        /*if(!touchDetected[j]){
		    if(d > 0.5) {
			    baselines[j]		+= compensationGain;
			    mean_change			+= compensationGain;				//for changing the taxels where we detected touch
		    }else if(d < -0.5) {
			    baselines[j]		-= compensationGain;
			    mean_change			-= compensationGain;				//for changing the taxels where we detected touch
		    }
        }*/

        // *** Algorithm 2
        /*if(!(touchDetected[j] || subTouchDetected[j])){
			non_touching_taxels++;										//for changing the taxels where we detected touch
			d = compensatedData(j);
			if(fabs(d)>0.5){
                change          = (compensationGain/50)*d/touchThresholds[j];
				baselines[j]    += change;
                mean_change     += change;
			}
		}*/

        d = compensatedData(j);
		if(fabs(d)>0.5){
            if(touchDetected[j]){
                gain            = contactCompensationGain/50;                
            }else{
                gain            = compensationGain/50;                
                non_touching_taxels++;
            }
            change          = gain*d/touchThresholds[j];
			baselines[j]    += change;
            mean_change     += change;

            if(baselines[j]<0){
                printf("port %s; tax %d; baseline %.2f; gain: %.4f; d: %.2f; raw: %.2f; change: %f; touchThr: %.2f\n", SkinPart_s[skinPart].c_str(), j, baselines[j], 
                    gain, d, rawData[j], change, touchThresholds[j]);
            }
		}        
    }
    
    //for compensating the taxels where we detected touch
    if (non_touching_taxels>0 && non_touching_taxels<skinDim && mean_change!=0){
        mean_change /= non_touching_taxels;
        for(unsigned int j=0; j<skinDim; j++) {
            if (touchDetected[j]) {
                baselines[j]		+= mean_change;
                if(baselines[j]<0){
                    printf("2)tax %d; baseline %.2f; meanchange: %f; \n", j, baselines[j], mean_change);
                }
            }
        }
    }
}

bool Compensator::doesBaselineExceed(unsigned int &taxelIndex, double &baseline, double &initialBaseline){
    vector<unsigned int>::iterator it;
	for(unsigned int i=0; i<skinDim; i++){
	    if(baselines[i]<minBaseline || baselines[i]>MAX_SKIN-minBaseline){
            it = find(saturatedTaxels.begin(), saturatedTaxels.end(), i);
            if(it==saturatedTaxels.end()){  // if the taxel hasn't been already signalled
			    //fprintf(stderr, "Baseline %d exceeds: %f\n", i, baselines[i]);
                saturatedTaxels.push_back(i);
                baseline = baselines[i];
                initialBaseline = initialBaselines[i];
                taxelIndex = i;
			    return true;
            }
        }
	}
	return false;
}


skinContactList Compensator::getContacts(){    
    vector<int>         contactXtaxel(skinDim, -1);     // contact for each taxel (-1 means no contact)
    deque<deque<int>>   taxelsXcontact;                 // taxels for each contact
    int                 contactId = 0;                  // id of the next contact to create
    int                 neighCont;                      // id of the contact of the current neighbor

    for(unsigned int i=0; i<skinDim; i++){
        if(touchDetectedFilt[i] ){ // && contactXtaxel[i]<0 (second condition should always be true)
            vector<int> *neighbors = &(neighborsXtaxel[i]);
            //printf("Taxel %d active. Going to check its %d neighbors\n", i, neighbors->size());
            for(unsigned int n=0; n<neighbors->size(); n++){
                
                neighCont = contactXtaxel[(*neighbors)[n]];
                if(neighCont >= 0){                                     // ** if neighbor belongs to a contact
                    if(contactXtaxel[i]<0){                             // ** add taxel to pre-existing contact
                        contactXtaxel[i] = neighCont;
                        taxelsXcontact[neighCont].push_back(i);
                        //printf("Add taxel to pre existing contact %d (neighbor %d)\n", neighCont, (*neighbors)[n]);
                    }else if(contactXtaxel[i]!=neighCont){              // ** merge 2 contacts
                        //mergeContacts(contactXtaxel[i], neighCont);
                        int newId = min(contactXtaxel[i], neighCont);
                        int oldId = max(contactXtaxel[i], neighCont);
                        deque<int> tax2move = taxelsXcontact[oldId];
                        for(deque<int>::iterator it=tax2move.begin(); it!=tax2move.end(); it++){
                            contactXtaxel[(*it)] = newId;               // assign new contact id
                            taxelsXcontact[newId].push_back((*it));     // add taxel ids to contact
                        }
                        taxelsXcontact[oldId].clear();      // clear the list of taxels belonging to the merged contact
                        //printf("Merge two contacts: %d and %d\n", oldId, newId);
                    }
                }

            }
            if(contactXtaxel[i]<0){            // ** if no neighbor belongs to a contact -> create new contact
                contactXtaxel[i] = contactId;
                taxelsXcontact.resize(contactId+1);
                taxelsXcontact[contactId].push_back(i);
                contactId++;
                //printf("New contact created: %d\n", contactId-1);
            }
        }
    }
    //printf("Clustering finished\n");

    skinContactList contactList;
    Vector CoP(3);
    for( deque<deque<int>>::iterator it=taxelsXcontact.begin(); it!=taxelsXcontact.end(); it++){
        int activeTaxels = it->size();
        //printf("Contact size: %d\n", activeTaxels);
        if(activeTaxels==0) continue;        
        
        CoP.zero();
        for( deque<int>::iterator tax=it->begin(); tax!=it->end(); tax++){
            CoP = CoP+taxelPos[(*tax)];
        }
        CoP = CoP/activeTaxels;
        skinContact c(bodyPart, skinPart, linkNum, CoP, CoP, activeTaxels, 0.0);
        contactList.push_back(c);
    }


    // temporarily suppose there is only one contact
    /*Vector CoP(3);
    CoP.zero();
    int taxelsTouched = 0;
    for(int i=0; i!=touchDetectedFilt.size(); i++){
        if(touchDetectedFilt[i]){
	        taxelsTouched++;
            if(taxelPosOri != NULL){
	            CoP[0]+=taxelPosOri[i][0];
	            CoP[1]+=taxelPosOri[i][1];
	            CoP[2]+=taxelPosOri[i][2];
            }
        }
    }
    if(taxelsTouched>0){
        CoP[0]=CoP[0]/taxelsTouched;
        CoP[1]=CoP[1]/taxelsTouched;
        CoP[2]=CoP[2]/taxelsTouched;
        skinContact c(bodyPart, skinPart, linkNum, CoP, CoP, taxelsTouched, 0.0);
        contactList.push_back(c);
    }*/
    
    return contactList;
}

void Compensator::setBinarization(bool value){
	binarization = value;
}

void Compensator::setSmoothFilter(bool value){
	if(smoothFilter != value){
		smoothFilter = value;
		if(value){
			// set the old output value of the smooth filter to the last read, to get a smooth start
			compensatedDataOld = compensatedData;			
		}
	}
}
bool Compensator::setSmoothFactor(float value){
	if(value<0 || value>1)
		return false;
	if(value==1) 
		value = 0.99f;	// otherwise with 1 the values don't update
	smoothFactorSem.wait();
	smoothFactor = value;
	smoothFactorSem.post();
	return true;
}

void Compensator::setLinkNum(unsigned int linkNum){
    this->linkNum = linkNum;
}

void Compensator::setBodyPart(BodyPart _bodyPart){
    this->bodyPart = _bodyPart;
}

void Compensator::setSkinPart(SkinPart _skinPart){
    this->skinPart = _skinPart;
}

bool Compensator::setAddThreshold(unsigned int thr){
    if(thr>=MAX_SKIN)
        return false;
    addThreshold = thr;
    return true;
}

bool Compensator::setCompensationGain(double gain){
    if(gain<=0.0)
        return false;
    compensationGain = gain;
    return true;
}

bool Compensator::setContactCompensationGain(double gain){
    if(gain<0.0)
        return false;
    contactCompensationGain = gain;
    return true;
}

unsigned int Compensator::getNumTaxels(){
    if(_isWorking)
        return skinDim;
    return 0;
}

Vector Compensator::getTouchThreshold(){
	touchThresholdSem.wait();
	Vector res = touchThresholds;
	touchThresholdSem.post();
	return res;
}

string Compensator::getBodyPartName(){ return BodyPart_s[bodyPart];}

string Compensator::getSkinPartName(){ return SkinPart_s[skinPart];}
SkinPart Compensator::getSkinPart(){ return skinPart;}

Vector Compensator::getCompensation(){  return baselines-initialBaselines;}
Vector Compensator::getBaselines(){     return baselines;}
Vector Compensator::getRawData(){       return rawData;}
Vector Compensator::getCompData(){      return compensatedData;}

bool Compensator::getBinarization(){
	return binarization;
}
bool Compensator::getSmoothFilter(){
	return smoothFilter;
}

float Compensator::getSmoothFactor(){
    smoothFactorSem.wait();
    float res=smoothFactor;
    smoothFactorSem.post();
	return res;
}

unsigned int Compensator::getLinkNum(){
    return linkNum;
}

unsigned int Compensator::getAddThreshold(){
    return addThreshold;
}
double Compensator::getCompensationGain(){
    return compensationGain;
}
double Compensator::getContactCompensationGain(){
    return contactCompensationGain;
}
string Compensator::getName(){
    return name;
}

string Compensator::getInputPortName(){
    return tactileSensorDevice->getValue("remote").asString().c_str();
}

bool Compensator::isWorking(){
    return _isWorking;
}

bool Compensator::setTaxelPositions(const char *filePath){
	ifstream posFile;
	posFile.open(filePath);		
	if (!posFile.is_open())
        return false;

    string posLine;
    int totalLines = 0;
	while (getline(posFile,posLine)){
		posLine.erase(posLine.find_last_not_of(" \n\r\t")+1);
		if(!posLine.empty())totalLines++;
	}
	posFile.clear(); 
	posFile.seekg(0, std::ios::beg);//rewind iterator
    if(totalLines!=skinDim){
        fprintf(stderr, "Error while reading taxel position file %s: num of lines %d is not equal to num of taxels %d.\n", 
            filePath, totalLines, skinDim);
    }

	for(int i= 0; getline(posFile,posLine); i++) {
		posLine.erase(posLine.find_last_not_of(" \n\r\t")+1);
		if(posLine.empty())
			continue;
		string number;
		istringstream iss(posLine, istringstream::in);
		for(int j = 0; iss >> number; j++ ){
            if(j<3)
			    taxelPos[i][j] = strtod(number.c_str(),NULL);
            else
                taxelOri[i][j-3] = strtod(number.c_str(),NULL);
		}
	}
    computeNeighbors(0.01);

	return true;
}
void Compensator::computeNeighbors(double maxDist){
    neighborsXtaxel.clear();
    neighborsXtaxel.resize(skinDim, vector<int>(0));
    for(unsigned int i=0; i<skinDim; i++){
        for(unsigned int j=i+1; j<skinDim; j++){
            if( norm(taxelPos[i]-taxelPos[j]) <= maxDist){
                neighborsXtaxel[i].push_back(j);
                neighborsXtaxel[j].push_back(i);
                //printf("Taxels %d (%s) and %d (%s) are neighbors\n", i, taxelPos[i].toString().c_str(), j, taxelPos[j].toString().c_str());
            }
        }
    }

    int minNeighbors=skinDim, maxNeighbors=0, ns;
    for(unsigned int i=0; i<skinDim; i++){
        ns = neighborsXtaxel[i].size();
        if(ns>maxNeighbors) maxNeighbors = ns;
        if(ns<minNeighbors) minNeighbors = ns;
        /*printf("\nTaxel %d neighbors are: ", i);
        for(unsigned int j=0; j<ns; j++)
            printf("%d ", neighborsXtaxel[i][j]);*/
    }
    //printf("Min neighbors: %d\nMax neighbors: %d\n", minNeighbors, maxNeighbors);
}
void Compensator::sendInfoMsg(string msg){
    printf("\n");
    printf("[%s]: %s", getInputPortName().c_str(), msg.c_str());
    Bottle& b = infoPort->prepare();
    b.clear();
    b.addString(getInputPortName().c_str());
    b.addString((": " + msg).c_str());
    infoPort->write();
}

