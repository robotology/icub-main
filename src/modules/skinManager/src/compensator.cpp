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
#include <yarp/math/Rand.h> // TEMP
#include "math.h"
#include <algorithm>
#include "iCub/skinManager/compensator.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinManager;
using namespace iCub::skinDynLib;

Compensator::Compensator(string _name, string _robotName, string outputPortName, string inputPortName, BufferedPort<Bottle>* _infoPort, 
                         double _compensationGain, double _contactCompensationGain, int addThreshold, float _minBaseline, bool _zeroUpRawData, 
                         bool _binarization, bool _smoothFilter, float _smoothFactor, unsigned int _linkNum)
									   : 
										compensationGain(_compensationGain), contactCompensationGain(_contactCompensationGain),
                                            addThreshold(addThreshold), infoPort(_infoPort),
                                            minBaseline(_minBaseline), binarization(_binarization), smoothFilter(_smoothFilter), 
                                            smoothFactor(_smoothFactor), robotName(_robotName), name(_name), linkNum(_linkNum)
{
    this->zeroUpRawData = _zeroUpRawData;
    _isWorking = init(_name, _robotName, outputPortName, inputPortName);
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
	    printf("Device not available.\n");
	    //printf("%s", Drivers::factory().toString().c_str());
	    return false;
    }
    // open the sensor interface	
    bool ok = tactileSensorDevice->view(tactileSensor);
    if (!ok) {
	    printf("Problems acquiring interfaces\n");
	    return false;
    }
    
    int getChannelsCounter = 0;
    skinDim = tactileSensor->getChannels();
    while(skinDim<=0){
        // getChannels() returns 0 if it hasn't performed the first reading yet
        // so wait for 1/50 sec, then try again        
        if(++getChannelsCounter==50){
            // after 50 failed tries give up and use the default value
            skinDim = 192;
            sendInfoMsg("Error reading the number of channels of the port. Using 192 as default value.");
            break;
        }
        Time::delay(0.02);
        skinDim = tactileSensor->getChannels();
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
    maxNeighDist = MAX_NEIGHBOR_DISTANCE;
    // by default every taxel is neighbor with all the other taxels
    list<int> defaultNeighbors(skinDim);
    int i=0;
    for(list<int>::iterator it=defaultNeighbors.begin();it!=defaultNeighbors.end();it++, i++) 
        *it = i;
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
        sendInfoMsg("Either the baselines of all the taxels are 255 or the noises are 0. Probably there is a hardware problem.");
    }
    for (unsigned int i=0; i<skinDim; i++) 
        touchThresholds[i] = max<double>(MIN_TOUCH_THR, touchThresholds[i]);

    // print to console
    /*if(_isWorking){
	    printf("\n[%s (%s)] Baselines:\n", name.c_str(), getSkinPartName().c_str());
	    for (unsigned int i=0; i<skinDim; i++) {
		    if(!(i%12)) fprintf(stderr, "\n");
		    fprintf(stderr,"%5.1f ", baselines[i]);		
	    }
	    printf("\n[%s (%s)] Thresholds (95 percentile):\n", name.c_str(), getSkinPartName().c_str());
	    for (unsigned int i=0; i<skinDim; i++) {
		    if(!(i%12)) fprintf(stderr, "\n");
            touchThresholds[i] = max<double>(MIN_TOUCH_THR, touchThresholds[i]);
		    fprintf(stderr,"%3.1f ", touchThresholds[i]);		
	    }
	    printf("\n");
    }*/
    sendInfoMsg("Calibration finished");

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
                char* temp = new char[300];
                sprintf(temp, "ERROR-Negative baseline. Port %s; tax %d; baseline %.2f; gain: %.4f; d: %.2f; raw: %.2f; change: %f; touchThr: %.2f", 
                    SkinPart_s[skinPart].c_str(), j, baselines[j], gain, d, rawData[j], change, touchThresholds[j]);
                sendInfoMsg(temp);
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
                    char* temp = new char[300];
                    sprintf(temp, "ERROR-Negative baseline. Taxel %d; baseline %.2f; meanchange: %f", j, baselines[j], mean_change);
                    sendInfoMsg(temp);
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
    deque<deque<int> >  taxelsXcontact;                 // taxels for each contact
    int                 contactId = 0;                  // id of the next contact to create
    int                 neighCont;                      // id of the contact of the current neighbor

    poseSem.wait();
    {
        for(unsigned int i=0; i<skinDim; i++){
            if(touchDetectedFilt[i] ){ // && contactXtaxel[i]<0 (second condition should always be true)
                list<int> *neighbors = &(neighborsXtaxel[i]);
                //printf("Taxel %d active. Going to check its %d neighbors\n", i, neighbors->size());
                for(list<int>::iterator it=neighbors->begin(); it!=neighbors->end(); it++){
                    neighCont = contactXtaxel[(*it)];
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
    }
    poseSem.post();
    //printf("Clustering finished\n");

    skinContactList contactList;
    Vector CoP(3), geoCenter(3), normal(3);
    double pressure;
    int activeTaxels;
    vector<unsigned int> taxelList;
    for( deque<deque<int> >::iterator it=taxelsXcontact.begin(); it!=taxelsXcontact.end(); it++){
        activeTaxels = it->size();
        //printf("Contact size: %d\n", activeTaxels);
        if(activeTaxels==0) continue;
        
        taxelList.resize(activeTaxels);
        CoP.zero();
        geoCenter.zero();
        normal.zero();
        pressure = 0.0;
        int i=0;
        for( deque<int>::iterator tax=it->begin(); tax!=it->end(); tax++, i++){
            CoP         += taxelPos[(*tax)] * compensatedData[(*tax)];
            normal      += taxelOri[(*tax)] * compensatedData[(*tax)];
            geoCenter   += taxelPos[(*tax)];
            pressure    += max(compensatedData[(*tax)], 0.0);
            taxelList[i] = *tax;
        }
        if(pressure!=0.0){
        	CoP         /= pressure;
            normal      /= pressure;
        }
        geoCenter   /= activeTaxels;
        pressure    /= activeTaxels;
        skinContact c(bodyPart, skinPart, linkNum, CoP, geoCenter, taxelList, pressure, normal);
        contactList.push_back(c);
    }
    //printf("ContactList: %s\n", contactList.toString().c_str());
    
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
BodyPart Compensator::getBodyPart(){ return bodyPart; }
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
Vector Compensator::getTaxelPosition(unsigned int taxelId){
    if(taxelId>=skinDim)
        return zeros(3);
    poseSem.wait();
    Vector res = taxelPos[taxelId];
    poseSem.post();
    return res;
}
vector<Vector> Compensator::getTaxelPositions(){
    poseSem.wait();
    vector<Vector> res = taxelPos;
    poseSem.post();
    return res;
}
Vector Compensator::getTaxelOrientation(unsigned int taxelId){
    if(taxelId>=skinDim)
        return zeros(3);
    poseSem.wait();
    Vector res = taxelOri[taxelId];
    poseSem.post();
    return res;
}
vector<Vector> Compensator::getTaxelOrientations(){
    poseSem.wait();
    vector<Vector> res = taxelOri;
    poseSem.post();
    return res;
}
Vector Compensator::getTaxelPose(unsigned int taxelId){
    if(taxelId>=skinDim)
        return zeros(6);
    poseSem.wait();
    Vector res = cat(taxelPos[taxelId], taxelOri[taxelId]);
    poseSem.post();
    return res;
}
vector<Vector> Compensator::getTaxelPoses(){
    vector<Vector> res(skinDim);
    poseSem.wait();
    for(unsigned int i=0; i<skinDim; i++){
        res[i] = cat(taxelPos[i], taxelOri[i]);
    }
    poseSem.post();
    return res;
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

bool Compensator::setMaxNeighborDistance(double d){
    if(d<0.0)
        return false;
    maxNeighDist = d;
    return true;
}

bool Compensator::setTaxelPosesFromFile(const char *filePath){
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
        char* temp = new char[200];
        sprintf(temp, "Error while reading taxel position file %s: num of lines %d is not equal to num of taxels %d.\n", 
            filePath, totalLines, skinDim);
        sendInfoMsg(temp);
    }
    poseSem.wait();
    {
	    for(unsigned int i= 0; getline(posFile,posLine); i++) {
		    posLine.erase(posLine.find_last_not_of(" \n\r\t")+1);
		    if(posLine.empty())
			    continue;
		    string number;
		    istringstream iss(posLine, istringstream::in);
		    for(unsigned int j = 0; iss >> number; j++ ){
                if(i<taxelPos.size()){
                    if(j<3)
			            taxelPos[i][j] = strtod(number.c_str(),NULL);
                    else
                        taxelOri[i][j-3] = strtod(number.c_str(),NULL);
                }
		    }
	    }
        computeNeighbors();
    }
    poseSem.post();

	return true;
}
bool Compensator::setTaxelPoses(vector<Vector> poses){
    if(poses.size()!=skinDim)
        return false;
    poseSem.wait();
    {
        for(unsigned int i=0; i<skinDim; i++){
            taxelPos[i] = poses[i].subVector(0,2);
            taxelOri[i] = poses[i].subVector(3,5);
        }
    }
    computeNeighbors();
    poseSem.post();
    return true;
}
bool Compensator::setTaxelPose(unsigned int taxelId, Vector pose){
    if(taxelId>=skinDim || pose.size()!=6)
        return false;
    poseSem.wait();
    {
        taxelPos[taxelId] = pose.subVector(0,2);
        taxelOri[taxelId] = pose.subVector(3,5);
    }
    updateNeighbors(taxelId);
    poseSem.post();
    return true;
}
bool Compensator::setTaxelPositions(vector<Vector> positions){
    if(positions.size()!=skinDim)
        return false;
    poseSem.wait();
    taxelPos = positions;
    computeNeighbors();
    poseSem.post();
    return true;
}
bool Compensator::setTaxelPosition(unsigned int taxelId, Vector position){
     if(taxelId>=skinDim || position.size()!=3)
        return false;
    poseSem.wait();
    taxelPos[taxelId] = position;
    updateNeighbors(taxelId);
    poseSem.post();
    return true;
}
bool Compensator::setTaxelOrientations(vector<Vector> orientations){
    if(orientations.size()!=skinDim)
        return false;
    poseSem.wait();
    taxelOri = orientations;
    poseSem.post();
    return true;
}
bool Compensator::setTaxelOrientation(unsigned int taxelId, Vector orientation){
     if(taxelId>=skinDim || orientation.size()!=3)
        return false;
    poseSem.wait();
    taxelOri[taxelId] = orientation;
    poseSem.post();
    return true;
}
void Compensator::computeNeighbors(){
    neighborsXtaxel.clear();
    neighborsXtaxel.resize(skinDim, list<int>(0));
    Vector v;
    double d2 = maxNeighDist*maxNeighDist;
    for(unsigned int i=0; i<skinDim; i++){
        for(unsigned int j=i+1; j<skinDim; j++){
            if(taxelPos[i][0]!=0.0 || taxelPos[i][1]!=0.0 || taxelPos[i][2]!=0.0){  // if the taxel exists
                v = taxelPos[i]-taxelPos[j];
                if( dot(v,v) <= d2){
                    neighborsXtaxel[i].push_back(j);
                    neighborsXtaxel[j].push_back(i);
                    //printf("Taxels %d (%s) and %d (%s) are neighbors\n", i, taxelPos[i].toString().c_str(), j, taxelPos[j].toString().c_str());
                }
            }
        }
    }

    int minNeighbors=skinDim, maxNeighbors=0, ns;
    for(unsigned int i=0; i<skinDim; i++){
        if(taxelPos[i][0]!=0.0 || taxelPos[i][1]!=0.0 || taxelPos[i][2]!=0.0){  // if the taxel exists
            ns = neighborsXtaxel[i].size();
            if(ns>maxNeighbors) maxNeighbors = ns;
            if(ns<minNeighbors) minNeighbors = ns;
            /*printf("\nTaxel %d neighbors are: ", i);
            for(unsigned int j=0; j<ns; j++)
                printf("%d ", neighborsXtaxel[i][j]);*/
        }
    }
    printf("[%s] min neighbors: %d; max neighbors: %d\n", getSkinPartName().c_str(), minNeighbors, maxNeighbors);
}
void Compensator::updateNeighbors(unsigned int taxelId){
    Vector v;
    double d2 = maxNeighDist*maxNeighDist;
    // remove all the neighbors of the taxel with id=taxelId
    neighborsXtaxel[taxelId].clear();

    for(unsigned int i=0; i<skinDim; i++){
        if(taxelPos[i][0]!=0.0 || taxelPos[i][1]!=0.0 || taxelPos[i][2]!=0.0){  // if the taxel exists
            // remove taxelId from the neighbor list (if there is)
            neighborsXtaxel[i].remove(taxelId);
            // check whether they are neighbors
            v = taxelPos[i]-taxelPos[taxelId];
            if( dot(v,v) <= d2){
                neighborsXtaxel[i].push_back(taxelId);
                neighborsXtaxel[taxelId].push_back(i);
            }
        }
    }
}
void Compensator::sendInfoMsg(string msg){
    printf("[%s]: %s\n", getInputPortName().c_str(), msg.c_str());
    Bottle& b = infoPort->prepare();
    b.clear();
    b.addString(getInputPortName().c_str());
    b.addString((": " + msg).c_str());
    infoPort->write();
}

