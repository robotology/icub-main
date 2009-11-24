#ifndef __IHA__WINDOW_ID_CALC__H__
#define __IHA__WINDOW_ID_CALC__H__

/*
 * Copyright (C) 2006 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
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

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <list>

namespace iCub {
	namespace iha {
		class WindowIDCalc;
	}
}
using namespace iCub::iha;


/** 
 * @ingroup icub_iha_ExperienceMetricSpace
 * \brief Calculate the Information Distance between two data streams over a given horizon.
 *
 * This class expects binned data to be entered in pairs (periodically) and will return on request the current information distance between the two pairs of data streams. It can also return other information such as the joint entropy.
 *  
 */
class iCub::iha::WindowIDCalc {
private:

	float LOG2;

	unsigned char numBins;

	// sumary information
	int *freqMatrix;
	int *freqMarginalX;
	int *freqMarginalY;
	int numSamples;

	struct DataPair {
		unsigned char a;
		unsigned char b;
	};
		
	list<DataPair> dataWindow;
	int windowSize;

public:
	// default Constructor
	WindowIDCalc() {
		//fprintf(stderr,"WindowIDCalc: default constructor\n");
	}
	// Constructor
	WindowIDCalc(const unsigned char numbins, const int window) {
		//fprintf(stderr,"WindowIDCalc: constructor numbins=%d, window=%d\n",numbins,window);
		numBins=numbins;
		windowSize=window;

		// Data will be stored in a moving window
		//dataWindow = new list<DataPair>;
		numSamples=0;

		// Summary data structure
		freqMatrix=new int[numbins*numbins];
		freqMarginalX=new int[numbins];
		freqMarginalY=new int[numbins];

		for (int i=0;i<numbins*numbins;i++) freqMatrix[i]=0;
		for (int i=0;i<numbins;i++) {
			freqMarginalX[i]=0;
			freqMarginalY[i]=0;
		}

		LOG2=(float) log(2);
	}
	~WindowIDCalc() {
		//fprintf(stderr,"WindowIDCalc: destructor\n");
		//delete dataWindow;
		delete [] freqMatrix;
		delete [] freqMarginalX;
		delete [] freqMarginalY;

	}

	// Add a new data line to the window
	void putNextDataPair(unsigned char x, unsigned char y){
		if (x>=numBins) {
			fprintf(stderr,"putNextDataPair: error x=%d out of range\n",x);
			x=numBins-1;
		}
		if (y>=numBins) {
			fprintf(stderr,"putNextDataPair: error y=%d out of range\n",y);
			y=numBins-1;
		}
		//fprintf(stderr,"putNextDataPair: x=%d y=%d\n",x,y);
		// store the data pair as an array
		DataPair datapair;
		datapair.a=x;
		datapair.b=y;
		// add to our data window
		dataWindow.push_back(datapair);

		// update the data structures
		freqMatrix[x*numBins+y]++;
		freqMarginalX[x]++;
		freqMarginalY[y]++;
		numSamples++;

		// move window along
		if (numSamples > windowSize) {
				// update the data structures
				datapair = dataWindow.front();

				freqMatrix[datapair.a*numBins+datapair.b]--;
				freqMarginalX[datapair.a]--;
				freqMarginalY[datapair.b]--;
				numSamples--;

				// remove the item
				dataWindow.pop_front();
				
		}
	}


	//---------------------------------------------------------------------------
	// To calculate information distance ...

	float getInformationDist() {
		// infodist(X,Y) = 2 * H(X,Y) - H(X) - H(Y)
		float val = 2 * getJointEntropy() - getEntropyX() - getEntropyY();
		/*if (isnan(val)) {
			fprintf(stderr,"WindowIDCalc::getInformationDist: Error val=nan\n");
		}*/
		return val;
	}


	float getJointEntropy() {
		// H(X,Y) =	- sum[p(x,y) * ln p(x,y)] over all x and y
		float val=0.0f;
		float p;
		for (unsigned int x=0;x<numBins;x++) {
			for (unsigned int y=0;y<numBins;y++) {
				p = (float) freqMatrix[x*numBins+y] / (float)numSamples;
				val -= p * getLog2(p);
			}
		}
		/*if (isnan(val)) {
			fprintf(stderr,"WindowIDCalc::getJointEntropy: Error val=nan numSamples=%d\n",numSamples);
		}*/
		return val;
	}

	float getEntropyX() {
		// H(X) = - sum[p(x) * ln p(x)] over all x
		//	where p(x) = marginalX / numSamples
		float val=0.0f;
		float p;
		for (unsigned int x=0;x<numBins;x++) {
				p = (float) freqMarginalX[x]/(float)numSamples;
				val -= p * getLog2(p);
		}
		/*if (isnan(val)) {
			fprintf(stderr,"WindowIDCalc::getEntropyX: Error val=nan numSamples=%d\n",numSamples);
		}*/
		return val;
	}

	float getEntropyY() {
		// H(Y) = - sum[p(y) * ln p(y)] over all y
		// p(x)=marginalY / numSamples
		float val=0.0f;
		float p;
		for (unsigned int y=0;y<numBins;y++) {
				p = (float) freqMarginalY[y]/(float)numSamples;
				val -= p * getLog2(p);
		}
		/*if (isnan(val)) {
			fprintf(stderr,"WindowIDCalc::getEntropyY: Error val=nan numSamples=%d\n",numSamples);
		}*/
		return val;
	}

	//---------------------------------------------------------------------------
	// get natural log
	float getLog2(float v) {
		if (v==0) return 0;
		return (float) (log((double) v) / LOG2 );
	}

	//----------------------------------------------------------------------------
	// some printing routines for testing
	void printFrequencyMatrix() {
		printf("Frequency Matrix\n");
		int total=0;
		for (unsigned int y=0;y<numBins;y++) {
			for (unsigned int x=0;x<numBins;x++) {
				printf("%d ",freqMatrix[x*numBins+y]);
				total+=freqMatrix[x*numBins+y];
			}
			printf("\n");
		}
		printf("Total %d\n",total);
	}

	void printProbMatrix() {
		printf("Prob Matrix\n");
		float total=0;
		for (unsigned int y=0;y<numBins;y++) {
			for (unsigned int x=0;x<numBins;x++) {
				printf("%d\t",((float) freqMatrix[x*numBins+y]/((float)numSamples)));
				total+=((float) freqMatrix[x*numBins+y]/((float)numSamples));
			}
			printf("\n");
		}
		printf("Total %d\n",total);
	}

	void printMarginalX() {
		printf("Marginal X\n");
		for (unsigned int x=0;x<numBins;x++) {
				printf("%d\t",freqMarginalX[x]);
		}
		printf("\n");
	}

	void printProbX() {
		printf("Prob X\n");
		for (unsigned int x=0;x<numBins;x++) {
				printf("%d\t",((float) freqMarginalX[x]/((float)numSamples)));
		}
		printf("\n");
	}

	void printMarginalY() {
		printf("Marginal Y\n");
		for (unsigned int y=0;y<numBins;y++) {
				printf("%d\t",freqMarginalY[y]);
		}
		printf("\n");
	}

	void printProbY() {
		printf("Prob Y\n");
		for (unsigned int y=0;y<numBins;y++) {
				printf("%d\t",((float) freqMarginalY[y]/((float)numSamples)));
		}
		printf("\n");
	}


};

#endif
