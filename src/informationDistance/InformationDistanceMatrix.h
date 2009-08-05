/*
 * Copyright (C) <2006> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Lars Olsson, and Hatice Kose-Bagci (University of Hertfordshire)
 * email:   h.kose-bagci@herts.ac.uk
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

/*
 *  Computes Information distance
 *
 */

#include "DistanceMatrix.h"
#include "BinWindowMaxEntropy.h"
/**
 * Calculates information distance
 */
class InformationDistanceMatrix : public DistanceMatrix {

protected:
	int **frequencyDist;
	int ****frequency2Dist;
	int numBins;
	int nb1;
	int ns1;

	bool adaptive_binning;
	BinWindowMaxEntropy* adaptiveBinners;
	int binning_window_size;
	int binning_histogram_size;

public:
#define  NATLOG_TO_LOG2  1.4426950408889634

	InformationDistanceMatrix(int ns, int nb):DistanceMatrix (ns), adaptive_binning(false)
	{
		nb1 = nb;
		ns1 = ns;
		numBins = nb;
		timeSteps = 0;
		int i,j,k,l;

		frequencyDist = (int**)malloc(ns*sizeof(int*));
		for(i = 0; i<ns; i++)
			frequencyDist[i] = (int*)malloc(numBins*sizeof(int));

		frequency2Dist = (int****)malloc(ns*sizeof(int***));
		for(i = 0; i<ns; i++)
			frequency2Dist[i] = (int***)malloc(ns*sizeof(int**));
		for(i = 0; i<ns; i++)
			for(j = 0; j<ns; j++)
				frequency2Dist[i][j] = (int**)malloc(numBins*sizeof(int*));

		for(i = 0; i<ns; i++)
			for(j = 0; j<ns; j++)
				for(k = 0; k<numBins; k++)
					frequency2Dist[i][j][k] = (int*)malloc(numBins*sizeof(int));

		for(i = 0; i<ns; i++)
			for(k = 0; k<numBins; k++)
					frequencyDist[i][k] = 0;


		for(i = 0; i<ns; i++)
			for(j = 0; j<ns; j++)
				for(k = 0; k<numBins; k++)
					for(l = 0; l<numBins; l++)
						frequency2Dist[i][j][k][l] = 0;

	}

	void setAdaptiveBinning(int ws, int hs) {

		binning_window_size = ws;
		binning_histogram_size = hs;
		adaptive_binning = true;

		if (adaptive_binning) {
			adaptiveBinners = new BinWindowMaxEntropy[ns1];
			for (int i=0;i<ns1;i++) {
				adaptiveBinners[i].init(numBins, binning_window_size, binning_histogram_size);
			}
		}
	}

	void removeAll(int ns, int nb){

	int i,j,k;

		for(i = 0; i<ns; i++)
			for(j = 0; j<ns; j++)
				for(k = 0; k<nb; k++)
					free(frequency2Dist[i][j][k]);

		for(i = 0; i<ns; i++)
			for(j = 0; j<ns; j++)
				free(frequency2Dist[i][j]);

		for(i = 0; i<ns; i++)
			free(frequency2Dist[i]);

		free(frequency2Dist);

		for(i = 0; i<ns; i++)
			free(frequencyDist[i]);

		free(frequencyDist);

	}

	~InformationDistanceMatrix(){
		removeAll(ns1, nb1);

	}


protected:
   /**
	* Updates the distributions using sensor data
	* @param values - sensors values
	*/
	void updateDistributions(int values[]){
		for(int i = 0; i < numSensors;i++){
			frequencyDist[i][values[i]]++;

			for(int j = 0; j < numSensors;j++){
				frequency2Dist[i][j][values[i]][values[j]]++;
			}
		}
	}

public:

	void normaliseMatrix(){
		printf("normalise");
	}

   /**
	* Adds new coming integer data to the distribution
	* @param values - sensors values
	*/
	void addData(int *values){
		timeSteps++;

		updateDistributions(values);
	}

   /**
	* Updates the bins using sensor data
	* @param values - sensors values
	*/
	int * alphabetize(double *values){
		int *data = (int*)malloc(numSensors*sizeof(int));
		for(int i = 0; i < numSensors;i++){
			int alphabetized = (int)(values[i] * (numBins));
			if(alphabetized == numBins)
				alphabetized--;
			data[i] = alphabetized;
		}

		return data;
	}

   /**
	* Adds new coming double data to the distribution
	* @param values - sensors values
	*/
	void addData(double *values){
		timeSteps++;
		int *aValues;

		if (adaptive_binning) {
			aValues = new int [numSensors];
			for (int i=0;i<numSensors;i++) {
				adaptiveBinners[i].putNextData(values[i]);
				aValues[i] = adaptiveBinners[i].getBinMaxEntropy(values[i]);
			}
		} else {
			aValues=alphabetize(values);
		}
		updateDistributions(aValues);
	}


   /**
	* Adds new coming integer data to the distribution
	*/
	void updateMatrix(){
		for(int i = 0; i < numSensors;i++){
			for(int j = i; j < numSensors; j++){
				matrix[i][j] = getInformationDistance(i, j);
				matrix[j][i] = matrix[i][j];
			}
		}
	}

   /**
	* Updates the matrix using sensor data
	* @param values - sensors values
	*/
	void updateMatrix(double *values){
		DistanceMatrix::updateMatrix(values);
	}

   /**
	* Sets the frequency distance matrix
	* @param dist - distance matrix
	*/
	void setFrequencyDist(int **dist){
		frequencyDist = dist;
	}

   /**
	* Sets the two dimensional frequency distance matrix
	* @param dist - distance matrix
	*/
	void setFrequency2Dist(int ****dist){
		frequency2Dist = dist;
	}

   /**
	* Sets the time steps
	*/
	void setTimeSteps(int steps){
		timeSteps = steps;
	}

   /**
	* Calculates log base 2
	*/
	static double log2(double val){
		return (val != 0.0) ? (log(val) * NATLOG_TO_LOG2) : 1.0;
	}


   /**
	* H(X)
	*/
	double getEntropy(int x){
		double res = 0.0;
		for(int i = 0; i < numBins; i++){
			res += ((frequencyDist[x][i] / (double)timeSteps) *
				log2((frequencyDist[x][i] / (double)timeSteps)));
		}
		return -res;
	}

   /**
	* H(X,Y)
	*/
	double getJointEntropy(int x, int y){
		double res = 0.0;
		for(int i = 0; i < numBins; i++)
			for(int j = 0; j < numBins; j++){
				res += ((frequency2Dist[x][y][i][j] /(double)timeSteps) *
			log2((frequency2Dist[x][y][i][j] / (double)timeSteps)));
			}
		return -res;
	}

   /**
	* H(X|Y) + H(Y|X)
	*/
	double getInformationDistance(int x, int y){
		return getConditionalEntropy(x, y) + getConditionalEntropy(y, x);
	}

   /**
	* H(Y|X) = H(X,Y) - H(X)
	*/
	double getConditionalEntropy(int y, int x){
		double entX = getEntropy(x);
		double jointXY = getJointEntropy(x, y);

		return jointXY - entX;
	}

   /**
	* I(X,Y) = H(X) - H(X|Y)
	*/
	double getMutualInformation(int x, int y){
		return getEntropy(x) - getConditionalEntropy(x, y);
	}

   /**
	* Prints the frequency distribution
	*/
	void printFrequencyDist(){

		for(int i = 0; i < numSensors;i++){
			double sum = 0.0;
			printf("");
			for(int j = 0; j < numBins;j++){
				printf("%lf " ,(frequencyDist[i][j] / (double)timeSteps));
				sum += (frequencyDist[i][j] / (double)timeSteps);
			}
			printf("\nsum:%lf\n",sum);
		}
		printf("");
	}

   /**
	* Prints the two-dimensional frequency distribution
	*/
	void printFrequency2Dist(){
		for(int i = 0; i < numSensors;i++){
			for(int j = 0; j < numSensors;j++){
				double sum = 0.0;
				printf("i: %d j: %d\n" , i , j);
				for(int k = 0; k < numBins;k++){
					for(int l = 0; l < numBins;l++){
						printf(" %5.5lf ", frequency2Dist[i][j][k][l] / (double)timeSteps);
						sum += (frequency2Dist[i][j][k][l] / (double)timeSteps);
					}
					printf("\n");
				}
				printf("\nsum: %lf\n", sum);
				}
		}
	}

   /**
	* Prints the two-dimensional frequency distribution
	* Takes two sensors as input
	*/
	void printFrequency2Dist(int sensor1, int sensor2){
		for(int k = 0; k < numBins;k++){
			for(int l = 0; l < numBins;l++){
				if(l != 0)
					printf(", %lf" ,frequency2Dist[sensor1][sensor2][k][l] / (double)timeSteps);
				else
					printf("%lf", frequency2Dist[sensor1][sensor2][k][l] / (double)timeSteps);
			}
			printf("");
		}
	}

   /**
	* Prints distance matrix
	*/
	void printDM(){
	for(int i = 0; i < numSensors;i++){
		for(int j = 0; j < numSensors; j++){
			if(j != 0)
				printf("  %lf",matrix[i][j]);
			else
				printf("%lf",matrix[i][j]);
			}
			printf("\n");
		}
	}

};

