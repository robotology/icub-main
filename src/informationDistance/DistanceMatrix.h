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
 *  Computes distance between sensors
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
using namespace std ;
/**
 * This class computes distance between sensors
 */
class DistanceMatrix {

protected:
	 double **matrix;
	 double **normalized_matrix;
	 int numSensors;
	 int timeSteps;

public:
	#define NATLOG_TO_LOG2 1.4426950408889634
	double *xpoints;
	double *ypoints;

	DistanceMatrix(){}
	DistanceMatrix(int ns){
		int i,j;
		numSensors = ns;
		matrix = (double**)malloc(numSensors*sizeof(double*));
		for(i = 0; i<numSensors; i++)
			matrix[i] = (double*)malloc(numSensors*sizeof(double));
		normalized_matrix = (double**)malloc(numSensors*sizeof(double*));
		for(i = 0; i<numSensors; i++)
			normalized_matrix[i] = (double*)malloc(numSensors*sizeof(double));
		xpoints = (double*)malloc(numSensors*sizeof(double));
		ypoints = (double*)malloc(numSensors*sizeof(double));


		// initialize with random values
		for(i = 0; i < numSensors; i++){
			xpoints[i] = rand();
			ypoints[i] = rand();
		}

		for(i = 0; i<numSensors; i++)
			for(j = 0; j<numSensors; j++){
			normalized_matrix[i][j] = 0;
			matrix[i][j] = 0;
		}
			timeSteps = 0;
	}


	~DistanceMatrix(){
		int i;
		for(i = 0; i<numSensors; i++)
			free(normalized_matrix[i]);
		free(normalized_matrix);

		for(i = 0; i<numSensors; i++)
			free(matrix[i]);
		free(matrix);

		free(xpoints);
		free(ypoints);

		}

	class MyPoint {
	public:
		int id;
		double x, y;

		MyPoint()
		{
			id = 0;
			x = 0;
			y = 0;
		};

		MyPoint(double x1, double y1, int id1){
			id = id1;
			x = x1;
			y = y1;
		}
	};


	typedef vector <MyPoint> POINTVECTOR;


	/**
	 * Calculates logarithm base 2
	 */
	static double log2(double val){
		return (val != 0.0) ? (log(val) * NATLOG_TO_LOG2) : 1.0;
	}

	/**
	 * Override for other method of computing distance
	 */
	void updateMatrix(double *values){
		for(int i = 0; i < numSensors;i++)
			for(int j = i; j < numSensors; j++){
				matrix[i][j] = matrix[i][j] + fabs(values[i] - values[j]);
				matrix[j][i] = matrix[i][j];

			}

		timeSteps++;
	}

	/**
	 * Returns matrix of distances
	 */
	double** getMatrix(){
		return matrix;
	}
	/**
	 *  Returns normalized matrix of distances
	 */
	double** getNormalizedMatrix(){
		return normalized_matrix;
	}

	/**
	 * Implements the relaxation algorithm
	 */
	void relax(){
		int i, j;
		double *forceX = (double*)malloc(numSensors*sizeof(double));
		double *forceY = (double*)malloc(numSensors*sizeof(double));
		double force = 0.0;
		double dist = 0.0;

		normaliseMatrix();

		for(i = 0; i < numSensors; i++){
			forceX[i] = 0.0;
			forceY[i] = 0.0;
			for(j = 0; j < numSensors;j++){
				if(j != i){
					/* euclidian distance */
					dist = sqrt(((xpoints[i] - xpoints[j]) *  (xpoints[i] - xpoints[j])) +
						((ypoints[i] - ypoints[j]) * (ypoints[i] - ypoints[j])));

					if(dist != 0.0)
						force =(dist - normalized_matrix[i][j]) / dist;
					else {
						force = normalized_matrix[i][j];
				}

					forceX[i] = forceX[i] + (force * (xpoints[j] -
									  xpoints[i]));
					forceY[i] = forceY[i] + (force * (ypoints[j] -
									  ypoints[i]));
				}
			}
		}

		for(i = 0; i < numSensors; i++){
			xpoints[i] = xpoints[i] + 0.002 * forceX[i];
			ypoints[i] = ypoints[i] + 0.002 * forceY[i];
		}
	free(forceX);
	free(forceY);
	}

	/**
	 * Implemets the relaxation algorithm for input times
	 */
	void relax(int steps){
		for(int i = 0; i < steps; i++)
			relax();
	}

	/**
	 * Returns points as one array
	 */
	double** getPoints(){
		double **temp= (double**)malloc(numSensors*sizeof(double*));
		int i,j;
		for(i = 0; i<numSensors; i++)
			temp[i] = (double*)malloc(2*sizeof(double));

		for(i = 0; i<numSensors; i++)
			for(j = 0; j<numSensors; j++){
				temp[i][j] = 0;
		}

		for(i = 0; i<numSensors; i++){
			temp[i][0] = xpoints[i];
			temp[i][1] = ypoints[i];
		}

		return temp;
	}

	/**
	 * Produces map of points
	 */
	int** getMap(){
		int nPoints = (int)sqrt(numSensors);
		int i,j,k, id;
		int **map= (int**)malloc(nPoints*sizeof(int*));
		for(i = 0; i<nPoints; i++)
			map[i] = (int*)malloc(nPoints*sizeof(int));
		POINTVECTOR points;
		POINTVECTOR ordered;

		for(i = 0; i < numSensors; i++){
			points.push_back(MyPoint(xpoints[i], ypoints[i], i + 1));
		}

			MyPoint lp;
		for(j = 0; j < numSensors; j++){
			double lowest = 10000;
			for(k  = 0; k < points.size(); k++){
				if(points.at(k).y < lowest){
					lp = points.at(k);
					lowest = ((MyPoint)points.at(k)).y;
					id = k;
				}
			}
			points.erase(points.begin()+id);
			ordered.push_back(lp);
		}
			int n = 0;
			for(j = 0; j < nPoints; j++){
				POINTVECTOR taken;
				n = j * nPoints;
				for(i = 0; i < nPoints; i++){
					taken.push_back(MyPoint(ordered.at(n + i).x,ordered.at(n + i).y,ordered.at(n + i).id));
				}

				for(k  = 0; k < nPoints; k++){
					double lowest = 10000;
					for(int l  = 0; l < taken.size(); l++){
						if(((MyPoint)taken.at(l)).x < lowest){
							lp = taken.at(l);
							lowest = taken.at(l).x;
							id = l;
						}
					}
				map[j][k] = lp.id;
				taken.erase(taken.begin()+id);

				}
			}
		ordered.clear();
		return map;
	}

	/**
	 * Produces map of points using the input points
	 */
	int** getMap(double* xypoints){
		int nPoints = (int)sqrt(numSensors);
		int i,j,k, id;
		int **map= (int**)malloc(nPoints*sizeof(int*));
		for(i = 0; i<nPoints; i++)
			map[i] = (int*)malloc(nPoints*sizeof(int));
		POINTVECTOR points;
		POINTVECTOR ordered;

		for(i = 0; i < numSensors; i++){
			points.push_back(MyPoint(xypoints[2*i+0], xypoints[2*i+1], i + 1));
		}

			MyPoint lp;
		for(j = 0; j < numSensors; j++){
			double lowest = 10000;
			for(k  = 0; k < points.size(); k++){
				if(points.at(k).y < lowest){
					lp = points.at(k);
					lowest = ((MyPoint)points.at(k)).y;
					id = k;
				}
			}
			points.erase(points.begin()+id);
			ordered.push_back(lp);
		}
			int n = 0;
			for(j = 0; j < nPoints; j++){
				POINTVECTOR taken;
				n = j * nPoints;
				for(i = 0; i < nPoints; i++){
					taken.push_back(MyPoint(ordered.at(n + i).x,ordered.at(n + i).y,ordered.at(n + i).id));
				}

				for(k  = 0; k < nPoints; k++){
					double lowest = 10000;
					for(int l  = 0; l < taken.size(); l++){
						if(((MyPoint)taken.at(l)).x < lowest){
							lp = taken.at(l);
							lowest = taken.at(l).x;
							id = l;
						}
					}
				map[j][k] = lp.id;
				taken.erase(taken.begin()+id);

				}
			}
		ordered.clear();
		return map;
	}

	/**
	 * Calculates the normalised matrix
	 */
	void normaliseMatrix(){
		for(int i = 0; i < numSensors;i++){
			for(int j = i; j < numSensors; j++){
				normalized_matrix[i][j] = (1 / ((double)(timeSteps + 1))) * matrix[i][j];
				normalized_matrix[j][i] = normalized_matrix[i][j];
			}
		}
	}

	/**
	 * Prints the map of points
	 */
	void printMap(double* xypoints){
		int nPoints = (int)sqrt(numSensors);
		int i,j;
		int **map;
		map = getMap(xypoints);
		printf("printing map...\n");
		for(i = 0; i<nPoints; i++){
			for(j = 0; j<nPoints; j++)
				printf("%d", map[i][j]);
			printf("\n");
		}
	}

};
