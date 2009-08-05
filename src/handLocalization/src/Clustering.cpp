// include header
#include <Mathematics.h>

// namespaces
using namespace thesis::mathematics;

using std::vector;

// ***************************************************************************

/**
 * Implementation of Cluster class
 */

// Init constructor
Cluster::Cluster () {
	//printf("Start:\t[Clustering]\n");
}

Cluster::~Cluster () {
	//printf("Quit:\t[Clustering]\n");
}


/* K-means Clustering Algorithm
 * Copyright (C) 2005 Danzhou Liu 
 
 * This code is free, and hopefully it will be useful for researchers 
 * but WITHOUT ANY WARRANTY. You can redistribute it and/or modify it 
 * under the terms of the GNU General Public License.  
 *
 * Contact information:
 *  Danzhou Liu, School of Computer Science, University of Central Florida, FL 32816 
 *  Homepage: www.cs.ucf.edu/~dzliu
 *  Email: dzliu@cs.ucf.edu
 */

void Cluster::kMeans( double **_data_ptr, string _method, int _clusterNum, int _dataNum, int _dimNum, double _haltRatio, int _maxIters, int *_label_ptr, double **_clusterCenter_ptr) {

	Statistics stat;
	int		i, j, k, interval, iters, tempindex;
	int		*clusterHist_ptr;
	double	diff, tempdist;
	double	**oldClusterCenter_ptr, **dist_ptr, *average_ptr, *std_ptr;
	
	// allocate memory
	clusterHist_ptr = new int [_clusterNum];

	oldClusterCenter_ptr = new double* [_clusterNum];
	for( i=0; i < _clusterNum; i++ ){
		oldClusterCenter_ptr[i] = new double [_dimNum];
	}

	dist_ptr = new double* [_dataNum];
	for( i=0; i < _dataNum; i++ ){
		dist_ptr[i] = new double [_clusterNum];
	}

	// precompute the average & standard deviation for the "fix" data
	average_ptr = new double [_dataNum];
	std_ptr		= new double [_dataNum];
	for ( i=0; i < _dataNum; i++) {
		average_ptr[i]	= stat.average(_data_ptr[i], _dimNum);
		std_ptr[i]		= stat.standardDeviationND(_data_ptr[i], _dimNum);
	}

	// choose the initial cluster centers, and set the old cluster centers to 0.0
	//printf("* initial cluster centers & set the old cluster centers to 0.0: *\n");
	interval = int(_dataNum/_clusterNum);
	//printf("interval: %d\n", interval);
	for( i=0; i < _clusterNum; i++ ){
		for( j=0; j < _dimNum; j++ ){
			//printf("[%d][%d] = %f ->", i,j,_clusterCenter_ptr[i][j]);
			_clusterCenter_ptr[i][j]	= _data_ptr[i*interval][j];
			oldClusterCenter_ptr[i][j]	= 0.0;
			//printf(" %f\n", _clusterCenter_ptr[i][j]);
		}
	}
	
	// calculate the diffence of centers
	diff = 0.0;
	for( i=0; i < _clusterNum; i++ ){
		for( j=0; j < _dimNum; j++ ){
			diff = diff + fabs( _clusterCenter_ptr[i][j] - oldClusterCenter_ptr[i][j]);
		}
	}

	iters=0;
	//printf("diff: %f\n", diff);
	while ( (diff > _haltRatio) && (iters < _maxIters) ){
		//printf("iters: %d\n", iters);

		iters++;
		
		// old centers are set to be current centers
		//printf("* old centers are set to be current centers *\n");
		for ( i=0; i < _clusterNum; i++ ){
			for ( j=0; j < _dimNum; j++ ){
				//printf("[%d][%d] = %f ->", i, j, oldClusterCenter_ptr[i][j]);
				oldClusterCenter_ptr[i][j] = _clusterCenter_ptr[i][j];
				//printf(" %f\n", oldClusterCenter_ptr[i][j]);
			}
		}
		
		// initialize the dist to be zeros
		for ( i=0; i < _dataNum; i++ ){
			for ( j=0; j < _clusterNum; j++ ){
				dist_ptr[i][j] = 0;
			}
		}

		// calculate the distance to each cluster center
		//printf("* distance computation: *\n");
		double y_, s_y, corr, cov_xy;
		for ( i=0; i < _dataNum; i++ ){
			for ( j=0; j < _clusterNum; j++ ){
				if (_method == "euclidian") {
					for ( k=0; k < _dimNum; k++ ){
						dist_ptr[i][j] = dist_ptr[i][j] + ( _data_ptr[i][k] - _clusterCenter_ptr[j][k])*(_data_ptr[i][k] - _clusterCenter_ptr[j][k]);
					}
				}
				else if (_method == "distance correlation") {
					y_ = stat.average(_clusterCenter_ptr[j], _dimNum);
					s_y = stat.standardDeviationND(_clusterCenter_ptr[j], _dimNum);
					cov_xy = stat.covariance(_data_ptr[i], _clusterCenter_ptr[j], average_ptr[i], y_, _dimNum);
					corr = cov_xy / (std_ptr[i]*s_y);

					//double corr = stat.correlationCoefficient(_data_ptr[i], _clusterCenter_ptr[j], _dimNum);
					//printf("%f <> %f \n", corr, stat.correlationCoefficient(_data_ptr[i], _clusterCenter_ptr[j], _dimNum));
					dist_ptr[i][j] = dist_ptr[i][j] + (1 - corr);
					//printf("%f ) ", dist_ptr[i][j]);
				}
			}
			//printf("\n");
		}

		// set the cluster label for each point
		//printf("* label the point with the corresp. cluster number: *\n");
		tempdist	= 0.0;
		tempindex	= 0;
		for ( i=0; i < _dataNum; i++ ){
			tempdist	= dist_ptr[i][0];
			tempindex	= 0;
			for ( j=1; j < _clusterNum; j++ ){
				if (dist_ptr[i][j] < tempdist){
					tempdist	= dist_ptr[i][j];
					tempindex	= j;
				}
			}
			_label_ptr[i] = tempindex;
		}
		
		// initialize the cluster histogram 
		for ( i=0; i < _clusterNum; i++ ){
			clusterHist_ptr[i] = 0;
		}
		// calculate the cluster histogram
		for ( i=0; i < _dataNum; i++ ){
			clusterHist_ptr[_label_ptr[i]]++;
		}
		
		// initialize the center of clusters
		for ( i=0; i < _clusterNum; i++ ){
			for ( j=0; j < _dimNum; j++ ){
				_clusterCenter_ptr[i][j] = 0.0;
			}
		}		
	
		// calculate the new cluster centers
		//printf("* calculate the new cluster centers: *\n");
		for ( i=0; i < _dataNum; i++ ){
			for ( j=0; j < _dimNum; j++ ){
				_clusterCenter_ptr[_label_ptr[i]][j] += _data_ptr[i][j];
			}
		}		
        // calculate the mean of all points in a cluster
		for ( i=0; i < _clusterNum; i++ ){
			for ( j=0; j < _dimNum; j++ ){
				//if (clusterHist_ptr[i] != 0) {
				//	printf("(%f ", _clusterCenter_ptr[i][j]);
					_clusterCenter_ptr[i][j] /= clusterHist_ptr[i];
				//	printf("%d %f) ", clusterHist_ptr[i], _clusterCenter_ptr[i][j]);
				//}
				//else {
				//	_clusterCenter_ptr[i][j] = 0;
				//}
			}
			//printf("\n");
		}

		diff = 0;
		for ( i=0; i < _clusterNum; i++ ){
			for ( j=0; j < _dimNum; j++ ){
				diff += fabs( _clusterCenter_ptr[i][j] - oldClusterCenter_ptr[i][j]);
			}
		}
		
	}//while

	// free allocated memory
	delete clusterHist_ptr;
	
	for( i=0; i < _clusterNum; i++ ){
		delete [] oldClusterCenter_ptr[i];
	}
	delete [] oldClusterCenter_ptr;

	for ( i=0; i < _dataNum; i++ ){
		delete [] dist_ptr[i];
	}
	delete [] std_ptr;
	delete [] average_ptr;
	delete [] dist_ptr;
}


