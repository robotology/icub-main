/* 
 *  FACEOBJECT.cpp
 *
 *  Created by Bret Fortenberry on Aug 27, 2003.
 *  Fixes: 
 *			In posterior(combine_mode mode) method:
 *			case average: of switch, was added a condition in outlier searching
 *			To avoid outlier searching, Eyes.size() must be greater than 0
 *			by Plinio Moreno  on May 9,2007
 *
 *		To compile properly,  the copy constructor FaceObject(const EyeObject &thelist);
 *		is commented, and the clear method does not clear the visualObject list.
 *		by Plinio Moreno on Oct 06, 2007
 *  Copyright (c) 2003 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *  
 */

#include <iCub/vis/faceobject.h>
#include <iostream>

FaceObject::FaceObject(){
	feature = e_face;
	leftEyes.reserve(EYEMEMSIZE);
	rightEyes.reserve(EYEMEMSIZE);
}
		
FaceObject::FaceObject(float x_in, float y_in, float xSize_in, float ySize_in, float scale_in){
	x = x_in;
	y = y_in;
	xSize = xSize_in;
	ySize = ySize_in;
	scale = scale_in;
	feature = e_face;
	leftEyes.reserve(EYEMEMSIZE);
	rightEyes.reserve(EYEMEMSIZE);
}

/*FaceObject::FaceObject(const FaceObject &thelist)
{
	objects = thelist.objects;
	x = thelist.x;
	y = thelist.y;
	xSize = thelist.xSize;
	ySize = thelist.ySize;
	scale = thelist.scale;
  eyes = thelist.eyes;
	leftEyes = thelist.leftEyes;
	rightEyes = thelist.rightEyes;
	feature = thelist.feature;
	activation = thelist.activation;
}*/

FaceObject::FaceObject(TSquare<float> &square)
{
	x = square.x;
	y = square.y;
	xSize = square.size;
	ySize = square.size;
	scale = square.scale;
	leftEyes.reserve(EYEMEMSIZE);
	rightEyes.reserve(EYEMEMSIZE);
	feature = e_face;
}

FaceObject::FaceObject(list<Square>::iterator face)
{
	x = (float)face->x;
	y = (float)face->y;
	xSize = (float)face->size;
	ySize = (float)face->size;
	scale = (float)face->scale;
	feature = e_face;
}

FaceObject::~FaceObject() {
	clear();
}

void FaceObject::clear(){
	/*if(objects.size())
	{
		list< VisualObject* >::iterator it = objects.begin();
		for(; it != objects.end(); ++it)
		{
			(*it)->clear();
		}
		objects.clear();
	}*/
	leftEyes.clear();
	rightEyes.clear();
}



void FaceObject::findMax()
{
  double max = -1000000;
  if(leftEyes.size()){
    //cout << "finding max left eye" << endl;
    eyes.leftEye = true;
    vector< EyeObject >::iterator it = leftEyes.begin();
    for(; it != leftEyes.end(); ++it){
      if(max < it->activation){
				max = it->activation;
				//cout << "max: " <<  max;
				eyes.xLeft = it->x;
				eyes.yLeft = it->y;
				eyes.leftScale = it->scale;
				//cout << " at (" << it->x << "," << it->y << ")" << endl; 
      }
    }
  }
  max = -1000000;
  if(rightEyes.size()){
    eyes.rightEye = true;
    vector< EyeObject >::iterator it = rightEyes.begin();
    for(; it != rightEyes.end(); ++it){
      if(max < it->activation){
				max = it->activation;
				eyes.xRight = it->x;
				eyes.yRight = it->y;
				eyes.rightScale = it->scale;
      }
    }
  }
}

void FaceObject::posterior(combine_mode mode)
{
	switch(mode){
		double maxAct;
		case mean_shift:
		case mpi_search:
		case maximum:
		// not currently used
		break;
		case face_only:
			eyes.xLeft = x + (xSize*0.7077f);
			eyes.yLeft = y + (xSize*0.3099f);
			eyes.leftScale = scale;
			eyes.xRight = x + (xSize*0.3149f);
			eyes.yRight = y + (xSize*0.3136f);
			eyes.rightScale = scale;
			break;

		case wt_max:
		case none:

			maxAct = -1000000;
			if(leftEyes.size()){
				eyes.leftEye = true;
				vector< EyeObject >::iterator it = leftEyes.begin();
				for(; it != leftEyes.end(); ++it){
					if(maxAct < it->activation){
						maxAct = it->activation;
						eyes.xLeft = it->x;
						eyes.yLeft = it->y;
						eyes.leftScale = it->scale;
					}
				}
			}

			maxAct = -1000000;
			if(rightEyes.size()){
				eyes.rightEye = true;
				vector< EyeObject >::iterator it = rightEyes.begin();
				for(; it != rightEyes.end(); ++it){
					if(maxAct < it->activation){
						maxAct = it->activation;
						eyes.xRight = it->x;
						eyes.yRight = it->y;
						eyes.rightScale = it->scale;
					}
				}
			}
			break;
		case wt_avg:
		case average:
			float xWt;
			float yWt;
			float scaleWt;
			float totalAct;
			vector< vector< float > > meanSub;
			vector< vector< float > > wtMeanSub;
			vector< vector< float > > invMtx;
			vector< vector< float > > covMtx;
			vector< EyeObject > *EyesPtr = 0;

			//cout << "leftEyes size = " << leftEyes.size() << endl;
			for(int cur_eye = 0; cur_eye < 2; cur_eye++){
				bool run = false;
				if(leftEyes.size() && cur_eye == 0){  
					EyesPtr = &leftEyes;
					run = true;
				}
				else if(rightEyes.size() && cur_eye == 1){
					EyesPtr = &rightEyes;
					run = true;
				}
				else
					cur_eye++;
				if (run){
					vector< EyeObject > Eyes = *EyesPtr;
					float expo;
					bool outlier = true;
					float mean3d[3];
					float determ;
					while(outlier && (Eyes.size() != 0)){//To avoid outlier searching, Eyes.size() must be greater than 0
						totalAct = 0.0f;
						xWt = 0.0f;
						yWt = 0.0f;
						scaleWt = 0.0f;
						//if(Eyes.size() == 0)
						//	_DEBUG_ERROR("No Eyes !?!?!?!");
						for (unsigned int i = 0; i < Eyes.size(); i++){
							expo = (float)exp(Eyes[i].activation);
							xWt += (expo * Eyes[i].x);
							yWt += (expo * Eyes[i].y);
							scaleWt += (expo * Eyes[i].scale);
							totalAct += expo;
						}
						mean3d[0] = xWt/totalAct; mean3d[1] = yWt/totalAct; mean3d[2] = scaleWt/totalAct;

						for(unsigned int pos = 0; pos < Eyes.size(); ++pos){
							vector< float > v;
							vector< float > wt;
							expo = (float)exp(Eyes[pos].activation);
							v.push_back(Eyes[pos].x - mean3d[0]);
							wt.push_back(v[0] * expo);
							v.push_back(Eyes[pos].y - mean3d[1]);
							wt.push_back(v[1] * expo);
							v.push_back(Eyes[pos].scale - mean3d[2]);
							wt.push_back(v[2] * expo);
							meanSub.push_back(v);
							wtMeanSub.push_back(wt);
						}
						float divTotAct = 1.0f/totalAct;
						if(mtxMult_2T(meanSub, wtMeanSub, covMtx, divTotAct) != 9){
							meanSub.clear();
							wtMeanSub.clear();
							invMtx.clear();
							covMtx.clear();
							break;
						}
						if((determ = det3(covMtx))){
							TransCof3(covMtx, invMtx, determ);
						}
						else { //later implement psudo inverse page 192
							meanSub.clear();
							wtMeanSub.clear();
							invMtx.clear();
							covMtx.clear();
							break; //exit while loop
						}
						outlier = findOutliers(meanSub, invMtx, Eyes);
						meanSub.clear();
						wtMeanSub.clear();
						invMtx.clear();
						covMtx.clear();
					}//while loop
					if(mean3d[0] > x && mean3d[0] < (x+xSize) && mean3d[1] > y && mean3d[1] < (y+ySize)){
						if(cur_eye == 0){
							eyes.xLeft = mean3d[0];
							eyes.yLeft = mean3d[1];
							eyes.leftScale = mean3d[2];
							eyes.leftEye = true;
						}
						if(cur_eye == 1){
							eyes.xRight = mean3d[0];
							eyes.yRight = mean3d[1];
							eyes.rightScale = mean3d[2];
							eyes.rightEye = true;
						}
					}
				}
			}

		break;
	};
}


void FaceObject::TransCof3(vector< vector< float > > &inMtx, vector< vector< float > > &rtnMtx, float det){
	float divDet = 1/det;
	vector< float > v;
	v.push_back((inMtx[1][1]*inMtx[2][2]-inMtx[1][2]*inMtx[2][1])*divDet);
	v.push_back(-(inMtx[0][1]*inMtx[2][2]-inMtx[0][2]*inMtx[2][1])*divDet);
	v.push_back((inMtx[0][1]*inMtx[1][2]-inMtx[0][2]*inMtx[1][1])*divDet);
	rtnMtx.push_back(v);
	vector< float > v2;
	v2.push_back(-(inMtx[1][0]*inMtx[2][2]-inMtx[1][2]*inMtx[2][0])*divDet);
	v2.push_back((inMtx[0][0]*inMtx[2][2]-inMtx[0][2]*inMtx[2][0])*divDet);
	v2.push_back(-(inMtx[0][0]*inMtx[1][2]-inMtx[0][2]*inMtx[1][0])*divDet);
	rtnMtx.push_back(v2);
	vector< float > v3;
	v3.push_back((inMtx[1][0]*inMtx[2][1]-inMtx[1][1]*inMtx[2][0])*divDet);
	v3.push_back(-(inMtx[0][0]*inMtx[2][1]-inMtx[0][1]*inMtx[2][0])*divDet);
	v3.push_back((inMtx[0][0]*inMtx[1][1]-inMtx[0][1]*inMtx[1][0])*divDet);
	rtnMtx.push_back(v3);
} //http://astronomy.swin.edu.au/~pbourke/analysis/inverse/

float FaceObject::det3(vector< vector< float > > &matrix){ 
	if ((matrix.size() != 3) || (matrix[0].size() != 3))
		return (0.0f);
	return ((matrix[0][0] * (matrix[1][1]*matrix[2][2] - matrix[1][2]*matrix[2][1])) - 
		(matrix[0][1] * (matrix[1][0]*matrix[2][2] - matrix[1][2]*matrix[2][0])) +
		(matrix[0][2] * (matrix[1][0]*matrix[2][1] - matrix[1][1]*matrix[2][0])));
} //http://www.mathworks.com/access/helpdesk/help/toolbox/aeroblks/determinantof3x3matrix.shtml



int FaceObject::mtxMult(vector< vector< float > > &matrix1, vector< vector< float > > &matrix2, vector< vector< float > > &rtnMtx){
	if(matrix1[0].size() != matrix2.size()){
		return 0;
		}
	rtnMtx.clear();
	float comb;
	int size = 0;
	for(unsigned int cur_row = 0; cur_row < matrix1.size(); ++cur_row){
		vector< float > v;
		for(unsigned int cur_col = 0; cur_col < matrix2[0].size(); ++cur_col){
			comb = 0;
			for(unsigned int cur_pos = 0; cur_pos < matrix1[0].size(); ++cur_pos){
				comb += matrix1[cur_row][cur_pos] * matrix2[cur_pos][cur_col];
			}
			v.push_back(comb);
			size++;
		}
		rtnMtx.push_back(v);
	}
	return size;
}


int FaceObject::mtxMult_2T(vector< vector< float > > &matrix1, vector< vector< float > > &matrix2, vector< vector< float > > &rtnMtx, float mult){
	if(matrix1.size() != matrix2.size())
		return 0;
	rtnMtx.clear();
	float comb;
	int size = 0;
	for(unsigned int cur_row = 0; cur_row < matrix1[0].size(); ++cur_row){
		vector< float > v;
		for(unsigned int cur_col = 0; cur_col < matrix2[0].size(); ++cur_col){
			comb = 0;
			for(unsigned int cur_pos = 0; cur_pos < matrix1.size(); ++cur_pos){
				comb += matrix1[cur_pos][cur_row] * matrix2[cur_pos][cur_col];
			}
			v.push_back(comb*mult);
			size++;
		}
		rtnMtx.push_back(v);
	}
	return size;
}


bool FaceObject::findOutliers(vector< vector< float > > &meanSub, vector< vector< float > > &invMtx, vector< EyeObject > &Eyes){
	vector< vector< float > > mnSubMtx;
	vector< vector< float > > tempMtx;
	vector< vector< float > > di;
	bool rtn = false;
	for (int i = (meanSub.size() - 1); i >= 0; --i){
		mnSubMtx.push_back(meanSub[i]);
		mtxMult(mnSubMtx, invMtx, tempMtx);
		mtxMult_2T(tempMtx, mnSubMtx, di);
		//cout << "outlier di = " << di[0][0] << endl;
		if (di[0][0] > 1.0f){
			//cout << "outlier di !!!!!!!= " << di[0][0] << endl;
			rtn = true;
			Eyes.erase(Eyes.begin() + i);
		}
		mnSubMtx.clear();
		tempMtx.clear();
		di.clear();
	}
	return rtn;
}


#ifndef WIN32
int FaceObject::prtMtx(vector< vector< float > > &matrix){
	for(unsigned int cur_row = 0; cur_row < matrix.size(); ++cur_row){
		cout << "(";
		for(unsigned int cur_col = 0; cur_col < matrix[0].size(); ++cur_col){
				cout << matrix[cur_row][cur_col] << ", ";
		}
		cout << ")\n";
	}
	cout << endl;
	return 1;
}
#endif


/*
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 
 *    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *    3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
