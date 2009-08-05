// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Basilio Noris
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
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


#pragma warning(disable: 4996)

#include <cstdio>
#include <cstring>
#include "MathLib/MathLib.h"
#include "quatMath.h"
#include "absorient.h"

void AO::AbsOrient(std::vector<vec3> L, std::vector<vec3> R, vec3 *rot, float *theta, float *scale, vec3 *translation)
{
	vec3 *pL = new vec3[L.size()];
	vec3 *pR = new vec3[R.size()];
	copy( L.begin(), L.end(), pL);
	copy( R.begin(), R.end(), pR);
	AbsOrient(pL, pR, L.size(), rot, theta, scale, translation);
	delete [] pL;
	delete [] pR;
}

void AO::AbsOrient(vec3 *L, vec3 *R, int pointCnt, vec3 *rot, float *theta, float *scale, vec3 *translation)
{
	/**********/
	/* Step 1 */
	/**********/
	// find the centroids and center the data;
	vec3 mL, mR;
	for (int i=0; i<pointCnt; i++)
	{
		mL += L[i];
		mR += R[i];
	}
	mL /= (float)pointCnt;
	mR /= (float)pointCnt;
	// subtract them from the data
	for (int i=0; i<pointCnt; i++)
	{
		L[i] -= mL;
		R[i] -= mR;
	}


	/**********/
	/* Step 2 */
	/**********/
	// compute covariances;
	float S[9];
	memset(S, 0, 9*sizeof(float));
	for (int k=0; k < pointCnt; k++)
	{
		for(int i=0; i < 3; i++)
		{
			for(int j=0; j < 3; j++)
			{
				S[3*i+j] += L[k]._[i]*R[k]._[j];
			}
		}
	}

	/**********/
	/* Step 3 */
	/**********/
	// compute N;
	#define SS(i,j) S[3*(i-1) + (j-1)]
	Matrix4 N = Matrix4(
		SS(1,1)+SS(2,2)+SS(3,3),  SS(2,3)-SS(3,2),  SS(3,1)-SS(1,3),  SS(1,2)-SS(2,1),
		SS(2,3)-SS(3,2),  SS(1,1)-SS(2,2)-SS(3,3),  SS(1,1)+SS(2,1),  SS(3,1)+SS(1,3),
		SS(3,1)-SS(1,3),  SS(1,2)+SS(2,1), -SS(1,1)+SS(2,2)-SS(3,3),  SS(2,3)+SS(3,2),
		SS(1,2)-SS(2,1),  SS(3,1)+SS(1,3),  SS(2,3)+SS(3,2), -SS(1,1)-SS(2,2)+SS(3,3)
	);
	#undef SS


	/**********/
	/* Step 4 */
	/**********/

	/*

	Matrix T(4,4), tmp1, tmp2;
	T(0,0) = 2; T(0,1) = 0; T(0,2) = 0; T(0,3) = 0;
	T(1,0) = 0; T(1,1) = 1; T(1,2) = 0; T(1,3) = 0;
	T(2,0) = 0; T(2,1) = 0; T(2,2) = -1; T(2,3) = 0;
	T(3,0) = 0; T(3,1) = 0; T(3,2) = 0; T(3,3) = 1;

	T.Print();
	Vector eigval(4);
	Matrix eigvec(4,4);
	T.Tridiagonalize(tmp1, eigvec);
	tmp1.TriEigen(eigval, eigvec);
	eigval.Print();
	eigvec.Print();
	*/




	// compute the rotation, scale and translation;
	Vector eigval;
	Matrix T, T2, eigvec;

	eigval.Resize(4);
	eigvec.Resize(4,4);

	Matrix(N).Tridiagonalize(T, eigvec);
	T.TriEigen(eigval, eigvec);
	int maxInd = 0;
	float maxVal(eigval(0));
	for(int i=0; i<(int)eigval.Size(); i++)
	{
		if(maxVal < eigval(i))
		{
			maxVal = eigval(i);
			maxInd = i;
		}
	}	
	Vector big = eigvec.GetColumn(maxInd);
	Quat qBig = Quat(big(0), big(1), big(2), big(3));
	
	// rotation
	float newtheta = acosf(qBig.w)*2;
	
	if (newtheta < PIf)
	{
		qBig = qBig.conj();
	}
	else
	{
		newtheta = -(newtheta - 2*PIf);
	}
	
	vec3 vec = qBig.vec();
	vec.normalize();

	// scale
	float sL = 0;
	float sR = 0;
	
	for (int i=0; i<pointCnt; i++)
	{
		sL += L[i].sqnorm();
		sR += R[i].sqnorm();
	}

	float rescale = sqrtf(sR / sL);

	// translation
	Quat qRot(vec, -newtheta);
	vec3 omL = qRot.rotate(mL);
	omL *= rescale;
	vec3 trans = mR - omL;

	/**********/
	/* Step 5 */
	/**********/
	// compute the rotation, scale and translation;
	//printf("rotation: (%.3f %.3f %.3f) -> %.1f°\n",vec.x,vec.y,vec.z, newtheta);
	//printf("resize factor: %.3f\n",rescale);
	//printf("translation: (%.3f %.3f %.3f)\n",trans.x,trans.y,trans.z);
	if(rot) (*rot) = vec;
	if(theta) (*theta) = newtheta;
	if(scale) (*scale) = rescale;
	if(translation) (*translation) = trans;

} 
