// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Basilio Noris EPFL
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

#ifndef _QUATERNION_FLOW_H_
#define _QUATERNION_FLOW_H_

#include "absorient.h"



#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*(PIf/180.0f))
#endif

class QuaternionFlow : public Filter
{
protected:
	OpticalFlow *flow;
	Vec2 fov;
	Quat qRot;
	vec3 head;
	vec3 up;
	Quat qHead;

public:
	QuaternionFlow(Vec2 fov = Vec2(DEG2RAD(30), DEG2RAD(30)), u32 features=400);
	~QuaternionFlow();
	void Draw(IplImage *image);
	void Apply(IplImage *image);
	IplImage *Process(IplImage *image){return NULL;};
	OpticalFlow *GetFlow(){return flow;};
	Quat GetRotation(){return qRot;};

	static Vec2 Angles2Vec(Vec2 angles, Vec2 fov, Vec2 res);
	static vec3 Vec2Angles(Vec2 v, Vec2 fov, Vec2 res);
	static vec3 Vec2Angles2(Vec2 v, Vec2 fov, Vec2 res);
	static Vec2 RotateVec2(Vec2 point, Quat qRot, Vec2 fov, Vec2 res);
};



#endif // _QUATERNION_FLOW_H_
