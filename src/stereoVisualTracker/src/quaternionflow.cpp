#include "public.h"
#include "basicMath.h"
#include "opticalflow.h"
#include "quaternionflow.h"

//TICTOC;

QuaternionFlow::QuaternionFlow(Vec2 fov, u32 features)
{
	flow = new OpticalFlow(features);
	this->fov = fov;
	head = vec3(0,0,1);
	up = vec3(0,1,0);
	qHead = Quat(head);
}

QuaternionFlow::~QuaternionFlow()
{
	if(flow) delete flow;
}

void QuaternionFlow::Apply(IplImage *image)
{
	if(!flow) return;
	flow->Apply(image);
	//	int hC = 0;

	Vec2 *orig = flow->GetOrigins();
	Vec2 *vecs = flow->GetVectors();
	int vectCnt = flow->GetCount();
	Vec2 res(image->width, image->height);
	std::vector<vec3> R, L;
	
	for (int i=0; i<vectCnt; i++)
	{
		R.push_back(Vec2Angles2(orig[i], fov, res)*2);
		L.push_back(Vec2Angles2(orig[i]+vecs[i], fov, res)*2);
	}

	float newtheta;
	float scale;
	vec3 newrot;
	vec3 translate;
	AO::AbsOrient(L, R, &newrot, &newtheta, &scale, &translate);
	qRot = Quat(0,0,1,0);
	if(newtheta == newtheta)
	{
		printf("theta: %.4f\t(%.3f %.3f %.3f)\n", newtheta, newrot.x, newrot.y, newrot.z);
		qRot = Quat(newrot, newtheta);
		head = qRot.rotate(head).normalized();
		up = qRot.rotate(up).normalized();
		qHead = qRot*qHead;
		if(head.x != head.x)
		{
			// NaN
			head = vec3(0,0,1);
			up = vec3(0,1,0);
			qHead = Quat(head);
		}
	}

}

void QuaternionFlow::Draw(IplImage *image)
{
	if(!flow) return;
	flow->Draw(image);
	int steps = 10;
	Vec2 res(image->width, image->height);
	for (int x=0; x <= steps; x++)
	{
		for (int y=0; y <= steps; y++)
		{
			Vec2 point (x*res.x/steps,y*res.y/steps);
			Vec2 point2 = RotateVec2(point, qRot, fov, res);
			cvLine(image, point.to2d(), point2.to2d(), CV_RGB(0,0,0), 3);
			cvLine(image, point.to2d(), point2.to2d(), CV_RGB(0,255,0), 2);
		}
	}
}


Vec2 QuaternionFlow::Angles2Vec(Vec2 angles, Vec2 fov, Vec2 res)
{
	float alpha, beta;
	alpha = asin(angles.x);
	beta = asin(angles.y);
	return Vec2((alpha/fov.x+0.5f)*res.x, (beta/fov.y+0.5f)*res.y);
}

vec3 QuaternionFlow::Vec2Angles(Vec2 v, Vec2 fov, Vec2 res)
{
	Vec2 a;
	a.x = (v.x / (float)res.x - 0.5f) * fov.x;
	a.y = -(v.y / (float)res.y - 0.5f) * fov.y;
	vec3 point = vec3(0,0,1);
	Quat qRot(vec3(1,0,0), a.y);
	point = qRot.rotate(point);
	qRot = Quat(vec3(0,1,0), a.x);
	point = qRot.rotate(point).normalized();
	return point;
}


vec3 QuaternionFlow::Vec2Angles2(Vec2 v, Vec2 fov, Vec2 res){
  Vec2 al,a;
  a.x = (v.x / (float)res.x - 0.5f);
  a.y = -(v.y / (float)res.y - 0.5f);
  float fx = 0.5/tan(0.5*fov.x);
  float fy = 0.5/tan(0.5*fov.y);
  al.x = atan(a.x/fx);
  al.y = atan(a.y/fy);
  vec3 point = vec3(0,0,1);
  Quat qRot(vec3(1,0,0), al.y);
  point = qRot.rotate(point);
  qRot = Quat(vec3(0,1,0), al.x);
  point = qRot.rotate(point).normalized();
  return point;
}


Vec2 QuaternionFlow::RotateVec2(Vec2 point, Quat qRot, Vec2 fov, Vec2 res)
{
	vec3 a = Vec2Angles2(point, fov, res);
	a = qRot.rotate(a).normalized();
	return Angles2Vec(Vec2(a.x,a.y), fov, res);
}

