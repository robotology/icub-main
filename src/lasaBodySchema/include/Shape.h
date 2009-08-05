// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
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
#ifndef __SHAPE_H__
#define __SHAPE_H__


#include "TreeParser.h"
#include "mathlib.h"
#include <GL/glu.h>
#include <GL/glut.h>
#include "RigidTransfo.h"


class Sphere;
class Capsule;
class Parallelipiped;



class Shape
{
protected:
  CVector3_t position; // position with respect to joint //formerly pos_joint
  CVector3_t orientation;//micha
  int highlight;

public:
  Shape();
  virtual ~Shape(){};
  virtual void Render(){};
    virtual void Update(CVector3_t *child_positions, int nbpos){};  
  void SetOrientation(CVector3_t orient){v_copy(orient,orientation);};
  void SetPosition(CVector3_t pos){v_copy(pos,position);};
  void GetPosition(CVector3_t pos)const {v_copy(position,pos);};
  virtual void Stream(ostream& out)const{};
  void Highlight(){highlight=1;}
  void Unhighlight(){highlight=0;}

  /**
   * computes the distance between to shapes
   * @param sh second shape
   * @param transfo the transformation converting the FoF from sh to this
   */
  virtual float Distance(Shape& sh, RigidTransfo& transfo);
};




class Capsule:public Shape
{
protected:
	CVector3_t axis;
	float radius;
	GLUquadric* cyl;

public:
	Capsule();
	Capsule(pTree config);
	virtual ~Capsule();
	void SetRadius(float r){radius=r;}
	float GetRadius()const {return radius;};

	void SetAxis(CVector3_t new_axis){v_copy(new_axis,axis);}
	void GetAxis(CVector3_t ax){v_copy(axis,ax);}
	virtual void Render();
    virtual void Update(CVector3_t *child_positions, int nbpos);  
	virtual void Stream(ostream& out)const;
	virtual float Distance(Shape& sh,RigidTransfo& transfo);
	virtual float Distance(Capsule& cap,RigidTransfo& transfo);
	virtual float Distance(Sphere& sph,RigidTransfo& transfo);
	virtual float Distance(Parallelipiped& para, RigidTransfo& transfo);

	void GetParam(CVector3_t pos_cap,CVector3_t axis_cap,float& radius_cap);

};




class Sphere:public Shape
{
private:
	float radius;

public:
	Sphere();
	Sphere(pTree config);
	void SetRadius(float r){radius=r;}
	float GetRadius()const {return radius;};
	//Compute the minimum distance between two spheres
	//
	//CVector3_t centre1, CVector3_t centre2 are the two centers of spheres
	//float radius1, float radius2 are the two radius of spheres
	//float Dist_sphere(const CVector3_t center1, const CVector3_t center2, float radius1, float radius2);
	virtual void Render();
	virtual void Stream(ostream& out)const;

	virtual float Distance(Sphere& sphere,RigidTransfo& transfo);
	virtual float Distance(Shape& sh, RigidTransfo& transfo);
	virtual float Distance(Capsule& cap, RigidTransfo& transfo);
	virtual float Distance(Parallelipiped& para, RigidTransfo& transfo);



	void GetParam(CVector3_t center_sph, float& radius_sph);
};




class Parallelipiped: public Shape
{
private:
  CVector3_t size;
  CVector3_t com;

public:
	Parallelipiped();
	Parallelipiped(pTree config);
	void SetSize(float x, float y, float z){v_set(x,y,z,size);};
	void SetSize(CVector3_t s){v_copy(s,size);};
	virtual void Render();
	virtual void Stream(ostream& out)const;

	float Distance_point_face(const CVector3_t point,int plan,float longueur,float largeur,float hauteur);      
	void Point_proche_plan(const CVector3_t point1,const CVector3_t point2,int plan,float longueur,float largeur,float hauteur,CVector3_t out);
	int Inter_droite_plan (const CVector3_t point1,const CVector3_t point2,int plan,float dist,CVector3_t out);
	
	virtual float Distance(Capsule& cap, RigidTransfo& transfo);
	virtual float Distance(Sphere& sph,RigidTransfo& transfo);
	virtual float Distance(Shape& sh, RigidTransfo& transfo);

};



#endif
