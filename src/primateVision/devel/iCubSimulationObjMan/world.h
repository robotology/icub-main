// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file world.h
 * \brief Header for the world creation
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

#pragma once
#include "SDL.h"
#include "SDL_opengl.h"
#include "MS3D.h"
#include "rendering.h" 
#include <ode/ode.h>
#include "objManio.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif 
#define SQUARE_DIMs 512
#define MAX_OBJS 50 
#include <yarp/os/Semaphore.h>

using namespace yarp::os;
using namespace iCub::contrib::primateVision;

class worldSimData{ 
public:	
	#define MAXNUM 100
	#define GPB 3// maximum number of geometries per body
	double l_massobj0;
	double l_massobj1;
	double l_massobj2;
	double l_massobj3;
	
	int OBJNUM;
	int waitOBJ;
	int S_OBJNUM;
	
	int cylOBJNUM;
	int waitOBJ1;
	int S_cylOBJNUM;
    int S_lablOBJNUM;
	int waitOBJ2;

	dReal color[100][3];
	dReal s_color[100][3];
	dReal color1[100][3];
	dReal s_color1[100][3];
	worldSimData();
};

class worldSim : public worldSimData {
public:
	static const bool textured = true;
	ConstString actWorld;
			// max number of objects
#define numObjJoints 5 //define Joints
    bool reinitialized;
	dJointID joint	[numObjJoints];
	dReal	 speed [numObjJoints];
	dJointID j;

	dBodyID tableBody[5];
	dGeomID tableGeom[5];

	dBodyID ballBody;
	dGeomID ballGeom;

	//init. encapsulated object - the Box
	dGeomID box_part[14];
	dGeomID box_geom[14];
	dBodyID Box;

	//Geometry group for the bodies of the Object
	dSpaceID boxObj;

	dBodyID box;
	dGeomID boxgeom;
     
    double rotation_x, rotation_x_increment;
    double rotation_y, rotation_y_increment;
    double rotation_z, rotation_z_increment;
    dMatrix3 Rtx,Rty,Rtz,Rtmp1,Rtmp2;

	int objManText;
	yarp::os::Semaphore objMutex;
	
struct MyObject {
  dBodyID boxbody;			// the body
  dGeomID geom[GPB];		// geometries representing this body
  dReal size[3];
};

 MyObject obj[MAXNUM];
 MyObject s_obj[MAXNUM];

 struct MyObject1 {
  dBodyID cylbody;			// the body
  dGeomID cylgeom[GPB];		// geometries representing this body
  dReal radius;
  dReal lenght;
};

 MyObject1 cyl_obj[MAXNUM];
 MyObject1 s_cyl_obj[MAXNUM];


struct MyObject2 {
  dBodyID labelbody;			// the body
  dGeomID labelgeom[GPB];		// geometries representing this body
  dReal radius;
  int label;
  int texture; 
  ConstString objManlabel;	
  
};
 MyObject2 s_labl_obj[MAXNUM];
 
 bool active[100];

bool passed;
 

//objMan
Port inPort_s;
BinPortable<ObjManServerParams> server_response; 
Bottle empty;
ObjManServerParams rsp;
int ObjManwidth, ObjManheight, psb_in;
double objManlastData;
int inc;

unsigned char *imgDataRGBA;
//Port to get online object list:
BufferedPort<ObjManServerList> inPort_objList; 
ObjManServerList *objList;

ConstString labelstring;
public:

	void resetSpeeds();
	void setJointSpeeds();
	void syncAngles();
	void ballDamping();
	void draw();
	void drawGeom(dGeomID g, const dReal *pos, const dReal *rot);//, float red  = 0.0f, float green = 128.5f, float blue = 255.0f);
    void setPosition(dReal agent1X, dReal agent1Z, dReal agent1Y );
	void activateWorld();
	void init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z );
    float computeScale(const char* strs[4]);


	~worldSim();

	worldSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z);
};
