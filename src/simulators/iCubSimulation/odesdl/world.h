// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file world.h
 * \brief Header for the world creation
 * \author Vadim Tikhanoff
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

#ifndef ICUBSIMULATION_WORLD_INC
#define ICUBSIMULATION_WORLD_INC

#include "SDL.h"
#include "SDL_opengl.h"
#include "rendering.h" 
#include <ode/ode.h>
#include <string>
#include "RobotConfig.h"
#include "WorldOp.h"

#define DENSITY (1.0)		// density of all objects

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif 

class WorldObject {
public:
    virtual dBodyID getBody() const = 0;
    virtual dGeomID getGeometry() const = 0;
    virtual bool create(const WorldOp& op, WorldResult& result, int idx) = 0;
    virtual bool takeColor() { return true; }
};

typedef dReal real3[3];

class WorldObjectList {
public:
    int len;
    int *counter;
    real3 *colors;

    WorldObjectList(int len, int& counter, real3 *colors) :
        len(len),
        counter(&counter),
        colors(colors) {
    }

    virtual WorldObject& get(int index) = 0;
    virtual const WorldObject& get(int index) const = 0;

    virtual bool create(const WorldOp& op, WorldResult& result) {
        if (counter==NULL) return false;

        if (!op.location.isValid()) {
            result.setFail("location not set");
            return false;
        }

        int at = *counter;
        if (at>=len-1) {
            result.setFail("too many objects");
            return false;
        }
        WorldObject& obj = get(at);
        if (!obj.create(op,result,at)) return false;
        (*counter)++;

        if (op.dynamic.get()) {
            dBodySetPosition(obj.getBody(),
                             op.location.get(0),
                             op.location.get(1),
                             op.location.get(2));
        } else {
            dGeomSetPosition(obj.getGeometry(),
                             op.location.get(0),
                             op.location.get(1),
                             op.location.get(2));
        }

        if (op.color.isValid()) {
            if (colors!=NULL) {
                colors[at][0] = op.color.get(0);
                colors[at][1] = op.color.get(1);
                colors[at][2] = op.color.get(2);
            }
        }
        return true;
    }

    bool inRange(int index) const {
        return index>=0 && index<(*counter);
    }

    void clear() {
        for (int i=0; i<*counter; i++) {
            // shouldn't bodies be destroyed as well?
            // original code doesn't do this, so maybe it is ok?
            dGeomDestroy(get(i).getGeometry());
        }
        *counter = 0;
    }
};

template <class T>
class WorldObjectListOf : public WorldObjectList {
public:
    T *store;

    WorldObjectListOf(T *store, int len, int& counter, real3 *colors) :
        WorldObjectList(len,counter,colors),
        store(store)
    {
    }

    virtual WorldObject& get(int index) {
        return store[index];
    }

    virtual const WorldObject& get(int index) const {
        return store[index];
    }

};

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

    int SPHNUM;
	int waitSPH;
	int S_SPHNUM;
	
	int cylOBJNUM;
	int waitOBJ1;
	int S_cylOBJNUM;
	
	int waitMOD;
	int s_waitMOD;
	int MODEL_NUM;
	int s_MODEL_NUM;
	
	dReal color[100][3];
	dReal s_color[100][3];
	dReal color1[100][3];
	dReal s_color1[100][3];
    dReal color2[100][3];
	dReal s_color2[100][3];
	worldSimData();
};

class worldSim : public worldSimData {
public:
	static const bool textured = true;
    yarp::os::ConstString actWorld;
	dTriMeshDataID TriData[100];
	dTriMeshX trimesh[100];

    dTriMeshDataID s_TriData[100];
	dTriMeshX s_trimesh[100];
	
    // max number of objects
#define numObjJoints 5 //define Joints

	dJointID joint	[numObjJoints];
	dReal	 speed [numObjJoints];
	dJointID j;

	dBodyID tableBody[5];
	dGeomID tableGeom[5];

    dBodyID tempBody;
	dGeomID tempGeom[2];

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
	
	bool WAITLOADING;
	bool static_model;
    class MyObject : public WorldObject {
    public:
        dBodyID boxbody;			// the body
        dGeomID geom[GPB];		// geometries representing this body
        dReal size[3];

        virtual dBodyID getBody() const { return boxbody; }
        virtual dGeomID getGeometry() const { return geom[0]; }

        virtual bool create(const WorldOp& op, WorldResult& result, int idx);
    };

    MyObject obj[MAXNUM];
    MyObject s_obj[MAXNUM];

    WorldObjectListOf<MyObject> box_static;
    WorldObjectListOf<MyObject> box_dynamic;

    class MyObject1 : public WorldObject {
    public:
        dBodyID cylbody;			// the body
        dGeomID cylgeom[GPB];		// geometries representing this body
        dReal radius;
        dReal lenght;

        virtual dBodyID getBody() const { return cylbody; }
        virtual dGeomID getGeometry() const { return cylgeom[0]; }
        virtual bool create(const WorldOp& op, WorldResult& result, int idx);
    };

    MyObject1 cyl_obj[MAXNUM];
    MyObject1 s_cyl_obj[MAXNUM];

    WorldObjectListOf<MyObject1> cylinder_static;
    WorldObjectListOf<MyObject1> cylinder_dynamic;

    int modelTexture[100];
    int s_modelTexture[100];

    class MyObject2 : public WorldObject {
    public:
        dBodyID body;			// the body
        dGeomID geom;  		// geometries representing this body
        virtual dBodyID getBody() const { return body; }
        virtual dGeomID getGeometry() const { return geom; }
        virtual bool create(const WorldOp& op, WorldResult& result, int idx);
        virtual bool takeColor() { return false; }
    };
    MyObject2 ThreeD_obj[100];
    MyObject2 s_ThreeD_obj[100]; 

    WorldObjectListOf<MyObject2> model_static;
    WorldObjectListOf<MyObject2> model_dynamic;

    class MyObject3 : public WorldObject {
    public:
        dBodyID sphbody;			// the body
        dGeomID sphgeom[GPB];		// geometries representing this body
        dReal radius;
        virtual dBodyID getBody() const { return sphbody; }
        virtual dGeomID getGeometry() const { return sphgeom[0]; }
        virtual bool create(const WorldOp& op, WorldResult& result, int idx);
    };

    MyObject3 sph[MAXNUM];
    MyObject3 s_sph[MAXNUM];

    WorldObjectListOf<MyObject3> sphere_static;
    WorldObjectListOf<MyObject3> sphere_dynamic;

    yarp::os::ConstString texture;
    std::string model_DIR;
public:

	void resetSpeeds();
	void setJointSpeeds();
	void syncAngles();
	void ballDamping();
	void draw();
	void drawGeom(dGeomID g, const dReal *pos, const dReal *rot);//, float red  = 0.0f, float green = 128.5f, float blue = 255.0f);
    void setPosition(dReal agent1X, dReal agent1Z, dReal agent1Y );
	void activateWorld(RobotConfig& config);
	void init( dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
               RobotConfig& config);
	void loadTexture(yarp::os::ConstString texture, int numTexture);


	~worldSim();

	worldSim(dWorldID world, dSpaceID space, dReal X, dReal Y, dReal Z,
             RobotConfig& config);
};


#endif
