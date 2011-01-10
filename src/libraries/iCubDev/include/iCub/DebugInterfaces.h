// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Giorgio Metta
 * email: giorgio.metta@iit.it
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

/**
 * @file DebugInterfaces.h
 * @brief A collection of debug methods useful to send raw commands directly to the control boards.
 */

/**
 * \defgroup icub_icubDev iCubDev
 * \ingroup icub_libraries
 *
 * A library that collects device interfaces. This is similar to the 
 * libYARP_dev in YARP. To be populated.
 *
 * Author: Marco Randazzo
 * Copyright (C) 2009 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef __DEBUGINTERFACES__
#define __DEBUGINTERFACES__

#include <math.h>
#include <string.h> // for memset
#include <stdlib.h> // for exit
#include <stdio.h> // for printf

// additional vocabs defined for the IDebug interface.
#define VOCAB_GENERIC_PARAMETER VOCAB4('g','e','n','p')
#define VOCAB_DEBUG_PARAMETER VOCAB4('d','b','g','p')

/* LATER: is it likely that some of these would move into iCub::dev namespace? */
namespace yarp{
    namespace dev {
        class IDebugInterface;
		class IDebugInterfaceRaw;
		class ImplementDebugInterface;
    }
}

#define _YARP_ASSERT_DEBUG(x) { if (!(x)) { printf("memory allocation failure\n"); /*yarp::os::exit(1);*/ } }

template <class T>
inline T* allocAndCheckDebug(int size)
{
    T* t = new T[size];
    _YARP_ASSERT (t != 0);
    memset(t, 0, sizeof(T) * size);
    return t;
}

template <class T>
inline void checkAndDestroyDebug(T* &p) {
    if (p!=0) {
        delete [] p;
        p = 0;
    }
}

class ControlBoardHelper2
{
public:
    ControlBoardHelper2(int n, const int *aMap, const double *angToEncs, const double *zs, const double *nw): zeros(0), 
        signs(0),
        axisMap(0),
        invAxisMap(0),
        angleToEncoders(0),
		newtonsToSensors(0)
    {
        nj=n;
        alloc(n);
        
        memcpy(axisMap, aMap, sizeof(int)*nj);
        
        if (zs!=0)
            memcpy(zeros, zs, sizeof(double)*nj);
        else
            memset(zeros, 0, sizeof(double)*nj);

        if (angToEncs!=0)
            memcpy(angleToEncoders, angToEncs, sizeof(double)*nj);
        else
            memset(angleToEncoders, 0, sizeof(double)*nj);

		if (nw!=0)
            memcpy(newtonsToSensors, nw, sizeof(double)*nj);
        else
            memset(newtonsToSensors, 0, sizeof(double)*nj);

        // invert the axis map
   		memset (invAxisMap, 0, sizeof(int) * nj);
		int i;
        for (i = 0; i < nj; i++)
		{
			int j;
			for (j = 0; j < nj; j++)
			{
				if (axisMap[j] == i)
				{
					invAxisMap[i] = j;
					break;
				}
			}
		}

    } 

    ~ControlBoardHelper2() 
    {
        dealloc();
    }

    bool alloc(int n)
    {
        nj=n;
        if (nj<=0)
            return false;

        if (zeros!=0)
            dealloc();

        zeros=new double [nj];
        signs=new double [nj];
        axisMap=new int [nj];
        invAxisMap=new int [nj];
        angleToEncoders=new double [nj];
		newtonsToSensors=new double [nj];
        _YARP_ASSERT_DEBUG(zeros != 0 && signs != 0 && axisMap != 0 && invAxisMap != 0 && angleToEncoders != 0 && newtonsToSensors != 0);

        return true;
    }

    bool dealloc()
    {
        checkAndDestroyDebug<double> (zeros);
        checkAndDestroyDebug<double> (signs);
        checkAndDestroyDebug<int> (axisMap);
        checkAndDestroyDebug<int> (invAxisMap);
        checkAndDestroyDebug<double> (angleToEncoders);
		checkAndDestroyDebug<double> (newtonsToSensors);
        return true;
    }

    inline int toHw(int axis)
    { return axisMap[axis]; }
    
    inline int toUser(int axis)
    { return invAxisMap[axis]; }

    //map a vector, no conversion
    inline void toUser(const double *hwData, double *user)
    {
        for (int k=0;k<nj;k++)
         user[toUser(k)]=hwData[k];
    }

     //map a vector, no conversion
    inline void toUser(const int *hwData, int *user)
    {
        for (int k=0;k<nj;k++)
         user[toUser(k)]=hwData[k];
    }

    //map a vector, no conversion
    inline void toHw(const double *usr, double *hwData)
    {
        for (int k=0;k<nj;k++)
            hwData[toHw(k)]=usr[k];
    }

     //map a vector, no conversion
    inline void toHw(const int *usr, int *hwData)
    {
        for (int k=0;k<nj;k++)
            hwData[toHw(k)]=usr[k];
    }

    inline void posA2E(double ang, int j, double &enc, int &k)
    {
        enc=(ang+zeros[j])*angleToEncoders[j];
        k=toHw(j);
    }

    inline double posA2E(double ang, int j)
    {
        return (ang+zeros[j])*angleToEncoders[j];
    }

    inline void posE2A(double enc, int j, double &ang, int &k)
    {
        k=toUser(j);

        ang=(enc/angleToEncoders[k])-zeros[k];
    }

    inline double posE2A(double enc, int j)
    {
        int k=toUser(j);
        
        return (enc/angleToEncoders[k])-zeros[k];
    }

	inline void impN2S(double newtons, int j, double &sens, int &k)
    {
        sens=newtons*newtonsToSensors[j]/angleToEncoders[j];
        k=toHw(j);
    }

    inline double impN2S(double newtons, int j)
    {
        return newtons*newtonsToSensors[j]/angleToEncoders[j];
    }

    inline void impN2S(const double *newtons, double *sens)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            impN2S(newtons[j], j, tmp, index);
            sens[index]=tmp;
        }
	}

    inline void trqN2S(double newtons, int j, double &sens, int &k)
    {
        sens=newtons*newtonsToSensors[j];
        k=toHw(j);
    }

    inline double trqN2S(double newtons, int j)
    {
        return newtons*newtonsToSensors[j];
    }

	//map a vector, convert from newtons to sensors
    inline void trqN2S(const double *newtons, double *sens)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            trqN2S(newtons[j], j, tmp, index);
            sens[index]=tmp;
        }
    }

	//map a vector, convert from sensor to newtons
    inline void trqS2N(const double *sens, double *newtons)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            trqS2N(sens[j], j, tmp, index);
            newtons[index]=tmp;
        }
    }

	inline void trqS2N(double sens, int j, double &newton, int &k)
    {
        k=toUser(j);

        newton=(sens/newtonsToSensors[k]);
    }

    inline double trqS2N(double sens, int j)
    {
        int k=toUser(j);
        
        return (sens/newtonsToSensors[k]);
    }

	inline void impS2N(const double *sens, double *newtons)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            impS2N(sens[j], j, tmp, index);
            newtons[index]=tmp;
        }
    }

	inline void impS2N(double sens, int j, double &newton, int &k)
    {
        k=toUser(j);

        newton=(sens/newtonsToSensors[k]*angleToEncoders[k]);
    }

    inline double impS2N(double sens, int j)
    {
        int k=toUser(j);
        
        return (sens/newtonsToSensors[k]*angleToEncoders[k]);
    }

    inline void velA2E(double ang, int j, double &enc, int &k)
    {
        k=toHw(j);
        enc=ang*angleToEncoders[j];
    }

	inline void velA2E_abs(double ang, int j, double &enc, int &k)
    {
        k=toHw(j);
        enc=ang*fabs(angleToEncoders[j]);
    }

    inline void velE2A(double enc, int j, double &ang, int &k)
    {
        k=toUser(j);
        ang=enc/angleToEncoders[k];
    }

	inline void velE2A_abs(double enc, int j, double &ang, int &k)
    {
        k=toUser(j);
        ang=enc/fabs(angleToEncoders[k]);
    }

    inline void accA2E(double ang, int j, double &enc, int &k)
    {
        velA2E(ang, j, enc, k);
    }
	
	inline void accA2E_abs(double ang, int j, double &enc, int &k)
    {
        velA2E_abs(ang, j, enc, k);
    }

    inline void accE2A(double enc, int j, double &ang, int &k)
    {
        velE2A(enc, j, ang, k);
    }

	inline void accE2A_abs(double enc, int j, double &ang, int &k)
    {
        velE2A_abs(enc, j, ang, k);
    }

    inline double velE2A(double enc, int j)
    {
        int k=toUser(j);
        return enc/angleToEncoders[k];
    }

	inline double velE2A_abs(double enc, int j)
    {
        int k=toUser(j);
        return enc/fabs(angleToEncoders[k]);
    }


    inline double accE2A(double enc, int j)
    {
        return velE2A(enc, j);
    }

	inline double accE2A_abs(double enc, int j)
    {
        return velE2A_abs(enc, j);
    }

    //map a vector, convert from angles to encoders
    inline void posA2E(const double *ang, double *enc)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            posA2E(ang[j], j, tmp, index);
            enc[index]=tmp;
        }
    }

    //map a vector, convert from encoders to angles
    inline void posE2A(const double *enc, double *ang)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            posE2A(enc[j], j, tmp, index);
            ang[index]=tmp;
        }
    }

    inline void velA2E(const double *ang, double *enc)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            velA2E(ang[j], j, tmp, index);
            enc[index]=tmp;
        }
    }

	inline void velA2E_abs(const double *ang, double *enc)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            velA2E_abs(ang[j], j, tmp, index);
            enc[index]=tmp;
        }
    }

    inline void velE2A(const double *enc, double *ang)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            velE2A(enc[j], j, tmp, index);
            ang[index]=tmp;
        }
    }

	inline void velE2A_abs(const double *enc, double *ang)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            velE2A_abs(enc[j], j, tmp, index);
            ang[index]=tmp;
        }
    }

    inline void accA2E(const double *ang, double *enc)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            accA2E(ang[j], j, tmp, index);
            enc[index]=tmp;
        }
    }

	inline void accA2E_abs(const double *ang, double *enc)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            accA2E_abs(ang[j], j, tmp, index);
            enc[index]=tmp;
        }
    }

    inline void accE2A(const double *enc, double *ang)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            accE2A(enc[j], j, tmp, index);
            ang[index]=tmp;
        }
    }

	inline void accE2A_abs(const double *enc, double *ang)
    {
        double tmp;
        int index;
        for(int j=0;j<nj;j++)
        {
            accE2A_abs(enc[j], j, tmp, index);
            ang[index]=tmp;
        }
    }

    inline int axes()
    { return nj; }
        
 	int nj;

	double *zeros;
	double *signs;
	int *axisMap;
	int *invAxisMap;
	double *angleToEncoders;
	double *newtonsToSensors;
};

inline ControlBoardHelper2 *castToMapper2(void *p)
{ return static_cast<ControlBoardHelper2 *>(p); }


inline ControlBoardHelper2 *castToMapper2(void *p);


/**
 * \ingroup icub_icubDev
 *
 * Debug Interface
 */
class yarp::dev::IDebugInterface {
public:
    virtual ~IDebugInterface() {}

    /* Set a generic parameter (for debug)
     * @param type is the CAN code representing the command message 
     * @return true/false on success/failure
     */
    virtual bool setParameter(int j, unsigned int type, double value)=0;

    /* Get a generic parameter (for debug)
     * @param type is the CAN code representing the command message 
     * @return true/false on success/failure
     */
    virtual bool getParameter(int j, unsigned int type, double* value)=0;

    /* Set a generic parameter (for debug)
	 * @param index is the number of the debug parameter
     * @return true/false on success/failure
     */
    virtual bool setDebugParameter(int j, unsigned int index, double value)=0;

    /* Get a generic parameter (for debug)
	 * @param index is the number of the debug parameter
     * @return true/false on success/failure
     */
    virtual bool getDebugParameter(int j, unsigned int index, double* value)=0;
};

class yarp::dev::IDebugInterfaceRaw {
public:
    virtual ~IDebugInterfaceRaw() {}

    /* Set a generic parameter (for debug)
	 * @param type is the CAN code representing the command message 
     * @return true/false on success/failure
     */
    virtual bool setParameterRaw(int j, unsigned int type, double value)=0;

    /* Get a generic parameter (for debug)
     * @param type is the CAN code representing the command message 
     * @return true/false on success/failure
     */
    virtual bool getParameterRaw(int j, unsigned int type, double* value)=0;

    /* Set a generic parameter (for debug)
     * @param index is the number of the debug parameter
     * @return true/false on success/failure
     */
    virtual bool setDebugParameterRaw(int j, unsigned int index, double value)=0;

    /* Get a generic parameter (for debug)
	 * @param index is the number of the debug parameter
     * @return true/false on success/failure
     */
    virtual bool getDebugParameterRaw(int j, unsigned int index, double* value)=0;
};

struct DebugParameters
{
	double data[8];

	DebugParameters() {for (int i=0; i<8; i++) data[i]=0;}
};

class yarp::dev::ImplementDebugInterface: public IDebugInterface
{
    void *helper;
    yarp::dev::IDebugInterfaceRaw *raw;
    double *dummy;
public:
    bool initialize(int k, const int *amap);
    bool uninitialize();
    ImplementDebugInterface(IDebugInterfaceRaw *v);
    ~ImplementDebugInterface();
	bool setParameter(int j, unsigned int type, double value);
    bool getParameter(int j, unsigned int type, double* value);
	bool setDebugParameter(int j, unsigned int index, double value);
    bool getDebugParameter(int j, unsigned int index, double *value);
};

#endif /* __DEBUGINTERFACES__ */
