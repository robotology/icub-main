/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

/**
 * \defgroup outliersDetection Outliers Detection
 *  
 * @ingroup ctrlLib
 *
 * Classes for outliers detection.
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __OUTLIERSDETECTION_H__
#define __OUTLIERSDETECTION_H__

#include <map>
#include <set>

#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

namespace iCub
{

namespace ctrl
{

/**
* \ingroup outliersDetection
*
* Abstract class for outliers detection.
*/
class OutliersDetection
{
public:
    /**
    * Perform outliers detection over the provided data. 
    * @param data contains points to be verified. 
    * @param options contains detection options. 
    * @return set containing the outliers indexes.
    */
    virtual std::set<size_t> detect(const yarp::sig::Vector &data,
                                    const yarp::os::Property &options) = 0;

    /**
    * Virtual destructor.
    */
    virtual ~OutliersDetection() { }
};


/**
* \ingroup outliersDetection
*
* Perform modified Thompson tau technique for outlier detection.
*/
class ModifiedThompsonTau : public OutliersDetection
{
protected:
    std::map<size_t,double> tauLUP;
    std::set<size_t> recurIdx;

public:
    /**
    * Default constructor.
    */
    ModifiedThompsonTau();

    /**
    * Perform outliers detection over the provided data. \n 
    * Only one element is considered at a time, unless the 
    * <i>recursive</i> option is specified. 
    * @param data contains points to be verified. 
    * @param options contains detection options. If the properties
    *                <i>mean</i> and <i>std</i> are provided with
    *                corresponding doubles, then the computation of
    *                relative values is skipped. If the property
    *                <i>sorted</i> is provided, then the data are
    *                expected to be sorted either in ascending or
    *                descending order. If the property
    *                <i>recursive</i> is specified, then the
    *                detection is carried out over and over on the
    *                outcome of the previous instance until no more
    *                outliers are found.
    * @return set containing the outliers indexes.
    */
    std::set<size_t> detect(const yarp::sig::Vector &data,
                            const yarp::os::Property &options);

    /**
    * Virtual destructor.
    */
    virtual ~ModifiedThompsonTau() { }
};

}

}

#endif



