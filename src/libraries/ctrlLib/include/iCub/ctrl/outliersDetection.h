/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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



