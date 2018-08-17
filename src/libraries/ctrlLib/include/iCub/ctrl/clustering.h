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
 * \defgroup clustering Clustering
 *  
 * @ingroup ctrlLib
 *
 * Classes for clustering.
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __CLUSTERING_H__
#define __CLUSTERING_H__

#include <map>
#include <set>

#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

namespace iCub
{

namespace ctrl
{

/**
* \ingroup clustering
*
* Abstract class for clustering.
*/
class Clustering
{
public:
    /**
    * Cluster the provided data. 
    * @param data contains points to be clustered. 
    * @param options contains clustering options. 
    * @return clusters as a mapping between classes and the sets of
    *         elements indexes wrt the original data.
    */
    virtual std::map<size_t,std::set<size_t>> cluster(const std::vector<yarp::sig::Vector> &data,
                                                      const yarp::os::Property &options) = 0;

    /**
    * Virtual destructor.
    */
    virtual ~Clustering() { }
};


/**
* \ingroup clustering
*
* Data clustering based on DBSCAN algorithm. 
* 
* @note This implementation is based on the code available at
*       https://github.com/gyaikhom/dbscan.
*/
class DBSCAN : public Clustering
{
public:
    /**
    * Cluster the provided data.
    * @param data contains points to be clustered. 
    * @param options contains clustering options. The available 
    *                options are: "epsilon" representing the
    *                proximity sensitivity; "minpts" representing
    *                the minimum number of neighbours.
    * @return clusters as a mapping between classes and the sets of
    *         elements indexes wrt the original data.
    */
    std::map<size_t,std::set<size_t>> cluster(const std::vector<yarp::sig::Vector> &data,
                                              const yarp::os::Property &options) override;

    /**
    * Virtual destructor.
    */
    virtual ~DBSCAN() { }
};

}

}

#endif



