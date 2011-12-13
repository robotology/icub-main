/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Carlo Ciliberto
 * email:  carlo.ciliberto@iit.it
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
 * \defgroup MIL Classifier
 *  
 * @ingroup boostMIL
 *
 * A weak classifier implementing the Multiple Instance Learning in an online boosting framework.
 *
 * \author Carlo Ciliberto
 * 
 * Copyright (C) 2010 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *  
 */ 



#ifndef __MIL_CLASSIFIER__
#define __MIL_CLASSIFIER__

#include <deque>

#include <yarp/sig/Vector.h>

#include "WeakClassifier.h"


namespace iCub
{

namespace boostMIL
{



struct Bag
{
    double      dist;
    double      weight;
    int         label;

    Bag()
    {}

    Bag(const double &_dist, const int &_label, const double &_weight)
        :dist(_dist),weight(_weight),label(_label)
    {}


    std::string  toString() const
    {
        std::stringstream strstr;

        strstr << "(dist " << dist << ") ";
        strstr << "(weight " << weight << ") ";
        strstr << "(label " << label << ") ";

        return strstr.str().c_str();
    }

    void fromString(const std::string &str)
    {
        yarp::os::Bottle bLoader(str.c_str());

        dist=bLoader.find("dist").asDouble();
        weight=bLoader.find("weight").asDouble();
        label=bLoader.find("label").asInt();
    }
};


class MILClassifier: public WeakClassifier
{
protected:
    //information regarding the center feature
    int                     feature_size;
    yarp::sig::Vector       center;
    double                  radius;

    //list of bags seen so far
    std::deque<Bag>         cache;
    int                     max_cache_size;
    double                  posweight;
    double                  negweight;

    //private methods
    virtual void        initResource        ();
    virtual double      distanceFromCenter  (const double *vec) const;
    virtual void        setCenter           (const double *_center);
    void                insertBag           (const double &dist, const int &label, const double &weight);
    double              bestRadius          ();

public:
    //Constructors
    MILClassifier   (const std::string &_type, yarp::os::ResourceFinder &_resource, const double *_center = NULL);
    MILClassifier   (const MILClassifier &mil, const double *_center = NULL);

    //Virtual destructor
    ~MILClassifier  (){clear();}


    //Public methods
    virtual void                            clear           ();
    virtual bool                            isReady         ();






    /**
    * Encodes the classifier in a single string
    *
    * @return a string encoding the current classifier form
    */
    virtual std::string                  toString    () const;

    /**
    * Loads a classifier from a string
    *
    * @param the string encoding the classifier to be loaded
    */
    virtual void                         fromString  (const std::string &str);



    /**
    * Classifies the given input.
    *
    * @param input the input to classify.
    * @return 1,-1 or 0 if the classifier classifies the input (respectively): 
    *         positive, negative or undetermined (e.g. the classifier is not ready to classify yet).
    */
    virtual int                             classify        (const Inputs *input, Output *output)   const;
    virtual void                            update          (const Inputs *input, double &weight);
    virtual double                          error_rate      (const std::list<const Inputs *> &dataset, std::list<double> &weights);

    virtual std::list<WeakClassifier*>      *pack           (const Inputs *input)   const;
    virtual MILClassifier                   *create         ()                      const   {return new MILClassifier(*this);}
};


class HOGClassifier: public MILClassifier
{
    int NH,NS,NV;
    int n_levels;
    int hog_bins;
    int feature_segment;

    //private methods
    virtual void        initResource        ();
    virtual double      distanceFromCenter  (const double *vec) const;

public:
    //Constructors
    HOGClassifier   (const std::string &_type, yarp::os::ResourceFinder &_resource, const double *_center = NULL);
    HOGClassifier   (const HOGClassifier &hog, const double *_center = NULL);

    //Virtual destructor
    ~HOGClassifier  (){}
};



class CorrClassifier: public MILClassifier
{
    int win_init_size;
    int win_increment;
    int n_levels;

    //private methods
    virtual void        initResource        ();
    virtual double      distanceFromCenter  (const double *vec) const;

public:
    //Constructors
    CorrClassifier   (const std::string &_type, yarp::os::ResourceFinder &_resource, const double *_center = NULL);
    CorrClassifier   (const CorrClassifier &corr, const double *_center = NULL);

    //Virtual destructor
    ~CorrClassifier  (){}
};


class DictionaryClassifier: public MILClassifier
{
    int                 index;
    bool                fabsize;

    virtual void        initResource        ();
    virtual double      distanceFromCenter  (const double *vec) const;
    virtual void        setCenter(const double *_center);

public:
    DictionaryClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, const double *_center = NULL);
    DictionaryClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, int _index, const double *_center = NULL);
    DictionaryClassifier(const DictionaryClassifier &dict, const double *_center = NULL);

    virtual std::list<WeakClassifier*>      *pack           (const Inputs *input)   const;

    virtual bool                            isReady         ();
    //Virtual destructor
    ~DictionaryClassifier(){}
};

}

}

#endif



