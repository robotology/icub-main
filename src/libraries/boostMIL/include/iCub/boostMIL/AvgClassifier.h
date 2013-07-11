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
 * \defgroup Selector Classifier
 *  
 * @ingroup boostMIL
 *
 * A weak classifier used for online learning which allows to keep perform feature selection. Originally 
 * proposed in [<a href="http://www.google.it/url?sa=t&rct=j&q=online%20boosting%20an%20vision&source=web&cd=1&ved=0CCMQFjAA&url=http%3A%2F%2Fwww.icg.tu-graz.ac.at%2FMembers%2Fhgrabner%2Fpub_hgrabner%2Fgrabner2006OnlineBoosting.pdf%2Fat_download%2Ffile&ei=F4OlToC2A6H64QTVs4HuBA&usg=AFQjCNH6lmEE4tff6UhkMR-IUPL9-DzhIQ&sig2=XB_8z7O0ES1P5ecedlpvHw">PDF</a>] 
 *
 *
 * \author Carlo Ciliberto
 * 
 * Copyright (C) 2010 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *  
 */ 



#ifndef __AVG_CLASSIFIER__
#define __AVG_CLASSIFIER__

#include <vector>

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



class AvgClassifier: public WeakClassifier
{
private:
    /**
    * Size allowed for the pool of WeakClassifiers
    */
    int                                 pool_size;
    
    /**
    * Actual list (or pool) of WeakClassifiers allowed to the Selector.
    */
    std::vector<WeakClassifier*>        pool;

    double                              threshold;
    bool                                threshold_fixed;
    std::deque<Bag>                     cache;
    double                              posweight;
    double                              negweight;



    std::vector<double>                 correct;
    std::vector<double>                 wrong;

    std::vector<double>                 alpha;


    //pointer to the function space passed during construction
    std::deque<WeakClassifier*>         *function_space;

    //private methods
    virtual void                        initResource();
    virtual bool                        substitute(const double &e) const;
    virtual WeakClassifier              *randomSample();

    //private and unimplemented on purpose (for now)
    virtual std::list<WeakClassifier*>  *pack       (const Inputs *input)           const   {return NULL;}
    virtual WeakClassifier              *create     ()                              const   {return NULL;}

    void                insertBag           (const double &dist, const int &label, const double &weight);
    double              bestRadius          ();


public:
    AvgClassifier      (const int &_type, yarp::os::ResourceFinder &_resource, std::deque<WeakClassifier*> *_function_space);
    AvgClassifier      (const std::string &_type, yarp::os::ResourceFinder &_resource, std::deque<WeakClassifier*> *_function_space);

    ~AvgClassifier     ()      {clear();}


    //public methods
    virtual bool        isReady     ();
    virtual void        clear       ();

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
    virtual void                         fromStream  (std::ifstream &fin);

    /**
    * Encodes the classifier in a single string
    *
    */
    virtual void                         toStream    (std::ofstream &fout) const;

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
    virtual int         classify    (const Inputs *input, Output *output)   const;
    
    /**
    * Margin given the input.
    *
    * @param input the input to classify.
    * @return the margin
    */
    virtual double         margin    (const Inputs *input, Output *output)   const;
    
    virtual void        update      (const Inputs *input, double &weight);
};


}

}

#endif


