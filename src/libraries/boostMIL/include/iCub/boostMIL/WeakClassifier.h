
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
 * \defgroup boostMIL boostMIL
 *  
 * @ingroup icub_libraries 
 *  
 * Library of boost learning methods. The current status of the library implements just 
 * Online Multiple Instance Learning Boosting
 *
 *
 * \author Carlo Ciliberto
 *  
 * \defgroup Weak Classifier
 *  
 * @ingroup boostMIL
 *
 * The basic element of boosting. A weak classifier is a binary function which takes an input X
 * and returns a positive (1) or negative (-1) output.
 *
 * \author Carlo Ciliberto
 * 
 * Copyright (C) 2010 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *  
 */ 


#ifndef __WEAK_CLASSIFIER__
#define __WEAK_CLASSIFIER__

#include <list>
#include <yarp/os/ResourceFinder.h>
#include <iCub/boostMIL/ClassifierInput.h>



namespace iCub
{

namespace boostMIL
{


class WeakClassifier
{
protected:
    /**
    * Resource containing the parameters information
    */
    yarp::os::ResourceFinder        &resource;

    /**
    * The type associated to the WeakClassifier
    */
    std::string                     type;

    /**
    * Last computed error.
    */
    double                          error;

    /**
    * Checks if the WeakClassifier has been initialized.
    */
    bool                            ready;


    /**
    * Allows prints by the boostMIL library
    */
    bool                            verbose;



private:
    /**
    * Constructor (private and unimplemented on purpose).
    */
    WeakClassifier();

public:

    /**
    * Constructor.
    *
    * @param _type type of the classifier. It is needed to associate particular inputs and to clone
    *              via the ClassifierFactory.
    * @param _resource container for the classifier parameters.
    */
    WeakClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource)
      :type(_type),resource(_resource)
    {}

    /**
    * Copy Constructor.
    *
    * @param wc the WeakClassifier from which the copy is instantiated.
    */
    WeakClassifier(const WeakClassifier &wc)
        :type(wc.type),resource(wc.resource)
    {}

    /**
    * Virtual Destructor
    */
    virtual ~WeakClassifier(){}

    
    /**
    * Clear Method. Returns the classifier to its original condition (Pre-initialization).
    */
    virtual void                        clear       ()                                                   = 0;

    /**
    * Returns the classifier type information
    * 
    * @return the type of the classifier
    */
    std::string                              getType     ()      const   {return type;}
    
    /**
    * Returns the current error exhibited by the classifier
    * 
    * @return the current error.
    */
    double                                   getError    ()      const   {return error;}

    /**
    * Extract the input information relative to this classifier type and creates a list of WeakClassifiers
    * from it. Used for online methods to create the space of hypothesys directly from training data.
    * Usually returns a null pointer.
    *
    * @param input the input from which extract the WeakClassifiers.
    * @return a list of pointers to the WeakClassifiers extracted from the input.
    */
    virtual std::list<WeakClassifier*>  *pack       (const Inputs *input)                        const  {return NULL;}

    /**
    * Returns the error rate of the classifier over a given dataset assuming uniform weight distribution
    *
    * @param dataset the datataset over which compute the error rate.
    * @return the error rate.
    */
    virtual double                      error_rate  (const std::list<Inputs *> &dataset)
    {
        std::list<double> weights(dataset.size(),1.0/dataset.size());
        return error_rate(dataset,weights);
    }

    /**
    * Returns the error rate of the classifier over a given dataset with specified weight distribution
    *
    * @param dataset the datataset over which compute the error rate.
    * @param weights the weight distribution over the given dataset.
    * @return the error rate.
    */
    virtual double                      error_rate  (const std::list<Inputs *> &dataset, const std::list<double> &weights)
    {
        double e = 0.0;
        std::list<Inputs*>   ::const_iterator d_itr = dataset.begin();
        std::list<double>    ::const_iterator w_itr = weights.begin();
        while(d_itr != dataset.begin())
        {
            if(classify(*d_itr) != (*d_itr)->getLabel())
                e += *w_itr;
            d_itr++;
        }
        return e;
    }

    
    //------------ pure virtual methods ------------//


    /**
    * Encodes the classifier in a single string
    *
    * @return a string encoding the current classifier form
    */
    virtual std::string                  toString    () const = 0;

    /**
    * Loads a classifier from a string
    *
    * @param the string encoding the classifier to be loaded
    */
    virtual void                         fromString  (const std::string &str) = 0;

    /**
    * Updates the classifier with respect to an input with associated weight. Usually an online method.
    *
    * @param input the input.
    * @param the relevance weight associated to the given input.
    */
    virtual void                        update      (const Inputs *input, double &weight)                = 0;


    /**
    * Updates the classifier and verifies if it is ready to be trained and to classify.
    *
    * @return true if and only if the classifier is ready.
    */
    virtual bool                        isReady     ()                                                   = 0;

    /**
    * Classifies the given input.
    *
    * @param input the input to classify.
    * @return 1,-1 or 0 if the classifier classifies the input (respectively): 
    *         positive, negative or undetermined (e.g. the classifier is not ready to classify yet).
    */
    virtual int                         classify    (const Inputs *input, Output *output=NULL)      const   = 0;

    /**
    * Virtual Constructor.
    */
    virtual WeakClassifier              *create     ()                                           const   = 0;
};


}

}

#endif


