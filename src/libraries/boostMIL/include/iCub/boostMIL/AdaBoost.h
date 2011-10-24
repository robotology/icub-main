
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




#ifndef __ADABOOST__
#define __ADABOOST__

#include "StrongClassifier.h"


namespace iCub
{

namespace boostMIL
{


class AdaBoost: public StrongClassifier
{
private:
    /**
    * Maximum number of iteration allowed to the AdaBoost algorithm.
    */
    int                     max_iter;

    /**
    * Finds the WeakClassifier among the ones in the provided function space 
    * which perform best over the given trainig set with associated weight distribution.
    *
    * @param training_set reference to the list of training samples.
    * @param weights reference to the list of weights associated to each training input.
    */
    WeakClassifier          *best_classifier(const std::list<Inputs*> &training_set,
                                             const std::list<double> &weights);

    /**
    * Initializer method. Should be called whenever the class resource is changed.
    */
    virtual void                            initResource    ();

    /**
    * Online method (private and unimplemented on purpose).
    */
    virtual void                            update          (const Inputs *input, double &weight)   {}

    /**
    * Online method (private and unimplemented on purpose).
    */
    virtual void                            train           (const Inputs *input)                   {}

public:

    /**
    * Constructor.
    * It assumes that the name of the Strong Classifier is 'ada_boost'.
    *
    * @param _resource container for the classifier parameters.
    */
    AdaBoost            (yarp::os::ResourceFinder &_resource);
    
    /**
    * Constructor.
    *
    * @param _type type of the classifier. It is needed to associate particular inputs and to clone
    *              via the ClassifierFactory.
    * @param _resource container for the classifier parameters.
    */
    AdaBoost            (const std::string &_type, yarp::os::ResourceFinder &_resource);

    /**
    * Virtual Destructor
    */
    ~AdaBoost           ()                                              {clear();}

    /**
    * Clear method.
    */
    virtual void        clear       ();
    
    /**
    * Verifies if the AdaBoost is ready to be trained and to classify.
    */
    virtual bool        isReady     ()              {return true;}

    /**
    * Virtual Constructor. Creates an exact clone of the AdaBoost class.
    */
    virtual AdaBoost    *create     ()      const   {return new AdaBoost(*this);}

    /**
    * Train method. train the system over a given training set.
    * 
    * @params training_set the set of training samples.
    */
    virtual void        train       (const std::list<Inputs*> &training_set);
};

}

}

#endif


