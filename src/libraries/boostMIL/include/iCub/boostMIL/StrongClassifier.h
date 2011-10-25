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
 * \defgroup Strong Classifier
 *  
 * @ingroup boostMIL
 *
 * The result of the training process. The strong classifier learns how to linearly combine the output of a 
 * family of weak classifiers in order to provide predictions for.
 *
 *
 * \author Carlo Ciliberto
 * 
 * Copyright (C) 2010 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *  
 */ 



#ifndef __STRONG_CLASSIFIER__
#define __STRONG_CLASSIFIER__

#include <stdio.h>

#include <vector>
#include <deque>

#include <iCub/boostMIL/WeakClassifier.h>


namespace iCub
{

namespace boostMIL
{



class StrongClassifier: public WeakClassifier
{
protected:

    /**
    * List of the WeakClassifiers contributing the decision function of the StrongClassifier.
    */
    std::vector <WeakClassifier *>   weak_classifiers;

    /**
    * List of relevance weight for the WeakClassifier contributing the decision function of the StrongClassifier.
    */
    std::vector <double>             alphas;
    
    /**
    * List of WeakClassifiers available to train the StrongClassifier.
    */
    std::deque  <WeakClassifier *>   function_space;

public:

    /**
    * Constructor.
    *
    * @param _type type of the classifier. It is needed to associate particular inputs and to clone
    *              via the ClassifierFactory.
    * @param _resource container for the classifier parameters.
    */
    StrongClassifier            (const std::string &_type, yarp::os::ResourceFinder &_resource)
        :WeakClassifier(_type,_resource){}

    
    /**
    * Virtual Destructor.
    */
    virtual ~StrongClassifier   ()      {clear();}

    /**
    * Returns the current size of the List of WeakClassifiers.
    */
    virtual int getSize   ()      {return weak_classifiers.size();}

    /**
    * Clear Method. Returns the classifier to its original condition (Pre-initialization).
    */
    virtual void        clear       ();

    /**
    * Computes the margin of the given input. The margin is the response of the StrongClassifier
    * decision function to the given input.
    *
    * @param input the input provided to the classifier.
    * @return the margin of the given input scaled between 0 and 1.
    */
    double              margin      (const Inputs *input, Output *output=NULL)       const;

    /**
    * Prints the weights of all classifiers on a file
    *
    * @param input the input provided to the classifier.
    * @return the margin of the given input. It returns 0 if the classifier is not ready yet to classify the input.
    */
    void                printAlphas      (FILE *f);

    /**
    * Classifies the given input.
    *
    * @param input the input to classify.
    * @return the sign of the margin. Returns 0 if the margin is 0.
    */
    virtual int         classify    (const Inputs *input, Output *output=NULL)       const;


    //------- pure virtual methods -----------//

    /**
    * Virtual Constructor.
    */
    virtual StrongClassifier            *create     ()                                                  const   = 0;

    /**
    * Updates the classifier and verifies if it is ready to be trained and to classify.
    *
    * @return true if and only if the classifier is ready.
    */
    virtual bool                        isReady     ()                                                          = 0;

    /**
    * Updates the classifier with respect to an input with associated weight. Usually an online method.
    *
    * @param input the input.
    * @param the relevance weight associated to the given input.
    */
    virtual void                        update      (const Inputs *input, double &weight)                       = 0;

    /**
    * Trains the classifier with respect to a list of inputs.
    *
    * @param training_set the list of inputs over which train the classifier.
    */
    virtual void                        train       (const std::list<Inputs*> &training_set)              = 0;

    /**
    * Trains the classifier with respect to a single input. Usually an online method.
    *
    * @param input the inputs over which train the classifier.
    */
    virtual void                        train       (const Inputs *input)                                       = 0;






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
    * Saves the strong classifier.
    *
    * @param path where the classifier has to be saved.
    *
    * @return true/false in case of success/failure
    */
    virtual bool                        save        (const std::string &path) const;
    
    
    /**
    * Loads a previously saved strong classifier.
    *
    * @param path where the classifier has been saved.
    *
    * @return true/false in case of success/failure
    */
    virtual bool                        load        (const std::string &path);
};


}

}

#endif



