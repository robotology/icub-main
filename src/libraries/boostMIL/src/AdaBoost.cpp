
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

#include <iostream>
#include <string>
#include <math.h>

#include <cassert>
#include <stdexcept>
#include <sstream>


#include <yarp/os/Bottle.h>

#include <iCub/boostMIL/AdaBoost.h>
#include <iCub/boostMIL/ClassifierFactory.h>


using namespace iCub::boostMIL;

//Constructors
AdaBoost::AdaBoost(yarp::os::ResourceFinder &_resource)
    :StrongClassifier("ada_boost",_resource)
{
    initResource();
}

//Constructors
AdaBoost::AdaBoost(const std::string &_type, yarp::os::ResourceFinder &_resource)
    :StrongClassifier(_type,_resource)
{
    initResource();
}






//Public Methods
void                AdaBoost::initResource()
{
    clear();

    if(resource.check(type.c_str()))
    {
        yarp::os::Bottle data = resource.findGroup(type.c_str());
    
        max_iter = data.check("max_iter",yarp::os::Value(50)).asInt();

        //load the function space
        std::string loader_type = data.check("loader",yarp::os::Value(type.c_str())).asString().c_str();
        //DatasetLoader::load(loader_type,resource,&function_space);
    }
}




void                AdaBoost::clear()
{
    StrongClassifier::clear();
    max_iter = 0;
}




void                AdaBoost::train(const std::list<Inputs*> &training_set)
{
    //check if the classifier is ready
    if(!isReady())
    {
        std::cout << "AdaBoost not yet initialized" << std::endl;
        return;
    }

    if(alphas.size())
        std::cout << "AdaBoost already trained. Training will continue." << std::endl;

    //initialize the weights
    std::list<double> weights(training_set.size(),(double) 1.0/training_set.size());

    for(int t = 0; t < max_iter; t++)
    {
        //find and clone the current best weak learner
        WeakClassifier *best_w = best_classifier(training_set,weights);
        if(best_w->getError() > 0.5)
        {
            std::cout << "There are not weak learners better than random guess." << std::endl;
            std::cout << "Training is stopped." << std::endl;
            break;
        }
        //add the weak learner to the strong classifier with corresponding alpha
        weak_classifiers.push_back(best_w);
        if(best_w->getError() < 1e-7)
        {
            std::cout << "The Weak Learner at iteration n. " << t << " solves the entire problem." << std::endl;
            alphas.push_back(1.0);
            break;
        }
        else
            alphas.push_back(0.5*log((1-best_w->getError())/best_w->getError()));


        //update the weights
        double norm = 0.0;
        std::list<Inputs*>::const_iterator t_itr = training_set.begin();
        std::list<double>::iterator w_itr = weights.begin();
        while(t_itr != training_set.end() && w_itr != weights.end())
        {
            int d = best_w->classify(*t_itr)*(*t_itr)->getLabel();
            *w_itr *= exp(-alphas[t]*d);
            norm += *w_itr;
            t_itr++;
            w_itr++;
        }

        w_itr = weights.begin();
        while(w_itr != weights.end())
        {
            *w_itr /= norm;
            w_itr++;
        }

    }

}





//Private Methods
WeakClassifier      *AdaBoost::best_classifier(const std::list<Inputs *> &training_set, const std::list<double> &weights)
{
    double best_error=1.0;
    WeakClassifier *best_wl;

    //find the weak classifier with lowest error rate
    for(unsigned int i = 0; i < function_space.size(); i++)
    {
        double error = function_space[i]->error_rate(training_set,weights);
        if(error < best_error)
        {
            best_error = error;
            best_wl = function_space[i];
        }
    }

    //clone the weak classifier
    WeakClassifier *w = best_wl->create();
    //check if the (copied) best_wl is ready
    if(!w->isReady())
    {
        std::ostringstream buffer;
        buffer << "The chosen weak learner is not ready yet";
        throw std::runtime_error(buffer.str());
    }

    return w;
}


