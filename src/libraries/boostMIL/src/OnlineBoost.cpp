
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

#include <stdio.h>
#include <math.h>
#include <cassert>
#include <stdexcept>
#include <sstream>
#include <string>

#include <algorithm>

#include <yarp/os/Bottle.h>

#include <iCub/boostMIL/OnlineBoost.h>
#include <iCub/boostMIL/SelectorClassifier.h>
#include <iCub/boostMIL/ClassifierFactory.h>


using namespace iCub::boostMIL;


OnlineBoost::OnlineBoost(yarp::os::ResourceFinder &_resource)
    :StrongClassifier("online_boost",_resource)
{
    initResource();
}

OnlineBoost::OnlineBoost(const std::string &_type, yarp::os::ResourceFinder &_resource)
    :StrongClassifier(_type,_resource)
{
    initResource();
}

void OnlineBoost::initResource()
{
    clear();

    if(resource.check(type.c_str()))
    {
        yarp::os::Bottle data = resource.findGroup(type.c_str());

        max_function_space_size=data.check("max_function_space_size",yarp::os::Value(0)).asInt();

        verbose=data.check("verbose");

        if(data.check("weak_classifiers_size"))
        {
            weak_classifiers_size = data.find("weak_classifiers_size").asInt();
            alphas.resize(weak_classifiers_size,0.0);

            std::string selector_type = data.check("selector_type",yarp::os::Value("selector")).asString().c_str();

            for(unsigned int i = 0; i < weak_classifiers_size; i++)
                weak_classifiers.push_back(new SelectorClassifier(selector_type,resource,&function_space));
        }
    }
}



bool OnlineBoost::isReady()
{
    //OnlineBoost is initialized if all the WeakClassifiers are initialized
    ready = true;
    if(weak_classifiers_size)
    {
        for(unsigned int i = 0; i < weak_classifiers.size(); i++)
             if(!weak_classifiers[i]->isReady()) ready = false;
    }
    else
        ready = false;
    return ready;
}



void OnlineBoost::train(const std::list<Inputs*> &training_set)
{
    int cnt=0;
    for(std::list<Inputs*>::const_iterator itr = training_set.begin(); itr != training_set.end(); itr++)
    {
        train(*itr);
        if(verbose) fprintf(stdout,"[BoostMIL] trained sample n. %d\r",++cnt);
    }
}

void OnlineBoost::train(const Inputs* input)
{
    double weight=1.0;
    update(input,weight);
}


void OnlineBoost::initialize(const Inputs *initializer)
{
    ClassifierFactory::instance().pack(initializer,function_space);
}

void OnlineBoost::initialize(const std::list<Inputs*> &initializer)
{
    for(std::list<Inputs*>::const_iterator itr=initializer.begin(); itr!=initializer.end(); itr++)
        ClassifierFactory::instance().pack((*itr),function_space);
}




void   OnlineBoost::fromString(const std::string &str)
{
    clear();

    yarp::os::Bottle bLoader(str.c_str());

    weak_classifiers_size=bLoader.size();

    for(int i=0; i<bLoader.size(); i++)
    {
        alphas.push_back(bLoader.get(i).asList()->find("alpha").asDouble());

        yarp::os::Bottle *bSelector=bLoader.get(i).asList()->find("weak_classifier").asList();

        weak_classifiers.push_back(new SelectorClassifier(bSelector->find("type").asString().c_str(),resource,&function_space));
        weak_classifiers.back()->fromString(bSelector->toString().c_str());
    }
}




void OnlineBoost::clearFunctionSpace()
{
    while(function_space.size())
    {
        delete function_space.front();
        function_space.pop_front();
    }
}

void OnlineBoost::update(const Inputs *input, double &weight)
{
    //if the algorithm uses a function space, extract new weak learners from the input (if it is a positive sample).
    if(function_space.size()<max_function_space_size && input->getLabel() == +1)
    {
        ClassifierFactory::instance().pack(input,function_space);

        //avoid getting a function space too large
        while(function_space.size() > max_function_space_size)
        {
            delete function_space.front();
            function_space.pop_front();
        }
    }

    if(isReady())
    {
        //propagate the sample through the list of weak classifiers
        for(unsigned int i = 0; i < weak_classifiers.size(); i++)
        {
            weak_classifiers[i]->update(input,weight);

            //if the WeakClassifiers abstains from classifing, do not update the weight
            if(weak_classifiers[i]->classify(input) != 0)
            {
                if(weak_classifiers[i]->classify(input) == input->getLabel())
                    weight *= 0.5/(1.0-weak_classifiers[i]->getError());
                else
                    weight *= 0.5/weak_classifiers[i]->getError();
            }

            //update alpha
            alphas[i] = 0.5*log((1-weak_classifiers[i]->getError())/weak_classifiers[i]->getError());

        }
    }
}



void OnlineBoost::clear()
{
    StrongClassifier::clear();

    weak_classifiers_size = 0;
}


void OnlineBoost::randomizeFuncSpace()
{
    random_shuffle(function_space.begin(),function_space.end());
}



