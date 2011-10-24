
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
#include <deque>
#include <math.h>


#include <fstream>
#include <cassert>
#include <stdexcept>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/math/Rand.h>

#include <iCub/boostMIL/SelectorClassifier.h>
#include <iCub/boostMIL/ClassifierFactory.h>



using namespace iCub::boostMIL;


SelectorClassifier::SelectorClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, std::deque<WeakClassifier*> *_function_space)
    :WeakClassifier(_type,_resource),function_space(_function_space)
{
    //if a function space is not provided. the sytem fails.
    if(function_space == NULL)
    {
        std::ostringstream buffer;
        buffer << "Error! Function Space not available!";
        throw std::runtime_error(buffer.str());
    }

    initResource();
    //initialize the random generator.
    yarp::math::Rand::init();
}



void SelectorClassifier::initResource()
{
    clear();

    if(resource.check(type.c_str()))
    {
        yarp::os::Bottle data = resource.findGroup(type.c_str());

        pool_size = data.check("pool_size",yarp::os::Value(1)).asInt();
        abs_thresh = data.check("abs_thresh",yarp::os::Value(1.0)).asDouble();
        rel_thresh = data.check("rel_thresh",yarp::os::Value(1e10)).asDouble();
        sleep_time = data.check("sleep_time",yarp::os::Value(0)).asInt();
    }
    else
    {
        pool_size = 1;
        abs_thresh = 1.0;
        rel_thresh = 1e10;
        sleep_time = 0;
    }

    //initialize the vectors
    for(int i = 0; i < pool_size; i++)
    {
        correct.push_back(1.0);
        wrong.push_back(1.0);
        asleep.push_back(sleep_time);
    }
}


void SelectorClassifier::clear()
{
    selected = -1;

    pool_size = 0;
    while(pool.size())
    {
        delete pool.back();
        pool.pop_back();
    }

    correct.clear();
    wrong.clear();
    asleep.clear();
    sleep_time = 0;
    abs_thresh = 0.0;
    rel_thresh = 0.0;
}


int SelectorClassifier::classify(const Inputs *input, Output *output) const
{
    //The selector abstains from classifing if: it has not been initialized
    if(!ready || selected == -1 || asleep[selected] >= 0)
        return 0;
    else
        return pool[selected]->classify(input,output);
}

bool SelectorClassifier::isReady()
{
    ready = true;
    //if the selector is not empty
    if(pool_size)
    {
        //fill the pool with new WeakClassifiers with the elements in the function_space list
        while(pool.size() < (unsigned) pool_size && function_space->size())
        {
            pool.push_back(function_space->front());
            function_space->pop_front();
        }

        //if the selector is still not completly filled it is not ready
        if(pool.size() != pool_size)
            ready = false;

        for(unsigned int i = 0; i < pool.size(); i++)
            if(!pool[i]->isReady()) ready = false;
    }
    else
        ready = false;

    return ready;
}




void SelectorClassifier::update(const Inputs *input, double &weight)
{
    if(isReady())
    {
        error = 1.0;
        std::vector<double> errors(pool.size());

        for(unsigned int i = 0; i < pool.size(); i++)
        {
            pool[i]->update(input,weight);

            //estimate the (online) error
            pool[i]->classify(input)==input->getLabel()?correct[i]++:wrong[i]++;

            errors[i] = wrong[i]/(correct[i]+wrong[i]);
            asleep[i]--;

            //keep track of the best WeakClassifier
            if(asleep[i] < 0 && errors[i] < error)
            {
                error = errors[i];
                selected = i;
            }
        }

        //handle unconsistencies of the trained WeakClassifiers
        if(selected>0 && !asleep[selected] && (error < 1e-6 || error > 0.5))
        {
            //debug
            std::cout << "this selector has best error " << this->error << std::endl;
        }

        //if needed, substitute bad performing Weak Classifiers
        for(unsigned int i = 0; i < pool.size(); i++)
        {
            if(asleep[i] < 0 && function_space->size() && substitute(errors[i]) && i != selected)
            {
                delete pool[i];
                pool[i] = randomSample();
                correct[i] = 1.0;
                wrong[i] = 1.0;
                asleep[i] = sleep_time;
            }
        }
    }
}



bool SelectorClassifier::substitute(const double &e) const
{
    return (e > rel_thresh*error || e > abs_thresh);
}



WeakClassifier *SelectorClassifier::randomSample()
{
    int idx = (int) floor(yarp::math::Rand::scalar(0,function_space->size()));

    //avoid index unconsistency
    if(idx==function_space->size()) idx--;

    WeakClassifier *rnd_h = function_space->at(idx);

    //Erase the sampled element from the function_space
    function_space->erase(function_space->begin() + idx);

    return rnd_h;
}




std::string  SelectorClassifier::toString() const
{
    if(ready)
    {
        std::stringstream strstr;

        strstr << "(type " << type << ") ";
        strstr << "(selected " << selected << ") ";

        strstr << "(abs_thresh " << abs_thresh <<") ";
        strstr << "(rel_thresh " << rel_thresh <<") ";
        strstr << "(sleep_time " << sleep_time <<") ";

        strstr << "(pool (";
        for(unsigned int i=0; i<pool.size(); i++)
        {
            strstr << "(";
            strstr << "(correct " << correct[i] << ") ";
            strstr << "(wrong " << wrong[i] << ") ";
            strstr << "(asleep " << asleep[i] << ") ";
            strstr << "(weak_classifier (" << pool[i]->toString() << "))";
            strstr << ") ";
        }
        strstr << "))";

        return strstr.str().c_str();
    }
    else
        return "";
}



void   SelectorClassifier::fromString(const std::string &str)
{
    clear();

    yarp::os::Bottle bLoader(str.c_str());

    type=bLoader.find("type").asString().c_str();
    selected=bLoader.find("selected").asInt();
    abs_thresh=bLoader.find("abs_thresh").asDouble();
    rel_thresh=bLoader.find("rel_thresh").asDouble();
    sleep_time=bLoader.find("sleep_time").asInt();

    yarp::os::Bottle *bPool=bLoader.find("pool").asList();

    pool_size=bPool->size();
    for(int i=0; i<pool_size; i++)
    {
        yarp::os::Bottle *bElement=bPool->get(i).asList();
        correct.push_back(bElement->find("correct").asDouble());
        wrong.push_back(bElement->find("wrong").asDouble());
        asleep.push_back(bElement->find("asleep").asInt());

        yarp::os::Bottle *bWL=bElement->find("weak_classifier").asList();
        pool.push_back(ClassifierFactory::instance().create(bWL->find("type").asString().c_str()));
        pool.back()->fromString(bWL->toString().c_str());
    }
}




