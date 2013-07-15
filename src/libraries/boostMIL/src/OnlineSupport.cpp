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

#include <fstream>

#include <algorithm>

#include <yarp/os/Bottle.h>

#include <iCub/boostMIL/OnlineSupport.h>
#include <iCub/boostMIL/AvgClassifier.h>
#include <iCub/boostMIL/ClassifierFactory.h>


using namespace iCub::boostMIL;


OnlineSupport::OnlineSupport(yarp::os::ResourceFinder &_resource)
    :StrongClassifier("online_boost",_resource)
{
    initResource();
}

OnlineSupport::OnlineSupport(const std::string &_type, yarp::os::ResourceFinder &_resource)
    :StrongClassifier(_type,_resource)
{
    initResource();
}

void OnlineSupport::initResource()
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
            correct.resize(weak_classifiers_size,0);

            std::string avg_type = data.check("avg_type",yarp::os::Value("avg")).asString().c_str();

            for(unsigned int i = 0; i < weak_classifiers_size; i++)
                weak_classifiers.push_back(new AvgClassifier(avg_type,resource,&function_space));
        }
    }
}



bool OnlineSupport::isReady()
{
    //OnlineSupport is initialized if all the WeakClassifiers are initialized
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



void OnlineSupport::train(const std::list<Inputs*> &training_set)
{
    int cnt=0;
    for(std::list<Inputs*>::const_iterator itr = training_set.begin(); itr != training_set.end(); itr++)
    {
        train(*itr);
        if(verbose) fprintf(stdout,"[BoostMIL] trained sample n. %d\r",++cnt);
    }
}

void OnlineSupport::train(const Inputs* input, double weight)
{
    update(input,weight);
}


void OnlineSupport::initialize(const Inputs *initializer)
{
    ClassifierFactory::instance().pack(initializer,function_space);
}

void OnlineSupport::initialize(const std::list<Inputs*> &initializer)
{
    for(std::list<Inputs*>::const_iterator itr=initializer.begin(); itr!=initializer.end(); itr++)
        ClassifierFactory::instance().pack((*itr),function_space);

    //shuffle the function_space
    random_shuffle(function_space.begin(),function_space.end());
}



void   OnlineSupport::fromStream(std::ifstream &fin)
{
    clear();

    int wc_size;
    fin.read((char*)&wc_size,sizeof(int));

    weak_classifiers_size=wc_size;

    alphas.resize(wc_size);
    correct.resize(wc_size);
    weak_classifiers.resize(wc_size);

    for(int i=0; i<wc_size; i++)
    {
        fin.read((char*)&alphas[i],sizeof(double));
        fin.read((char*)&correct[i],sizeof(int));

        int size_of_type;
        fin.read((char*)&size_of_type,sizeof(int));
        char *wc_type=new char[size_of_type+1];
        fin.read((char*)wc_type,size_of_type*sizeof(char));

        wc_type[size_of_type]='\0';
        weak_classifiers[i]=new AvgClassifier(wc_type,resource,&function_space);
        delete [] wc_type;

        weak_classifiers[i]->fromStream(fin);
    }

    isReady();
}


void   OnlineSupport::toStream(std::ofstream &fout) const
{
    //write the size of the list of weak classifiers
    int wc_size=this->weak_classifiers.size();
    fout.write((char*)&wc_size,sizeof(int));

    for(int i=0; i<wc_size; i++)
    {
        fout.write((char*)&alphas[i],sizeof(double));
        fout.write((char*)&correct[i],sizeof(int));

        int size_of_type=weak_classifiers[i]->getType().size();
        fout.write((char*)&size_of_type,sizeof(int));

        char *wc_type=new char[size_of_type+1];
        sprintf(wc_type,"%s",weak_classifiers[i]->getType().c_str());
        fout.write((char*)wc_type,size_of_type*sizeof(char));

        delete [] wc_type;
        weak_classifiers[i]->toStream(fout);
    }
}




void   OnlineSupport::fromString(const std::string &str)
{
    clear();

    yarp::os::Bottle bLoader;
    bLoader.fromString(str.c_str());

    weak_classifiers_size=bLoader.size();

    for(int i=0; i<bLoader.size(); i++)
    {
        alphas.push_back(bLoader.get(i).asList()->find("alpha").asDouble());
        correct.push_back((int)bLoader.get(i).asList()->find("correct").asDouble());

        yarp::os::Bottle *bSelector=bLoader.get(i).asList()->find("weak_classifier").asList();

        weak_classifiers.push_back(new AvgClassifier(bSelector->find("type").asString().c_str(),resource,&function_space));
        weak_classifiers.back()->fromString(bSelector->toString().c_str());
    }

    isReady();
}




void OnlineSupport::clearFunctionSpace()
{
    while(function_space.size())
    {
        delete function_space.front();
        function_space.pop_front();
    }
}

void OnlineSupport::update(const Inputs *input, double &weight)
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


            if(weak_classifiers[i]->classify(input)>0)
            {
                if(input->getLabel()>0)
                {
                    correct[i]++;
                    double alpha = 1/(1+exp((double)-correct[i]) );
                    weight *= 1.0-alpha;
                }
                else
                {
                    correct[i]--;
                    weight *= 1.0;
                }
            }
            else
            {
                if(input->getLabel()>0)
                    weight *= 1.1;
                else
                    weight *= 1.0;
            }

            alphas[i]=1/(1+exp((double)-correct[i]) );


            ////if the WeakClassifiers abstains from classifing, do not update the weight
            //if(weak_classifiers[i]->classify(input) > 0)
            //{
            //    if(weak_classifiers[i]->cl]\assify(input) == input->getLabel())
            //    {
            //        weight *= 0.5/alphas[i];
            //    }
            //    else
            //    {
            //        weight *= 0.5/alphas[i]);
            //    }
            //}

            
            //update alpha
            //alphas[i] = 0.5*log((1-weak_classifiers[i]->getError())/weak_classifiers[i]->getError());

        }
    }
}



void OnlineSupport::clear()
{
    StrongClassifier::clear();

    weak_classifiers_size = 0;
}


void OnlineSupport::randomizeFuncSpace()
{
    random_shuffle(function_space.begin(),function_space.end());
}





double OnlineSupport::margin(const Inputs *input, Output *output) const
{
    double sum=0.0;
    if(ready)
    {
        for(size_t i=0; i<weak_classifiers.size(); i++)
        {
            double w=alphas[i]>0.0?alphas[i]:0.0;
            sum+=w*(0.5*(weak_classifiers[i]->classify(input,output)+1) ); //0 1 instead of -1 1
        }
    }

    return sum;
}


