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

#include <iCub/boostMIL/AvgClassifier.h>
#include <iCub/boostMIL/ClassifierFactory.h>



#define EPSILON 1e-06



using namespace iCub::boostMIL;


AvgClassifier::AvgClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, std::deque<WeakClassifier*> *_function_space)
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



void AvgClassifier::initResource()
{
    clear();

    if(resource.check(type.c_str()))
    {
        yarp::os::Bottle data = resource.findGroup(type.c_str());

        pool_size = data.check("pool_size",yarp::os::Value(1)).asInt();

        if(data.check("threshold"))
        {
            threshold_fixed=true;
            threshold=data.find("threshold").asDouble();
        }
        else
        {
            threshold_fixed=false;
            threshold=1.0;
        }

    }
    else
    {
        pool_size = 1;
    }

    //initialize the vectors
    for(int i = 0; i < pool_size; i++)
    {
        correct.push_back(1.0);
        wrong.push_back(1.0);

        alpha.push_back(1.0);
    }
}


void AvgClassifier::clear()
{
    ready=false;

    if(!threshold_fixed)
        threshold=1.0;

    posweight       = 0.0;
    negweight       = 0.0;

    pool_size = 0;
    while(pool.size())
    {
        delete pool.back();
        pool.pop_back();
    }

    correct.clear();
    wrong.clear();
    alpha.clear();
}



double AvgClassifier::margin(const Inputs *input, Output *output) const
{
    double sum=0.0;
    if(ready)
    {
        for(int i=0; i<pool_size; i++)
        {
            double w=alpha[i]>0.0?alpha[i]:0.0;
            sum+=w*(0.5*(pool[i]->classify(input,output)+1) ); //0 1 instead of -1 1
        }
    }

    return sum;
}


int AvgClassifier::classify(const Inputs *input, Output *output) const
{
    if(this->margin(input,output)>threshold)
        return 1;
    else
        return -1;
}



bool AvgClassifier::isReady()
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


/*

void AvgClassifier::update(const Inputs *input, double &weight)
{
    if(isReady())
    {
        error = 1.0;
        std::vector<double> errors(pool.size());

        double response=0.0;
        for(unsigned int i = 0; i < pool.size(); i++)
        {
            //first update the single WL with the new sample
            pool[i]->update(input,weight);

            //update the alpha
            if(pool[i]->classify(input)>0)
                if(input->getLabel()>0)
                    correct[i]++;
                else
                    wrong[i]++;

            alpha[i] = 1/(1+exp(-(correct[i]-wrong[i])) );

            if(pool[i]->classify(input)>0)
                response+=alpha[i];
        }

        this->insertBag(response,input->getLabel(),weight);
        if(!threshold_fixed)
            threshold=this->bestRadius();

        //get the value 
        //update the sample error
        //if(pool[i]->classify(input)==1)
    }


    //update the threshold of the classifier

}

*/

void AvgClassifier::update(const Inputs *input, double &weight)
{
    if(isReady())
    {
        error = 1.0;
        std::vector<double> errors(pool.size());

        double response=0.0;
        for(unsigned int i = 0; i < pool.size(); i++)
        {
            //first update the single WL with the new sample
            pool[i]->update(input,weight);

            //update the alpha
            if(pool[i]->classify(input)>0)
                if(input->getLabel()>0)
                    correct[i]++;
                else
                    wrong[i]++;

            //alpha[i] = 1/(1+exp(-(correct[i]-wrong[i])) );
            alpha[i]=log(sqrt((double)correct[i]/(wrong[i]+0.000001)));
            if(alpha[i]<0.0)
                alpha[i]=0.0;


            if(pool[i]->classify(input)>0)
                response+=alpha[i];
        }

        this->insertBag(response,input->getLabel(),weight);
        if(!threshold_fixed)
            threshold=this->bestRadius();

        //get the value 
        //update the sample error
        //if(pool[i]->classify(input)==1)
    }


    //update the threshold of the classifier

}

//
//
//bool AvgClassifier::substitute(const double &e) const
//{
//    return (e > rel_thresh*error || e > abs_thresh);
//}



WeakClassifier *AvgClassifier::randomSample()
{
    int idx = (int) floor(yarp::math::Rand::scalar(0,function_space->size()));

    //avoid index unconsistency
    if(idx==function_space->size()) idx--;

    WeakClassifier *rnd_h = function_space->at(idx);

    //Erase the sampled element from the function_space
    function_space->erase(function_space->begin() + idx);

    return rnd_h;
}




void  AvgClassifier::toStream(std::ofstream &fout) const
{
    if(ready)
    {
        fout.write((char*)&posweight,sizeof(double));
        fout.write((char*)&negweight,sizeof(double));

        fout.write((char*)&threshold,sizeof(double));
        fout.write((char*)&pool_size,sizeof(int));

        for(int i=0; i<pool_size; i++)
        {
            fout.write((char*)&correct[i],sizeof(double));
            fout.write((char*)&wrong[i],sizeof(double));

            int size_of_type=pool[i]->getType().size();
            fout.write((char*)&size_of_type,sizeof(int));

            char *wc_type=new char[size_of_type+1];
            sprintf(wc_type,"%s",pool[i]->getType().c_str());
            fout.write((char*)wc_type,size_of_type*sizeof(char));

            delete [] wc_type;

            pool[i]->toStream(fout);
        }
    }
}

void   AvgClassifier::fromStream(std::ifstream &fin)
{
    clear();

    fin.read((char*)&posweight,sizeof(double));
    fin.read((char*)&negweight,sizeof(double));

    fin.read((char*)&threshold,sizeof(double));
    fin.read((char*)&pool_size,sizeof(int));


    correct.resize(pool_size);
    wrong.resize(pool_size);
    alpha.resize(pool_size);
    pool.resize(pool_size);

    for(int i=0; i<pool_size; i++)
    {
        fin.read((char*)&correct[i],sizeof(double));
        fin.read((char*)&wrong[i],sizeof(double));
        alpha[i]=1.0/(1+exp((double)-(correct[i]-wrong[i]) ) );

        int size_of_type;
        fin.read((char*)&size_of_type,sizeof(int));
        char *wc_type=new char[size_of_type+1];
        fin.read((char*)wc_type,size_of_type*sizeof(char));

        wc_type[size_of_type]='\0';

        pool[i]=ClassifierFactory::instance().create(wc_type);
        delete [] wc_type;

        pool[i]->fromStream(fin);
    }

    isReady();
}



std::string  AvgClassifier::toString() const
{
    if(ready)
    {
        std::stringstream strstr;

        strstr << "(type " << type << ") ";

        strstr << "(posweight " << posweight << ") ";
        strstr << "(negweight " << negweight << ") ";

        strstr << "(threshold " << threshold << ") ";
        strstr << "(pool (";
        for(unsigned int i=0; i<pool.size(); i++)
        {
            strstr << "(";
            strstr << "(correct " << correct[i] << ") ";
            strstr << "(wrong " << wrong[i] << ") ";
            strstr << "(weak_classifier (" << pool[i]->toString() << "))";
            strstr << ") ";
        }
        strstr << "))";

        return strstr.str().c_str();
    }
    else
        return "";
}



void   AvgClassifier::fromString(const std::string &str)
{
    clear();

    yarp::os::Bottle bLoader(str.c_str());

    type=bLoader.find("type").asString().c_str();

    posweight=bLoader.find("posweight").asDouble();
    negweight=bLoader.find("negweight").asDouble();

    threshold=bLoader.find("threshold").asDouble();
    yarp::os::Bottle *bPool=bLoader.find("pool").asList();


    pool_size=bPool->size();
    for(int i=0; i<pool_size; i++)
    {
        yarp::os::Bottle *bElement=bPool->get(i).asList();
        correct.push_back(bElement->find("correct").asDouble());
        wrong.push_back(bElement->find("wrong").asDouble());

        yarp::os::Bottle *bWL=bElement->find("weak_classifier").asList();
        pool.push_back(ClassifierFactory::instance().create(bWL->find("type").asString().c_str()));
        pool.back()->fromString(bWL->toString().c_str());
    }

    isReady();
}






void                            AvgClassifier::insertBag            (const double &dist, const int &label, const double &weight)
{
    AvgBag bag(dist,label,weight);

    //find the right position in the cache deque
    double  min_weight = -1.0;
    double  min_dist;
    int     min_idx;
    int     idx = 0;
    for(unsigned int i = 0; i < cache.size(); i++)
    {
        if(min_weight < cache[i].weight || min_weight == -1.0)
        {
            min_weight = cache[i].weight;
            min_dist = cache[i].dist;
            min_idx = i;
        }
        if(bag.dist > cache[i].dist) idx = i+1;
    }

    //if the list is too big, erase the lightest element
    if(cache.size() == 100)
    {
        //if the new bag has the smallest weight don't bother inserting it in the cache
        if(bag.weight <= min_weight)
            return;

        if(bag.dist > cache[min_idx].dist)
            idx--;

        //remove the weight from the weight counters
        if(cache[min_idx].label == +1)
            posweight -= cache[min_idx].weight;
        else
            negweight -= cache[min_idx].weight;

        cache.erase(cache.begin()+min_idx);
    }

    //insert the new element
    cache.insert(cache.begin()+idx,bag);

    if(bag.label == +1)
        posweight += bag.weight;
    else
        negweight += bag.weight;
}



double                          AvgClassifier::bestRadius           ()
{
    double best_radius = 0.0;

    // Error is not normalized in this case
    double fr=posweight, fa=0.0;
    double minerr=fr+fa;
    double newthreshold=0.0;
    
    std::deque<AvgBag>::iterator itr = cache.begin();
    while(itr != cache.end())
    {
        newthreshold = itr->dist;
        if(fr+fa<minerr-EPSILON)
        {
            best_radius = newthreshold;
            minerr=fr+fa;
        }

        do
        {
            if(itr->label == +1)
                fr -= itr->weight;
            else
                fa += itr->weight;
        }
        while(++itr != cache.end() && fabs(itr->dist - newthreshold)<EPSILON);
    }

    if(minerr < 0) minerr =0.0;

    // Check if everything should be accepted
    if (fa<minerr-EPSILON)
    {
        best_radius=-1.0;
        minerr=fa;
    }


    // Check that the fr is zero
    if (fabs(fr)/(negweight+posweight)> 0.1)
    {
        std::ostringstream buffer;
        buffer << "MILClassifier of type: '" << type << "'" << std::endl;
        buffer << "Something amiss with the online ball learner: fr " << fr;
        throw std::runtime_error(buffer.str());
    }

    // Minerr is the weight of the misclassified examples. The denominator is total weight.
    error = minerr/(negweight+posweight);

    return best_radius;
}




