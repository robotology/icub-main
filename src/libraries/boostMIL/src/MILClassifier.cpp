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

#include <cassert>
#include <stdexcept>
#include <sstream>
#include <fstream>

#include <stdio.h>
#include <math.h>

#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

#include <iCub/boostMIL/MILClassifier.h>
#include <iCub/boostMIL/ClassifierInput.h>




#define EPSILON 1e-06


using namespace iCub::boostMIL;

//Constructors
MILClassifier::MILClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, const double *_center)
    :WeakClassifier(_type,_resource)
{
    initResource();

    if(_center != NULL)
        setCenter(_center);
}

MILClassifier::MILClassifier(const MILClassifier &mil, const double *_center)
    :WeakClassifier(mil)
{
    initResource();

    if(_center==NULL)
        _center = mil.center.data();

    setCenter(_center);

    radius = mil.radius;
    error = mil.error;

}





//Private Methods
void                            MILClassifier::initResource         ()
{
    clear();

    if(resource.check(type.c_str()))
    {
        yarp::os::Bottle data = resource.findGroup(type.c_str());

        max_cache_size = data.check("max_cache_size",yarp::os::Value(50)).asInt();

        if(data.check("feature_size"))
        {
            feature_size = data.find("feature_size").asInt();
            center.resize((size_t)feature_size);
        }
    }
}


double                          MILClassifier::distanceFromCenter   (const double *vec) const
{
    double dist = 0.0;
    for(int i = 0; i < feature_size; i++)
        dist += (center[i]-vec[i])*(center[i]-vec[i]);
    return sqrt(dist);
}

void                            MILClassifier::setCenter            (const double *_center)
{
    //clear everything
    radius = 0;

    posweight = 0.0;
    negweight = 0.0;
    cache.clear();

    if(center.size() != feature_size)
    {
        center.zero();
        center.clear();
        center.resize(feature_size);
    }

    for(int i = 0; i < feature_size; i++)
        center[i]=_center[i];
}

void                            MILClassifier::insertBag            (const double &dist, const int &label, const double &weight)
{
    Bag bag(dist,label,weight);

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
    if(cache.size() == max_cache_size)
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



double                          MILClassifier::bestRadius           ()
{
    double best_radius = 0.0;

    // Error is not normalized in this case
    double fr=posweight, fa=0.0;
    double minerr=fr+fa;
    double newthreshold=0.0;
    
    std::deque<Bag>::iterator itr = cache.begin();
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




//Public  Methods
void                            MILClassifier::clear                ()
{
    radius          = 0;
    feature_size    = 0;
    max_cache_size  = 0;
    posweight       = 0.0;
    negweight       = 0.0;

    cache.clear();
    center.zero();
    center.clear();
}



bool                            MILClassifier::isReady              ()
{
    ready = (feature_size != 0 && feature_size == center.size());
    return ready;
}


int                             MILClassifier::classify             (const Inputs *input, Output *output) const
{
    if(!ready)
        return 0;

    const yarp::sig::Vector *bag = input->getInput(type);

    if(!bag)
        return 0;

    if(radius<0)
        return +1;


    if(bag->size() % feature_size != 0 || feature_size == 0)
    {
        std::ostringstream buffer;
        buffer << "MILClassifier of type: '" << type << "'" << std::endl;
        buffer << "Wrong input size: " << bag->size() << " is not divisible by " << feature_size;
        throw std::runtime_error(buffer.str());
    }

    int label=-1;

    for(size_t i = 0; i < bag->size(); i += feature_size)
    {
        //compute the distance between the two vectors
        double dist=distanceFromCenter(bag->data()+i);
        if(dist<radius)
        {
            label=+1;
                
            yarp::sig::Vector p=input->getPosition(type,i/feature_size);           

            if(output!=NULL)
                output->pushPosition(input->getPosition(type,i/feature_size));
        }
    }

    return label;
}




void  MILClassifier::toStream(std::ofstream &fout) const
{
    fout.write((char*)&type,sizeof(int));
    fout.write((char*)&feature_size,sizeof(int));
    fout.write((char*)&radius,sizeof(double));

    fout.write((char*)&posweight,sizeof(double));
    fout.write((char*)&negweight,sizeof(double));

    for(int i=0; i<feature_size; i++)
        fout.write((char*)&center[i],sizeof(double));

    size_t cache_size=cache.size();
    fout.write((char*)&cache_size,sizeof(size_t));

    for(size_t i=0; i<cache_size; i++)
        fout.write((char*)&cache[i],sizeof(Bag));
}


void   MILClassifier::fromStream(std::ifstream &fin)
{
    clear();

    fin.read((char*)&type,sizeof(int));
    fin.read((char*)&feature_size,sizeof(int));
    fin.read((char*)&radius,sizeof(double));

    fin.read((char*)&posweight,sizeof(double));
    fin.read((char*)&negweight,sizeof(double));

    center.resize(feature_size);
    for(int i=0; i<feature_size; i++)
        fin.read((char*)&center[i],sizeof(double));

    size_t cache_size;
    fin.read((char*)&cache_size,sizeof(size_t));

    cache.resize(cache_size);
    for(size_t i=0; i<cache_size; i++)
        fin.read((char*)&cache[i],sizeof(Bag));
}





std::string  MILClassifier::toString() const
{
    std::stringstream strstr;

    strstr << "(type " << type << ") ";
    strstr << "(feature_size " << feature_size << ") ";
    strstr << "(radius " << radius << ") ";

    strstr << "(posweight " << posweight << ") ";
    strstr << "(negweight " << negweight << ") ";

    strstr << "(center (";
    for(size_t i=0; i<center.size(); i++)
        strstr << center[i] << " ";
    strstr << ")) ";

    strstr << "(cache (";
    for(unsigned int i=0; i<cache.size(); i++)
        strstr << "(" << cache[i].toString() << ") ";
    strstr << ")) ";

    return strstr.str().c_str();
}


void   MILClassifier::fromString(const std::string &str)
{
    clear();

    yarp::os::Bottle bLoader(str.c_str());

    type=bLoader.find("type").asString().c_str();
    feature_size=bLoader.find("feature_size").asInt();
    radius=bLoader.find("radius").asDouble();

    posweight=bLoader.find("posweight").asDouble();
    negweight=bLoader.find("negweight").asDouble();

    yarp::os::Bottle *bCenter=bLoader.find("center").asList();
    for(int i=0; i<bCenter->size(); i++)
        center.push_back(bCenter->get(i).asDouble());

    yarp::os::Bottle *bCache=bLoader.find("cache").asList();
    for(int i=0; i<bCache->size(); i++)
    {
        Bag b;
        b.fromString(bCache->get(i).asList()->toString().c_str());
        cache.push_back(b);
    }
}




void                            MILClassifier::update               (const Inputs *input, double &weight)
{
    const yarp::sig::Vector *bag = input->getInput(type);

    if(bag)
    {
        //check that the vector dimension is divisible by the feature_size
        if(bag->size()%feature_size != 0 || feature_size == 0)
        {
            std::ostringstream buffer;
            buffer << "MILClassifier of type: '" << type << "'" << std::endl;
            buffer << "Wrong input size: " << bag->size() << " is not divisible by " << feature_size;
            throw std::runtime_error(buffer.str());
        }

        //find the element in the bag which is nearest to the weak classifier center
        double min_dist = -1.0;
        for(size_t i = 0; i < bag->size(); i+=feature_size)
        {
            double dist = distanceFromCenter(bag->data()+i);
            if(dist < min_dist || min_dist < 0.0)
                min_dist = dist;
        }

        //insert the element in the right position of the cache
        insertBag(min_dist,input->getLabel(),weight);

        //find the best radius and update the error
        radius = bestRadius();
    }
}



double                          MILClassifier::error_rate           (const std::list<const Inputs *> &dataset, std::list<double> &weights)
{
    //find the best radius, given this dataset
    radius = 0;
    
    posweight = 0.0;
    negweight = 0.0;
    cache.clear();


    max_cache_size = dataset.size();

    std::list<const Inputs *>::const_iterator   d_itr = dataset.begin();
    std::list<double>        ::iterator         w_itr = weights.begin();

    while(d_itr != dataset.end())
    {
        //find the best element in the bag
        const yarp::sig::Vector *bag    = (*d_itr)->getInput(type.c_str());
        const int label                 = (*d_itr)->getLabel();

        //check that the vector dimension is divisible by the feature_size
        if(bag->size()%feature_size != 0 || feature_size == 0)
        {
            std::ostringstream buffer;
            buffer << "MILClassifier of type: '" << type << "'" << std::endl;
            buffer << "Wrong input size: " << bag->size() << " is not divisible by " << feature_size;
            throw std::runtime_error(buffer.str());
        }

        //find the element in the bag which is nearest to the weak classifier center
        double min_dist = -1.0;
        for(size_t i = 0; i < bag->size(); i+=feature_size)
        {
            double dist = distanceFromCenter(bag->data()+i);
            if(dist < min_dist || min_dist < 0.0)
                min_dist = dist;
        }

        //insert the element in the right position of the cache
        insertBag(min_dist,label,*w_itr);
    }

    //find the best radius and update the error
    radius = bestRadius();

    return error;
}




std::list<WeakClassifier*>      *MILClassifier::pack                (const Inputs *input) const
{
    std::list<WeakClassifier*> *list = NULL;

    if(input!=NULL)
    {
        list=new std::list<WeakClassifier*>;

        const yarp::sig::Vector *bag = input->getInput(type);
        //if the Correesponding input is found, pack it
        if(bag)
        {
            //check that the vector dimension is divisible by the feature_size
            if(bag->size()%feature_size != 0 || feature_size == 0)
            {
                std::ostringstream buffer;
                buffer << "MILClassifier of type: '" << type << "'" << std::endl;
                buffer << "Wrong input size: " << bag->size() << " is not divisible by " << feature_size;
                throw std::runtime_error(buffer.str());
            }
            for(size_t i= 0; i < bag->size(); i += feature_size)
                list->push_back(new MILClassifier(type,resource,bag->data()+i));
        }
    }
    return list;
}





//----------------- HOG Classifier ------------------//

//Constructors
HOGClassifier::HOGClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, const double *_center)
    :MILClassifier(_type,_resource,_center)
{
    initResource();

    if(_center != NULL)
        setCenter(_center);
}


HOGClassifier::HOGClassifier(const HOGClassifier &hog, const double *_center)
    :MILClassifier(hog)
{
    initResource();

    if(_center==NULL)
        _center = hog.center.data();

    setCenter(_center);

    radius = hog.radius;
    error = hog.error;

}


void                            HOGClassifier::initResource         ()
{
    clear();

    if(resource.check(type.c_str()))
    {
        yarp::os::Bottle data = resource.findGroup(type.c_str());

        max_cache_size = data.check("max_cache_size",yarp::os::Value(50)).asInt();

        if(data.check("n_levels") && data.check("hog_bins"))
        {
            hog_bins=data.find("hog_bins").asInt();
            n_levels=data.find("n_levels").asInt();

            feature_size=n_levels*hog_bins;

            center.resize(feature_size);
        }
    }
}

double                          HOGClassifier::distanceFromCenter   (const double *vec) const
{
    double dist=0.0;
    double dist2=0.0;

    for(int i=0; i<feature_size; i++)
    {
        dist+=vec[i]*center[i];
        dist2+=vec[i]*vec[i]*center[i]*center[i];
    }


    dist=1-dist/sqrt(dist2);

    return dist;
}


//----------------- Corr Classifier ------------------//

//Constructors
CorrClassifier::CorrClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, const double *_center)
    :MILClassifier(_type,_resource,_center)
{
    initResource();

    if(_center != NULL)
        setCenter(_center);
}


CorrClassifier::CorrClassifier(const CorrClassifier &corr, const double *_center)
    :MILClassifier(corr)
{
    initResource();

    if(_center==NULL)
        _center = corr.center.data();

    setCenter(_center);

    radius = corr.radius;
    error = corr.error;
}


void                            CorrClassifier::initResource         ()
{
    clear();

    if(resource.check(type.c_str()))
    {
        yarp::os::Bottle data = resource.findGroup(type.c_str());

        max_cache_size = data.check("max_cache_size",yarp::os::Value(50)).asInt();

        if(data.check("init_win_size"))
        {
            win_init_size=data.find("win_init_size").asInt();
            win_increment=data.find("win_increment").asInt();
            n_levels=data.find("n_levels").asInt();
            int nChan=data.check("color")?3:1;

            feature_size=nChan*win_init_size;

            center.resize(feature_size);
        }
    }
}

double                          CorrClassifier::distanceFromCenter   (const double *vec) const
{
    double dist=0.0;

    for(int i=0; i<feature_size; i++)
        dist+=vec[i]*center[i];

    dist=1.0-sqrt(dist);

    return dist;
}


//----------------- Dictionary Classifier ------------------//

//Constructors
DictionaryClassifier::DictionaryClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, const double *_center)
    :MILClassifier(_type,_resource,NULL)
{
    initResource();

    if(_center != NULL)
        setCenter(_center);
}

DictionaryClassifier::DictionaryClassifier(const std::string &_type, yarp::os::ResourceFinder &_resource, int _index, const double *_center)
    :MILClassifier(_type,_resource,NULL),index(_index)
{
    initResource();

    if(_center != NULL)
        setCenter(_center);
}


DictionaryClassifier::DictionaryClassifier(const DictionaryClassifier &dict, const double *_center)
    :MILClassifier(dict)
{
    initResource();

    if(_center==NULL)
        _center = dict.center.data();

    setCenter(_center);

    radius = dict.radius;
    error = dict.error;

    index = dict.index;
}


double                          DictionaryClassifier::distanceFromCenter   (const double *vec) const
{
    double val=vec[index];
    if(fabsize)
        val=fabs(val);

    return fabs(center[0]-val);
}



std::list<WeakClassifier*>      *DictionaryClassifier::pack                (const Inputs *input) const
{
    std::list<WeakClassifier*> *list = NULL;

    if(input!=NULL)
    {
        list=new std::list<WeakClassifier*>;

        const yarp::sig::Vector *bag = input->getInput(type);
        //if the Correesponding input is found, pack it
        if(bag)
        {
            //check that the vector dimension is divisible by the feature_size
            if(bag->size()%feature_size != 0 || feature_size == 0)
            {
                std::ostringstream buffer;
                buffer << "DictionaryClassifier of type: '" << type << "'" << std::endl;
                buffer << "Wrong input size: " << bag->size() << " is not divisible by " << feature_size;
                throw std::runtime_error(buffer.str());
            }
            for(size_t f=0; f<bag->size(); f+=feature_size)
                for(size_t i=0; i<(size_t)feature_size; i++)
                    list->push_back(new DictionaryClassifier(type,resource,i,bag->data()+f));
        }
    }
    return list;
}

void                            DictionaryClassifier::initResource                                ()
{
    if(resource.check(type.c_str()))
    {
        yarp::os::Bottle data = resource.findGroup(type.c_str());
        fabsize=data.check("absolute",yarp::os::Value("true")).asString()=="true";
    }
}



void                            DictionaryClassifier::setCenter             (const double *_center)
{
    //clear everything
    radius = 0;

    posweight = 0.0;
    negweight = 0.0;
    cache.clear();

    if(center.size() != 1)
    {
        center.zero();
        center.clear();
        center.resize(1);
    }

    center[0]=_center[0];
}


bool                            DictionaryClassifier::isReady()
{
    ready = (feature_size != 0 && center.size()==1);
    return ready;
}


