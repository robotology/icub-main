// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick, Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iCub/vis/GroupSalience.h>

#include <stdio.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace iCub::vis;


bool GroupSalience::open(yarp::os::Searchable& config) {

    bool ok = Salience::open(config);
    if (!ok)
        return false;

    Bottle lst = config.findGroup("subfilter",
                                  "a list of filters").tail();

    // create the filters
    for (int i=0; i<lst.size(); i++) {
        ConstString name = lst.get(i).toString();
        printf("Creating subfilter %s\n", name.c_str());
        Salience *filter = NULL;
        if (config.check(name)) {
            // we have a sub-entry for configuration
            Bottle& subConfig = config.findGroup(name);
            name = subConfig.check("filter",Value(name)).toString();
            filter = SalienceFactories::getPool().get(name);
            if (filter!=NULL) {
                bool ok = filter->open(subConfig);
                if (!ok) {
                    delete filter;
                    filter = NULL;
                    clear();
                    return false;
                }
            }         
        } else {
            // no sub-entry for configuration
            filter = SalienceFactories::getPool().get(name);
            if (filter!=NULL) {
                bool ok = filter->open(config);
                if (!ok) {
                    delete filter;
                    filter = NULL;
                    clear();
                    return false;
                }
            }
        }

        if (filter!=NULL) {
            add(filter);
        }
        
    }

    return true;
}

bool GroupSalience::configure(Searchable& config){
    return true;
}

bool GroupSalience::respond(const Bottle &command,Bottle &reply){
    bool rec = false;
    bool ok = false;

    // should implement here the general ISalienceControls
    // TODO
    ok = true;

    // pass command/reply on to child filters
    switch (command.get(0).asVocab()) {
    case SALIENCE_VOCAB_NAME:
        rec = true;
        {
            string fName;
            string subName;
            int pos;
            Bottle subCommand;
            for (unsigned int i = 0; i < group.size(); i++){
                subCommand.clear();
                fName = command.get(1).asString();
                pos = fName.find_first_of(group[i]->getFilterName());
                if (pos == 0){ // corresponding child filter found
                    pos = fName.find_first_of('.');
                    if (pos  > -1){ // there is a subfilter name
                        subName = fName.substr(pos + 1, fName.size()-1);
                        subCommand.add(command.get(0)); // add modified filter name
                        subCommand.add(Value(subName.c_str()));
                    }
                    for (int ii = 2; ii < command.size(); ii++) // copy remaining values
                        subCommand.add(command.get(ii));
                    ok = group[i]->respond(subCommand, reply);
                }
            }
        }
        break;
    default:
        break;
    }
    return ok;
}

void GroupSalience::apply(ImageOf<PixelRgb>& src, 
                          ImageOf<PixelRgb>& dest,
                          ImageOf<PixelFloat>& sal) {
    dest.resize(src);
    sal.resize(src);
    dest.zero();
    sal.zero();
    double weight_i;
    for (unsigned int i=0; i<group.size(); i++) {
        ImageOf<PixelFloat>& sal_i = salienceMap[i];
        ImageOf<PixelRgb>& dest_i = salienceView[i];
        sal_i.resize(src);
        dest_i.resize(src);
        group[i]->apply(src, dest_i, sal_i);
        weight_i = group[i]->getWeight() / weightSum;
        IMGFOR(sal_i,x,y) {
            sal(x,y) += (float)(weight_i * sal_i(x,y));
            PixelRgb& pix_i = dest_i(x,y);
            PixelRgb& pix = dest(x,y);
            pix.r += (unsigned char)(weight_i * (int)pix_i.r);
            pix.g += (unsigned char)(weight_i * (int)pix_i.g);
            pix.b += (unsigned char)(weight_i * (int)pix_i.b);
        }
        
    }
}

void GroupSalience::add(Salience *filter){
    group.push_back(filter);
    ImageOf<PixelFloat> blankFloat;
    ImageOf<PixelRgb> blankRgb;
    salienceMap.push_back(blankFloat);
    salienceView.push_back(blankRgb);
    weightSum += filter->getWeight();
}

bool GroupSalience::setChildFilterName(int j, string s){
    if (j < (int)group.size() && j >=0){
        group[j]->setFilterName(s);
        return true;
    }
    return false;
}

string GroupSalience::getChildFilterName(int j){
    if (j < (int)group.size() && j >=0)
        return group[j]->getFilterName();
    return string("outofbounds");
}

bool GroupSalience::setWeight(double w){
    for (unsigned int i = 0; i < group.size(); i++)
        setChildWeight(i,w);
    return true;
}

int GroupSalience::getChildCount(){
    return group.size();
}

bool GroupSalience::setChildWeights(yarp::os::Bottle& subWeights){
    if (subWeights.size() >= (int)group.size()){
        weightSum = 0.0;
        for (unsigned int i = 0; i < group.size(); i++){
            group[i]->setWeight(subWeights.get(i).asDouble());
            weightSum += subWeights.get(i).asDouble();
        }
        return true;
    }
    return false;
}

bool GroupSalience::getChildWeights(yarp::os::Bottle *subWeights){
    subWeights->clear();
    for (unsigned int i = 0; i < group.size(); i++)
        subWeights->addDouble(group[i]->getWeight());
    return true;
}

bool GroupSalience::setChildWeight(int j, double w){
    if (j < (int)group.size() && j >=0){
        weightSum -= group[j]->getWeight();
        group[j]->setWeight(w);
        weightSum += w;
        return true;
    }
    return false;
}

double GroupSalience::getChildWeight(int j){
    if (j < (int)group.size() && j >=0)
        return group[j]->getWeight();
    return 0.0;
}
