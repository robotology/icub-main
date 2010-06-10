// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick, Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_GROUPSALIENCE_INC
#define ICUB_GROUPSALIENCE_INC

// std
#include <vector>
#include <string>
#include <iostream>

// iCub
#include <iCub/vis/Salience.h>
#include <iCub/vis/SalienceFactory.h>

namespace iCub {
    namespace vis {
        class GroupSalience;
    }
}


/**
 * A group of filters.
 */
class iCub::vis::GroupSalience : public Salience { 
public:
    GroupSalience() {
        weightSum = 0.0f;
    }
    
    virtual ~GroupSalience() {
        clear();
    }

    virtual bool open(yarp::os::Searchable& config);

    virtual bool configure(yarp::os::Searchable& config);

    virtual bool respond(const Bottle &command,Bottle &reply);

    virtual void apply(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
                       yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
                       yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);

    virtual void add(Salience *filter);
 
    virtual bool setChildFilterName(int j, string s);
    virtual string getChildFilterName(int j);
    virtual bool setWeight(double w);
    virtual int getChildCount();
    virtual bool setChildWeights(yarp::os::Bottle& subWeights);
    virtual bool getChildWeights(yarp::os::Bottle *subWeights);
    virtual bool setChildWeight(int j, double w);
    virtual double getChildWeight(int j);

private:
    void clear() {
        for (unsigned int i=0; i<group.size(); i++) {
            delete group[i];
            group[i] = NULL;
        }
        group.clear();
        salienceMap.clear();
        salienceView.clear();
    }
    std::vector<Salience*> group;
    std::vector<yarp::sig::ImageOf<yarp::sig::PixelFloat> > salienceMap;
    std::vector<yarp::sig::ImageOf<yarp::sig::PixelRgb> > salienceView;
    double weightSum;
    
};


#endif
