// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_SALIENCE_INC
#define ICUB_SALIENCE_INC

// std
#include <iostream>
#include <string>

// yarp
#include <yarp/os/ConstString.h>
#include <yarp/os/IConfig.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Bottle.h>

// iCub
#include <iCub/SalienceInterfaces.h>
#include <iCub/Conspicuity.h>

using namespace std;
using namespace yarp::os;

/**
 * Classes developed as part of the iCub project.
 */
namespace iCub {
    /**
     *
     * Contributed classes.  Developers can put anything they like
     * here.  Over time, these classes may get reorganized into other
     * namespaces and made "official".  In that case, a typedef (or
     * equivalent) should be retained here for some time so that users
     * don't have to worry about the classes disappearing.
     *
     */
    namespace contrib {
        class Salience;
    }
}

/**
 * Base class for salience filters.
 */
class iCub::contrib::Salience : public iCub::contrib::ISalienceControls,
                                public yarp::os::IConfig {
protected:
    double weight;
    string filterName;
    bool activateConspicuity;
    Conspicuity conspicuity;

public:

    Salience();
    virtual ~Salience();

    /**
     * IConfig open. Call first from every Salience implementation open() method.
     */
    virtual bool open(yarp::os::Searchable& config);

    /**
     * IConfig close. Call first from every Salience implementation close() method.
     */
    virtual bool close();

    /**
     * Respond to remote calls (called from Salience Module or parent filters)
     */
    virtual bool respond(const Bottle &command,Bottle &reply){return true;};

    /**
     * Apply the salience filter.\n
     * Call this function to apply the filter. 
     * Apply() performs operations common to all filters (like conspicuity
     * calculation if requested by the configuration) and then call applyImpl().
     * @param src The source image
     * @param dest An annotated version of the image (for debugging).
     * This visualization image is currently not necessarily calculated by every filter.
     * (As of 070603: Currently the SalienceModule discardes the dest image and calculates an RGB image from the sal float map at the end of the update loop)
     * @param sal The salience map output
     */
    virtual void apply(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
                       yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
                       yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);

    /**
     * Filter implementations should implement this function as it serves as a hook and
     * will be called by apply().
     */
    virtual void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
                       yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
                       yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal) ;

    // planned extension for access to hierarchical tree of filters
    // remote like:
    // filt "filter_name" set config config_key config_value
    // filt "filter_name" get config [config_key]
    // filt "filter_name" set w 0.5
    // filt "filter_name" set wc 1 0.5
    /*
    virtual std::vector<Salience*> getChildren(){
        return NULL;
    }

    virtual Salience* getChild(string name){
        return NULL;
    }

    virtual Bottle getConfiguration(){
        ...
    }

    virtual void setConfiguration(Bottle &config){
        configure(config);
    }
    */

    virtual bool setFilterName(string n){
        filterName = n;
        return true;
    }

    virtual bool setChildFilterName(int j, string s){
        cout << "Salience::setChildFilterName not implemented!" << endl;
        return false;
    }

    virtual string getFilterName(){
        return filterName;
    }

    virtual string getChildFilterName(int j){
        cout << "Salience::getChildFilterName not implemented!" << endl;
        return false;
    }

    virtual bool setWeight(double w){
        weight = w;
        return true;
    }
    virtual double getWeight(){
        return weight;
    }
    virtual int getChildCount(){
        return 0;
    }
    virtual bool setChildWeights(yarp::os::Bottle& subWeights){
        cout << "Salience::setChildWeights not implemented!" << endl;
        return false;
    } 
    virtual bool getChildWeights(yarp::os::Bottle *subWeights){
        cout << "Salience::getChildWeights not implemented!" << endl;
        return false;
    }
    virtual bool setChildWeight(int j, double w){
        cout << "Salience::setChildWeight not implemented!" << endl;
        return false;
    }
    virtual double getChildWeight(int j){
        cout << "Salience::getChildWeight not implemented!" << endl;
        return 0.0;
    }
};

#endif
