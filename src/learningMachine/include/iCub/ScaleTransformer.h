/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Linear scaling transformer.
 *
 */

#ifndef __ICUB_SCALETRANSFORMER__
#define __ICUB_SCALETRANSFORMER__

#include <vector>
#include <stdexcept>

#include "iCub/IFixedSizeTransformer.h"
#include "iCub/IScaler.h"

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace contrib {
namespace learningmachine {

class TransformerSupport;

/**
 * The ScaleTransformer is a ITransformer that supports element-based scaling
 * transformations. These transformations include standardization, normalization 
 * and other kind of linear scalings based on an offset and scale. The type and 
 * configuration can be set for individual elements in the input vectors.
 *
 * \see iCub::contrib::learningmachine::IFixedSizeTransformer
 * \see iCub::contrib::learningmachine::ITransformer
 * \see iCub::contrib::learningmachine::IScaler
 *
 * \author Arjan Gijsberts
 *
 */

/* 
 * Note that this class internally handles a 'null' type scaler, which is basically a 
 * null pointer in the vector of scalers. This has the disadvantage that regular checking
 * for null pointers is needed in various methods. An alternative would have been to 
 * explicitely declare a passthrough scaler that does not do any operation on the samples. 
 * However, this would add some overhead on the method calling and would create a requirement
 * that the user has registered this null scaler in the factory. In its current form, with 
 * internal handling of null scalers, we do not have this dependency and the class will 
 * not cause any conflict in the absence of scalers.
 */
class ScaleTransformer : public IFixedSizeTransformer {
private:
    /**
     * The vector of IScaler objects.
     */
    std::vector<IScaler*> scalers;

    /**
     * Resets the vector of scalers and deletes each element.
     */
    void deleteAll();

    /**
     * Resets the vector of scalers and deletes each element.
     *
     * @param size the desired size of the vector
     */
    void deleteAll(int size);

    /**
     * Sets the scaler at a certain position to a given type.
     *
     * @param index the index of the scaler to change
     * @param type the key identifier of the desired scaler
     */
    void setAt(int index, std::string type);

    /**
     * Returns a pointer to the scaler at a certain position.
     *
     * @param index the index of the scaler
     */
    IScaler* getAt(int index) {
        if(index >= 0 && index < this->scalers.size()) {
            return this->scalers[index];
        } else {
            throw std::runtime_error("Index for scaler out of bounds!");
        }
    }

    /**
     * Checks whether the scaler on a certain position is an empty scaler 
     * ('null').
     *
     * @param index the index of the scaler to check
     * @return true if the scaler is null
     */
    bool isEmptyScaler(int index);

    /**
     * Sets all scalers to a given type.
     */
    void setAll(std::string type);

    /*
     * Inherited from ITransformer.
     */
    virtual void writeBottle(Bottle& bot);

    /*
     * Inherited from ITransformer.
     */
    virtual void readBottle(Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param dom initial domain/codomain size
     */
    ScaleTransformer(int dom = 1);

    /**
     * Destructor.
     */
    ~ScaleTransformer();

    /*
     * Inherited from ITransformer.
     */
    virtual ITransformer* create() {
        return new ScaleTransformer();
    }

    /*
     * Inherited from ITransformer.
     */
    virtual void transform(const Vector& input, Vector& output);

    /*
     * Inherited from ITransformer.
     */
    virtual std::string getInfo();

    /*
     * Inherited from ITransformer.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from ITransformer.
     */
    virtual void setDomainSize(int size);

    /*
     * Inherited from ITransformer.
     */
    virtual void setCoDomainSize(int size);

    /*
     * Inherited from ITransformer.
     */
    virtual void reset();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable &config);

protected:

};


} // learningmachine
} // contrib
} // iCub

#endif
