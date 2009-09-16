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

#include "iCub/IFixedSizeTransformer.h"
#include "iCub/IScaler.h"
#include "iCub/TransformerSupport.h"

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
     * An instance of the TransformerSupport class.
     */
    TransformerSupport* support;
     
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

public:
    /**
     * Constructor.
     *
     * @param name name for the 
     * @param size initial size of the vector
     */
    ScaleTransformer(TransformerSupport* support, std::string name = "Scaler", int size = 1);

    /**
     * Destructor.
     */
    ~ScaleTransformer();

    /**
     * Feeds a single sample into the scaler, so that it can use this sample
     * to learn the appropriate offset and scale.
     *
     * @param value the sample value
     */
    //virtual void feedSample(const Vector& in);

    /*
     * Inherited from ITransformer.
     */
    virtual ITransformer* create() {
        return new ScaleTransformer(this->support);
    }

    /*
     * Inherited from ITransformer.
     */
    virtual void transform(const Vector& input, Vector& output);

    /*
     * Inherited from ITransformer.
     */
    virtual std::string getStats();

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
