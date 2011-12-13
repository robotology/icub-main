/*
 * Copyright (C) 2007-2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
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

#ifndef LM_SCALETRANSFORMER__
#define LM_SCALETRANSFORMER__

#include <vector>

#include "iCub/learningMachine/IFixedSizeTransformer.h"
#include "iCub/learningMachine/IScaler.h"

namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_transformers
 *
 * The ScaleTransformer is a ITransformer that supports element-based scaling
 * transformations. These transformations include standardization, normalization
 * and other kind of linear scalings based on an offset and scale. The type and
 * configuration can be set for individual elements in the input vectors.
 *
 * \see iCub::learningmachine::IFixedSizeTransformer
 * \see iCub::learningmachine::ITransformer
 * \see iCub::learningmachine::IScaler
 *
 * \author Arjan Gijsberts
 *
 */

class ScaleTransformer : public IFixedSizeTransformer {
protected:
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
     * @throw runtime error if the index is out of bounds
     */
    IScaler* getAt(int index);

    /**
     * Sets all scalers to a given type.
     */
    void setAll(std::string type);

    /*
     * Inherited from ITransformer.
     */
    virtual void writeBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from ITransformer.
     */
    virtual void readBottle(yarp::os::Bottle& bot);

public:
    /**
     * Constructor.
     *
     * @param dom initial domain/codomain size
     */
    ScaleTransformer(unsigned int dom = 1);

    /**
     * Copy constructor.
     */
    ScaleTransformer(const ScaleTransformer& other);

    /**
     * Destructor.
     */
    virtual ~ScaleTransformer();

    /**
     * Assignment operator.
     */
    ScaleTransformer& operator=(const ScaleTransformer& other);

    /*
     * Inherited from ITransformer.
     */
    virtual ScaleTransformer* clone() {
        return new ScaleTransformer(*this);
    }

    /*
     * Inherited from ITransformer.
     */
    virtual yarp::sig::Vector transform(const yarp::sig::Vector& input);

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
    virtual void setDomainSize(unsigned int size);

    /*
     * Inherited from ITransformer.
     */
    virtual void setCoDomainSize(unsigned int size);

    /*
     * Inherited from ITransformer.
     */
    virtual void reset();

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable &config);

protected:

};

} // learningmachine
} // iCub

#endif
