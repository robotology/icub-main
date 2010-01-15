/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#ifndef LM_TRANSFORMERCATALOGUE__
#define LM_TRANSFORMERCATALOGUE__

#include "iCub/ITransformer.h"
#include "iCub/IScaler.h"
#include "iCub/ScaleTransformer.h"
#include "iCub/Standardizer.h"
#include "iCub/Normalizer.h"
#include "iCub/FixedRangeScaler.h"
#include "iCub/RandomFeature.h"


namespace iCub {
namespace learningmachine {

void registerTransformers() {
    // register scalers
    FactoryT<std::string, IScaler>::instance().registerPrototype(new NullScaler());
    FactoryT<std::string, IScaler>::instance().registerPrototype(new Standardizer());
    FactoryT<std::string, IScaler>::instance().registerPrototype(new Normalizer());
    FactoryT<std::string, IScaler>::instance().registerPrototype(new FixedRangeScaler());

    // register proper transformers
    FactoryT<std::string, ITransformer>::instance().registerPrototype(new ScaleTransformer());
    FactoryT<std::string, ITransformer>::instance().registerPrototype(new RandomFeature());


}

} // learningmachine
} // iCub

#endif
