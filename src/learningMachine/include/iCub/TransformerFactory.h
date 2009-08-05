/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Transformer factory class for creating types of ITransformer and IScaler 
 objects.
 *
 */

#ifndef __ICUB_TRANSFORMERFACTORY__
#define __ICUB_TRANSFORMERFACTORY__

#include "iCub/FactoryT.h"
#include "iCub/ITransformer.h"
#include "iCub/IScaler.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

typedef FactoryT<std::string, ITransformer> TransformerFactory;
typedef FactoryT<std::string, IScaler> ScalerFactory;

} // learningmachine
} // contrib
} // iCub

#endif
