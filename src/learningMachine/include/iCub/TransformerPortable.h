/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Convenience header to define TransformerPortable type.
 *
 */

#ifndef __ICUB_TRANSFORMERPORTABLE__
#define __ICUB_TRANSFORMERPORTABLE__

#include "iCub/PortableT.h"
#include "iCub/ITransformer.h"


namespace iCub {
namespace contrib {
namespace learningmachine {

typedef PortableT<ITransformer> TransformerPortable;

} // learningmachine
} // contrib
} // iCub
