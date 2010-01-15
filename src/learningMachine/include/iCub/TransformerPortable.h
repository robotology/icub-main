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

#ifndef LM_TRANSFORMERPORTABLE__
#define LM_TRANSFORMERPORTABLE__

#include "iCub/PortableT.h"
#include "iCub/ITransformer.h"


namespace iCub {
namespace learningmachine {

/**
 * \ingroup icub_libLM_transformers
 *
 * A portable wrapper around an ITransformer.
 *
 * \see iCub::learningMachine::PortableT
 * \see iCub::learningMachine::ITransformer
 *
 * \author Arjan Gijsberts
 */

typedef PortableT<ITransformer> TransformerPortable;

} // learningmachine
} // iCub

#endif

