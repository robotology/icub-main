/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Machine factory class for creating types of IMachineLearner objects.
 *
 */

#ifndef __ICUB_MACHINEPORTABLE__
#define __ICUB_MACHINEPORTABLE__

#include <stdexcept>

#include <yarp/os/Portable.h>

#include "iCub/IMachineLearner.h"
#include "iCub/MachineFactory.h"


namespace iCub {
namespace contrib {
namespace learningmachine {

/**
 *
 * A portable class for learning machines. This class can be used to send
 * abstract machine types over ports.
 *
 * \see iCub::contrib::learningmachine::IMachineLearner
 * \see iCub::contrib::learningmachine::MachineFactory
 * \see iCub::contrib::learningmachine::TrainModule
 * \see iCub::contrib::learningmachine::PredictModule
 *
 * \author Arjan Gijsberts
 *
 */

class MachinePortable : public Portable {
private:
    /**
     * A pointer to the actual machine implementation.
     */
    IMachineLearner* machine;

    /**
     * The factory for creating machines.
     */
    MachineFactory* factory;

public:
    /**
     * Constructor.
     *
     * @param m The initial wrapped learning machine
     */
    MachinePortable(IMachineLearner* m = (IMachineLearner*) 0) : machine(m) {
    }

    /**
     * Constructor.
     *
     * @param machineName The name specifier for the wrapped learning machine
     */
    MachinePortable(std::string& machineName) {
        this->machine = MachineFactory::instance().create(machineName);
    }

    /**
     * Destructor.
     */
    ~MachinePortable() {
        delete(this->machine);
    }
    
    /**
     * Writes a portable machine to a connection.
     *
     * @param connection the connection
     * @return true on success
     */
    bool write(ConnectionWriter& connection);

    /**
     * Reads a portable machine from a connection.
     *
     * @param connection the connection
     * @return true on success
     */
    bool read(ConnectionReader& connection);

    /**
     * Writes a portable machine to a file.
     *
     * @param filename the filename
     * @return true on success
     */
    bool writeToFile(std::string filename);
    
    /**
     * Reads a portable machine from a file.
     *
     * @param filename the filename
     * @return true on success
     */
    bool readFromFile(std::string filename); 

    /**
     * Returns true iff this wrapping object contains an actual machine.
     *
     * @return true if there is a wrapped machine
     */
    bool hasMachine() {
        return (this->machine != (IMachineLearner*) 0);
    }

    /**
     * The accessor for the actual machine that this object wraps around.
     *
     * @return a pointer to the actual machine
     */
    IMachineLearner* getMachine();

    /**
     * The mutator for the actual machine.
     *
     * @param m a pointer for the machine
     */
    void setMachine(IMachineLearner* m) {
        this->machine = m;
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
