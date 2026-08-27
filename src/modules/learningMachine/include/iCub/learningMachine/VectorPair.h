/*
 * Copyright (C) 2026 Istituto Italiano di Tecnologia (IIT)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef LM_VECTORPAIR_H
#define LM_VECTORPAIR_H

#include <cstdint>

#include <yarp/os/Bottle.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/Portable.h>
#include <yarp/sig/Vector.h>

namespace iCub {
namespace learningmachine {

/**
 * A pair of vectors using the wire format formerly provided by
 * yarp::os::PortablePair<yarp::sig::Vector, yarp::sig::Vector>.
 *
 * PortablePair was removed in YARP 4.  Keeping the serialization local makes
 * learningMachine compatible with both existing ports and recorded data.
 */
class VectorPair : public yarp::os::Portable
{
public:
    yarp::sig::Vector head;
    yarp::sig::Vector body;

    bool read(yarp::os::ConnectionReader& connection) override
    {
        connection.convertTextMode();

        const std::int32_t header = connection.expectInt32();
        if (header != BOTTLE_TAG_LIST) {
            return false;
        }

        const std::int32_t length = connection.expectInt32();
        if (length != 2) {
            return false;
        }

        return head.read(connection) && body.read(connection);
    }

    bool write(yarp::os::ConnectionWriter& connection) const override
    {
        connection.appendInt32(BOTTLE_TAG_LIST);
        connection.appendInt32(2);

        const bool ok = head.write(connection) && body.write(connection);
        if (ok) {
            connection.convertTextMode();
        }
        return ok;
    }

    void onCompletion() const override
    {
        head.onCompletion();
        body.onCompletion();
    }
};

} // namespace learningmachine
} // namespace iCub

#endif // LM_VECTORPAIR_H
