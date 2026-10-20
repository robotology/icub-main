/**
 * Utilities for working with YARP vectors.
 */

#ifndef ICUB_CTRL_VECTOR_UTILS_H
#define ICUB_CTRL_VECTOR_UTILS_H

#include <cstddef>
#include <yarp/sig/Vector.h>

namespace iCub
{
namespace ctrl
{

/**
 * Copy the values of src into dest starting at position.
 *
 * The destination is not resized. Returns false if the requested range does
 * not fit in dest.
 */
inline bool setSubVector(yarp::sig::Vector& dest,
                         int position,
                         const yarp::sig::Vector& src)
{
    if (position < 0 || static_cast<std::size_t>(position) > dest.size() ||
        src.size() > dest.size() - static_cast<std::size_t>(position)) {
        return false;
    }

    for (std::size_t i = 0; i < src.size(); ++i) {
        dest[static_cast<std::size_t>(position) + i] = src(i);
    }
    return true;
}

} // namespace ctrl
} // namespace iCub

#endif // ICUB_CTRL_VECTOR_UTILS_H
