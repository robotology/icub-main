/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Problem class (to read file and convert to x and y vectors)
 *
 * Note:
 *
 * Todo:
 * - Could use some rewriting. The definitions for serialization could be more 
 *   consistent and clear and the read method could be reimplemented to C++ style.
 *
 */
#ifndef _PROBLEM_H
#define _PROBLEM_H

#include "LSSVM.h"
#include <string>

#define SERIAL_PRECISION 12

namespace iCub {
namespace contrib {
namespace liblssvm {

/*
 * Class containing problem: vector of input vectors x and vector of output 
 * scalars y
 */
class Problem {
  public:
    Problem();
    void readSparseFile(std::string filename);
    void writeSparseFile(std::ostream& ostream);

    void save(std::ostream &ostream);
    void load(std::istream &istream);

    std::pair<double, double> normalizeLabels();
    void scaleLabels(const double lower, const double upper);
    void normalizeFeatures();

    std::vector< lssvm_vector > x;
    std::vector<double> y;
    bool classification;
};

} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
