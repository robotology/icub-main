/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Kernel Alignment header file
 *
 * Todo: 
 */

#ifndef _KERNEL_ALIGNMENT_H
#define _KERNEL_ALIGNMENT_H

#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>
#include <boost/numeric/bindings/traits/ublas_symmetric.hpp>

#include <boost/numeric/ublas/vector_expression.hpp>


#include <boost/numeric/bindings/traits/std_vector.hpp>
//#include <boost/numeric/ublas/io.hpp>
//#include <string>
//#include <fstream>

#include "Kernel.h"

#include <boost/timer.hpp>

namespace ublas = boost::numeric::ublas;
// namespace atlas = boost::numeric::bindings::atlas;
// namespace lapack = boost::numeric::bindings::lapack;

namespace iCub {
namespace contrib {
namespace liblssvm {

class KernelAlignment {
  public:
    // constructor + destructor
    KernelAlignment(double c = DEFAULT_C);
    ~KernelAlignment();

    // standard: 2 matrices
    //double calculate(const lssvm_matrix &k1, const lssvm_matrix &k2);
    // kernel target alignment (rank 1 matrix yy')
    //double calculate(const lssvm_matrix &k1, const std::vector<double> &y);
    // most useful and efficient: kernel alignment over dataset when kernel is set
    double calculate(const std::vector< lssvm_vector > &x, const std::vector<double> &y);
    //double calculate(const std::vector<double> &x, const std::vector<double> &y);

    double frobeniusProduct(const lssvm_matrix &K, const std::vector<double> &y);
    double frobeniusNorm(const lssvm_matrix &K);
    double frobeniusNorm(const std::vector<double> &y);
    //double frobeniusNorm(const lssvm_matrix &m);

    double frobeniusProduct(const std::vector<double> &x, const std::vector<double> &y);

    void preprocessLabels(std::vector<double> &y);

    // accessors + mutators
    Kernel &getKernel();
    void setKernel(Kernel& kernel);

    double getAlignment();
    double getAlignmentTime();

    double getC();
    void setPreprocessMatrix(bool matrix_pp);
    bool getPreprocessMatrix();

  private:
    void preprocessMatrix(lssvm_matrix &x);

    // used kernel
    Kernel* kernel;
    // latest alignment score
    double alignment;
    // kernel alignment calculation time
    double alignmentTime;
    // double c
    double c;
    // default c
    static const int DEFAULT_C = 1;
    // matrix preprocess
    bool matrix_pp;

};

} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
