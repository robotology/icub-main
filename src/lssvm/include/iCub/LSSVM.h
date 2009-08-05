/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Main LSSVM header file
 *
 * Todo: 
 */

#ifndef _LSSVM_H
#define _LSSVM_H

#include <string>
#include <fstream>
#include <set>

#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>
#include <boost/numeric/bindings/traits/ublas_symmetric.hpp>
#include <boost/numeric/bindings/lapack/gesv.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/timer.hpp>

#include "Kernel.h"

#define SMALL_CONSTANT 0.0001
#define SERIAL_PRECISION 12


namespace ublas = boost::numeric::ublas;
namespace atlas = boost::numeric::bindings::atlas;
namespace lapack = boost::numeric::bindings::lapack;

namespace iCub {
namespace contrib {
namespace liblssvm {

//typedef ublas::vector<double> lssvm_vector;
//typedef ublas::matrix<double, ublas::column_major> lssvm_matrix;
typedef ublas::symmetric_adaptor<lssvm_matrix, ublas::upper> lssvm_sym_matrix;
typedef ublas::matrix_column< lssvm_matrix > lssvm_matrix_col;


/*
 * Standard LSSVM using Cholesky decomposition.
 */
class LSSVM {
  public:
    // constructor + destructor
    LSSVM(double c = DEFAULT_C);
    virtual ~LSSVM();

    // train
    virtual void train(const std::vector< lssvm_vector > &x, const std::vector<double> &y);
    // predict single sample
    double predict(const lssvm_vector &x_pred);
    // predict vector of samples, return prediction
    std::vector<double> predict(const std::vector< lssvm_vector > &x_pred);
    // predict vector of samples given desired output, return error
    double predict(const std::vector< lssvm_vector > &x, const  std::vector<double> &y);
    // various statistics
    double getLOO();
    void setLOO(double loo);
    virtual double getRealLOO(const std::vector<double> &y); // only use for debugging!
    double getKFoldCV(const std::vector< lssvm_vector > &x, const std::vector<double> &y, const int k);
    double getTrainingTime();
    double getKernelTime();
    double getInversionTime();
    double getPredictionTime();

    // serialize + deserialize
    void saveToFile(const std::string filename);
    static LSSVM &loadFromFile(const std::string filename);

    void save(std::ostream &ostream);
    void load(std::istream &istream);

    // print string version
    virtual std::string toString();

    // accessors + mutators
    double getC();
    void setC(double c);
    Kernel* getKernel();
    void setKernel(Kernel* kernel);
    bool getClassification();
    void setClassification(bool c);

  protected:
    int classify(double x) { return (x >= 0) ? +1 : -1; }
    double error(double desired, double actual) {
      double y_dist = desired - actual;
      if(this->getClassification()) {
        return (classify(desired) == classify(actual)) ? 0 : 1;
      } else {
        return y_dist * y_dist;
      }
    }

    // trade-off parameter c
    double c;
    // used kernel
    Kernel* kernel;
    // vector containing alpha values
    lssvm_vector alphas;
    // bias
    double bias;
    // leave one out
    double loo;
    // training time
    double trainingTime;
    // kernel matrix construction time
    double kernelTime;
    // matrix inversion time
    double inversionTime;
    // predicting time
    double predictionTime;
    // indicate whether we are doing classification or regression
    bool classification;

    // input vectors x
    std::vector< lssvm_vector > x;

    static const int DEFAULT_C = 1;
};


/*
 * Reference LSSVM using standard inversion method.
 */
class ReferenceLSSVM : public LSSVM {
  public:
    ReferenceLSSVM(double c = DEFAULT_C);
    void train(const std::vector< lssvm_vector > &x, const std::vector<double> &y);
    std::string toString();
};


/*
 * Partial LSSVM that trains on a partial random subset.
 */
class PartialLSSVM : public LSSVM {
  public:
    PartialLSSVM(double c = DEFAULT_C, double p = DEFAULT_P);
    void train(const std::vector< lssvm_vector > &x, const std::vector<double> &y);
    void train(const std::vector< lssvm_vector > &x, const std::vector<double> &y, int subsetSize);
    std::string toString();

    virtual double getRealLOO(const std::vector<double> &y); // only use for debugging!

    double getP();
    void setP(double p);
  private:
    double p;
    static const double DEFAULT_P = 0.2;

    // input vectors x
    std::vector< lssvm_vector > total_x;
};

} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
