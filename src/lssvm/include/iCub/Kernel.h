/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Abstract Kernel class
 *
 * Note:
 *  Implementation emphasizes on programmer convenience to plugin in 
 *  custom kernel implementation. In order to use a custom kernel, the
 *  programmer needs to extend the Kernel class and to register a construction
 *  method in the KernelFactory.
 *
 * Todo:
 * - See MercerKernels.h
 */
#ifndef _KERNEL_H
#define _KERNEL_H

#include <boost/numeric/bindings/atlas/clapack.hpp>
#include <boost/numeric/bindings/atlas/cblas.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <clapack.h>

#include <string>
#include <map>

namespace ublas = boost::numeric::ublas;
namespace atlas = boost::numeric::bindings::atlas;

namespace iCub {
namespace contrib {
namespace liblssvm {

typedef ublas::vector<double> lssvm_vector;
typedef ublas::matrix<double, ublas::column_major> lssvm_matrix;
typedef ublas::symmetric_adaptor<lssvm_matrix, ublas::upper> lssvm_sym_matrix;
// subtype of kernel used (int id)
typedef unsigned int kernel_id_type;
typedef std::vector<double> param_array;

class Kernel {
  public:
    // destructor
    virtual ~Kernel() {}
    // evaluate kernel function on two input vectors (pure virtual)
    virtual double eval(const lssvm_vector &v1, const lssvm_vector &v2) = 0;
    // returns a kernel matrix for the current kernel function
    virtual lssvm_matrix getMatrix(const std::vector< lssvm_vector > &x);
    virtual lssvm_matrix getMatrix(const std::vector< lssvm_vector > &x, const double c);

    // static method to load a kernel from a string representation
    static Kernel* fromString(std::string str);
    // write a kernel to a string representation
    std::string toString();

    // load kernel configuration from a string
    virtual void loadConfig(const std::string& str);
    // save kernel configuration to a string
    virtual std::string saveConfig();

    // return kernel identification (pure virtual)
    virtual kernel_id_type getKernelID() = 0;

    // routine to remove non valid parameters, used for EC integration
    virtual void sanitizeParams() {};


    // return kernel parameters, not used generally
    param_array getParams();
    // set kernel parameters, not used generally
    void setParams(param_array p);

    // get kernel parameter with specified index
    double getParam(unsigned int index);
    // set kernel parameter with specified index
    void setParam(unsigned int index, double val);

    // return number of parameters
    size_t getNoParams();
    // set number of parameters
    void setNoParams(size_t pc);

  protected:
    // constructor, should not be used
    Kernel() {}

    // support function to tokenize a string into a vector given a delimiter (static)
    static void tokenize(const std::string& str, const std::string& delimiter,
                                              std::vector<std::string>& tokens);

    // support function to calculate dot product of two vectors
    static double dot_product(const lssvm_vector &v1, const lssvm_vector &v2);
    // support function to calculate squared distance of two vectors
    static double squared_distance(const lssvm_vector &v1, const lssvm_vector &v2);

    // kernel parameters
    param_array params;
    // number of parameters
    size_t noParams;
  private:
};

class KernelFactory {
  public:
    // function pointer to kernel subtype creating function
    typedef Kernel* (* kernel_creator)();

    // single threaded singleton
    static KernelFactory& instance();

    // register creator function
    bool regCreateFn(kernel_id_type key, kernel_creator func);

    // calls creator function associated with the key
    Kernel* create(kernel_id_type key) const;

  private:
    std::map<kernel_id_type, kernel_creator> kernel_map;
};

} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
