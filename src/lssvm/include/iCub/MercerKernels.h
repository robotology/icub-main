/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Mercer Kernels
 * 
 * Note:
 * - Follow pattern when adding new Kernels. The kernel must extend Kernels
 *   and must register itself with the KernelFactory. Make sure to give the
 *   kernel a unique KERNEL_ID.
 *
 * Todo: 
 * - Possible add optimizations in kernel evaluations possible (e.g. store 
 *   squares and reuse them: dynamic programming?)
 * - Add more kernels
 */
#ifndef _MERCER_KERNELS_H
#define _MERCER_KERNELS_H

#define SMALL_CONST 0.00001

#include <math.h>
#include "Kernel.h"

namespace iCub {
namespace contrib {
namespace liblssvm {

void registerKernels();

/*
 * Polynomial Kernel: ((v1'*v2 + param2)^param1
 */
class PolynomialKernel : public Kernel {
  public:
    static const kernel_id_type KERNEL_ID = 0;

    PolynomialKernel(int degree = DEF_DEGREE, double constant = DEF_CONSTANT);

    double eval(const lssvm_vector &v1, const lssvm_vector &v2);

    kernel_id_type getKernelID();

    void sanitizeParams();

    static Kernel* create() { return new PolynomialKernel(); }

  private:
    static const int DEF_DEGREE = 2;
    static const double DEF_CONSTANT = 1.0;
};


/*
 * Radial Base Function Kernel: exp(-param1*|v1-v2|^2)
 */
class RBFKernel : public Kernel {
  public:
    static const kernel_id_type KERNEL_ID = 1;

    RBFKernel(double gamma = DEF_GAMMA);

    double eval(const lssvm_vector &v1, const lssvm_vector &v2);

    kernel_id_type getKernelID();

    void sanitizeParams();

    static Kernel* create() { return new RBFKernel(); }
  private:
    static const double DEF_GAMMA = 1.0;
};


/*
 * Gaussian Radial Base Function Kernel: exp(-|v1-v2|^2/2param1^2)
 */
class GaussianRBFKernel : public Kernel {
  public:
    static const kernel_id_type KERNEL_ID = 2;

    GaussianRBFKernel(double sigma = DEF_SIGMA);

    double eval(const lssvm_vector &v1, const lssvm_vector &v2);

    kernel_id_type getKernelID();

    void sanitizeParams();

    static Kernel* create() { return new GaussianRBFKernel(); }
  private:
    static const double DEF_SIGMA = 1.0;
};


/*
 * Boolean Function Kernel: (1+param1)^(u'*v)
 */
class BooleanKernel : public Kernel {
  public:
    static const kernel_id_type KERNEL_ID = 3;

    BooleanKernel(double gamma = DEF_GAMMA);

    double eval(const lssvm_vector &v1, const lssvm_vector &v2);

    kernel_id_type getKernelID();

    void sanitizeParams();

    static Kernel* create() { return new BooleanKernel(); }
  private:
    static const double DEF_GAMMA = 1.0;
};


/*
 * Sigmoid Function Kernel: tanh(param1*u'*v + param2)
 */
class SigmoidKernel : public Kernel {
  public:
    static const kernel_id_type KERNEL_ID = 4;

    SigmoidKernel(double gamma = DEF_GAMMA, double constant = DEF_CONSTANT);

    double eval(const lssvm_vector &v1, const lssvm_vector &v2);

    kernel_id_type getKernelID();

    void sanitizeParams();

    static Kernel* create() { return new SigmoidKernel(); }
  private:
    static const double DEF_GAMMA = 1.0;
    static const double DEF_CONSTANT = 1.0;
};


/*
 * Test Kernel for GP Generated Kernels (parsed individual can be copy pasted here)
 */
class GeneratedKernel : public Kernel {
  public:
    static const kernel_id_type KERNEL_ID = 1000;

    GeneratedKernel();

    double add_kernels(double k1, double k2);
    double multiply_kernels(double k1, double k2);
    double weight_kernel(double w, double k2);

    double polynomial_kernel_gp(const lssvm_vector &v1, const lssvm_vector &v2, int d, double c);
    double rbf_kernel_gp(const lssvm_vector &v1, const lssvm_vector &v2, double g);


    double eval(const lssvm_vector &v1, const lssvm_vector &v2);

    kernel_id_type getKernelID();

    void sanitizeParams();

    static Kernel* create() { return new GeneratedKernel(); }
  private:
};


} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
