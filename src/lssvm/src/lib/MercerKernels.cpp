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
#ifndef _MERCER_KERNELS_CPP
#define _MERCER_KERNELS_CPP

#include <math.h>
#include "iCub/MercerKernels.h"

namespace iCub {
namespace contrib {
namespace liblssvm {


/*
 * Polynomial Kernel: ((v1'*v2 + param2)^param1
 */
PolynomialKernel::PolynomialKernel(int degree, double constant) {
  this->noParams = 2;
  this->params.resize(noParams);
  this->params[0] = degree;
  this->params[1] = constant;
}

double PolynomialKernel::eval(const lssvm_vector &v1, const lssvm_vector &v2) {
  assert(v1.size() == v2.size());
  assert(this->params.size() >= this->noParams);
  assert(this->params[1] >= 0);

  return pow(Kernel::dot_product(v1, v2) + this->params[1], (int) this->params[0]);
}

kernel_id_type PolynomialKernel::getKernelID() {
  return KERNEL_ID;
}

void PolynomialKernel::sanitizeParams() {
  if(this->getParam(1) < 0.0) this->setParam(1, 0.0);
}


/*
 * Radial Base Function Kernel: exp(-param1*|v1-v2|^2)
 */
RBFKernel::RBFKernel(double gamma) {
  this->noParams = 1;
  this->params.resize(noParams);
  this->params[0] = gamma;
}

double RBFKernel::eval(const lssvm_vector &v1, const lssvm_vector &v2) {
  assert(v1.size() == v2.size());
  assert(this->params.size() >= this->noParams);
  assert(this->params[0] > 0);

  return exp(- this->params[0] * Kernel::squared_distance(v1, v2));
}

kernel_id_type RBFKernel::getKernelID() {
  return KERNEL_ID;
}

void RBFKernel::sanitizeParams() {
  if(this->getParam(0) <= 0) this->setParam(0, SMALL_CONST);
}


/*
 * Gaussian Radial Base Function Kernel: exp(-|v1-v2|^2/2param1^2)
 */
GaussianRBFKernel::GaussianRBFKernel(double sigma) {
  this->noParams = 1;
  this->params.resize(noParams);
  this->params[0] = sigma;
}

double GaussianRBFKernel::eval(const lssvm_vector &v1, const lssvm_vector &v2) {
  assert(v1.size() == v2.size());
  assert(this->params.size() >= this->noParams);
  assert(this->params[0] != 0);

  return exp( - Kernel::squared_distance(v1, v2) / (2 * this->params[0] * this->params[0]));
}

kernel_id_type GaussianRBFKernel::getKernelID() {
  return KERNEL_ID;
}

void GaussianRBFKernel::sanitizeParams() {
  if(this->getParam(0) == 0) this->setParam(0, SMALL_CONST);
}


/*
 * Boolean Function Kernel: (1+param1)^(u'*v)
 */
BooleanKernel::BooleanKernel(double gamma) {
  this->noParams = 1;
  this->params.resize(noParams);
  this->params[0] = gamma;
}

double BooleanKernel::eval(const lssvm_vector &v1, const lssvm_vector &v2) {
  assert(v1.size() == v2.size());
  assert(this->params.size() >= this->noParams);
  assert(this->params[0] > 0);

  return pow(1 + this->params[0], Kernel::dot_product(v1, v2));
}

kernel_id_type BooleanKernel::getKernelID() {
  return KERNEL_ID;
}

void BooleanKernel::sanitizeParams() {
  if(this->getParam(0) <= 0) this->setParam(0, SMALL_CONST);
}


/*
 * Sigmoid Function Kernel: tanh(param1*u'*v + param2)
 */
SigmoidKernel::SigmoidKernel(double gamma, double constant) {
  this->noParams = 2;
  this->params.resize(noParams);
  this->params[0] = gamma;
  this->params[1] = constant;
}

double SigmoidKernel::eval(const lssvm_vector &v1, const lssvm_vector &v2) {
  assert(v1.size() == v2.size());
  assert(this->params.size() >= this->noParams);
  assert(this->params[0] > 0);

  return tanh(this->params[0] * Kernel::dot_product(v1, v2) + this->params[1]);
}

kernel_id_type SigmoidKernel::getKernelID() {
  return KERNEL_ID;
}

void SigmoidKernel::sanitizeParams() {
  if(this->getParam(0) <= 0) this->setParam(0, SMALL_CONST);
}


/*
 * Sigmoid Function Kernel: tanh(param1*u'*v + param2)
 */
GeneratedKernel::GeneratedKernel() {
  // the parameter can be used to select various kinds of composed kernels.
  this->noParams = 1;
  this->params.resize(noParams);
}

double GeneratedKernel::add_kernels(double k1, double k2) {
  return k1 + k2;
}

double GeneratedKernel::multiply_kernels(double k1, double k2) {
  return k1 * k2;
}

double GeneratedKernel::weight_kernel(double w, double k2) {
  return w * k2;
}

double GeneratedKernel::polynomial_kernel_gp(const lssvm_vector &v1, const lssvm_vector &v2, int d, double c) {
  PolynomialKernel *kernel = new PolynomialKernel(d, c);
  double result = kernel->eval(v1, v2);
  delete(kernel);
  return result;
}

double GeneratedKernel::rbf_kernel_gp(const lssvm_vector &v1, const lssvm_vector &v2, double g) {
  RBFKernel *kernel = new RBFKernel(g);
  double result = kernel->eval(v1, v2);
  delete(kernel);
  return result;
}


double GeneratedKernel::eval(const lssvm_vector &v1, const lssvm_vector &v2) {
  assert(v1.size() == v2.size());
  switch(int(this->params[0])) {
    case 0:
      return 1;
      // Value for C: 
      // Fitness: 
      break;
    case 11: // diabetes
      return weight_kernel(14.6984,polynomial_kernel_gp(v1,v2,1,41.8612));
      // Value for C: 1.04858e+06
      // Fitness: 0.209575
      break;
    case 12: // diabetes 2
      return add_kernels(polynomial_kernel_gp(v1,v2,1,2479.04),add_kernels(polynomial_kernel_gp(v1,v2,1,4087.35),rbf_kernel_gp(v1,v2,0.00407447)));
      // Value for C: 356.144
      // Fitness: 0.218657
      break;
    case 13: // diabetes 3
      return polynomial_kernel_gp(v1,v2,2,0.967272);
      // Value for C: 0.00676379
      // Fitness: 0.218717
      break;


    case 21: // housing
      return weight_kernel(10.8723,polynomial_kernel_gp(v1,v2,2,4096));
      // Value for C: 0.000586752
      // Fitness: 0.161954
      break;


    case 31: // reaching 1
      return add_kernels(weight_kernel(8.57596,add_kernels(polynomial_kernel_gp(v1,v2,3,4.5471),weight_kernel(6.33162,add_kernels(rbf_kernel_gp(v1,v2,3.75235),rbf_kernel_gp(v1,v2,182.875))))),weight_kernel(9.65618,add_kernels(multiply_kernels(polynomial_kernel_gp(v1,v2,2,0.00124523),rbf_kernel_gp(v1,v2,45.4625)),rbf_kernel_gp(v1,v2,217.869))));
      // Value for C: 3625.53
      // Fitness: 0.0586628
      break;


    case 41: // reaching 2
      return add_kernels(add_kernels(rbf_kernel_gp(v1,v2,6.22523),add_kernels(polynomial_kernel_gp(v1,v2,2,5.83943),add_kernels(weight_kernel(12.6237,weight_kernel(14.3648,rbf_kernel_gp(v1,v2,0.208867))),weight_kernel(15.8328,rbf_kernel_gp(v1,v2,0.366326))))),polynomial_kernel_gp(v1,v2,4,5.40037));
      // Value for C: 15.3218
      // Fitness: 0.00374935
      break;


    case 51: // reaching 3
      return multiply_kernels(polynomial_kernel_gp(v1,v2,5,4.68267),rbf_kernel_gp(v1,v2,0.0964027));
      // Value for C: 0.0625065
      // Fitness: 0.00184249
      break;


    case 61: // wisconsin
      return add_kernels(weight_kernel(18.4809,weight_kernel(11.4193,polynomial_kernel_gp(v1,v2,3,0.00208004))),weight_kernel(20,add_kernels(rbf_kernel_gp(v1,v2,0.0112758),rbf_kernel_gp(v1,v2,962.2))));
      // Value for C: 5.29471e-06
      // Fitness: 0.0333333
      break;
    case 8:
      break;
    case 9:
      break;
  }

  return 0.0;

}

kernel_id_type GeneratedKernel::getKernelID() {
  return KERNEL_ID;
}

void GeneratedKernel::sanitizeParams() {
}

Kernel* createGenerated() { return new GeneratedKernel(); }


/*
 * Register kernels here
 */
void registerKernels() {
  KernelFactory::instance().regCreateFn(PolynomialKernel::KERNEL_ID, PolynomialKernel::create);
  KernelFactory::instance().regCreateFn(RBFKernel::KERNEL_ID, RBFKernel::create);
  KernelFactory::instance().regCreateFn(GaussianRBFKernel::KERNEL_ID, GaussianRBFKernel::create);
  KernelFactory::instance().regCreateFn(BooleanKernel::KERNEL_ID, BooleanKernel::create);
  KernelFactory::instance().regCreateFn(SigmoidKernel::KERNEL_ID, SigmoidKernel::create);
  KernelFactory::instance().regCreateFn(GeneratedKernel::KERNEL_ID, GeneratedKernel::create);
}

} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
