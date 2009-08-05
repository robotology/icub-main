/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Kernel Alignment implementation
 *
 * Note:
 * - Template programming would be more neat...
 *
 * Todo:
 * - Check implementation of the Kernel Alignment
 * - Possibly add other implementations besides just Kernel Target Alignment (K vs K etc.)
 * - Check yy' vs yy'
 *
 */
#ifndef _KERNEL_ALIGNMENT_CPP
#define _KERNEL_ALIGNMENT_CPP

#include "iCub/KernelAlignment.h"

namespace iCub {
namespace contrib {
namespace liblssvm {

KernelAlignment::KernelAlignment(double c) {
  this->alignment = 0.0;
  this->alignmentTime = -1.0;
  this->c = c;
  this->matrix_pp = false;
}

KernelAlignment::~KernelAlignment() {
  delete(this->kernel);
}


// double KernelAlignment::calculate(const lssvm_matrix &k1, const lssvm_matrix &k2) {
//   boost::timer* timer = new boost::timer();
// 
//   this->alignment = 1.0;
// 
//   this->alignmentTime = timer->elapsed();
//   delete(timer);
//   return this->alignment;
// }

// double KernelAlignment::calculate(const lssvm_matrix &k1, const std::vector<double> &y) {
//   boost::timer* timer = new boost::timer();
// 
//   this->alignment = 2.0;
// 
//   this->alignmentTime = timer->elapsed();
//   delete(timer);
//   return this->alignment;
// }

double KernelAlignment::calculate(const std::vector< lssvm_vector > &x, const std::vector<double> &y) {
  boost::timer* timer = new boost::timer();
  assert(x.size() == y.size());

  //std::cout << "Product F(K, yy'): " << frobeniusProduct(x,y) << std::endl;
  //std::cout << "Norm F(K): " << frobeniusNorm(x) << std::endl;
  //std::cout << "Norm F(yy'): " << frobeniusNorm(y) << std::endl;

  lssvm_matrix K = this->kernel->getMatrix(x, this->c);

  if(this->matrix_pp)
    this->preprocessMatrix(K);

  this->alignment = frobeniusProduct(K, y) / sqrt(frobeniusNorm(K) * frobeniusNorm(y));

  this->alignmentTime = timer->elapsed();
  delete(timer);
  return this->alignment;
}



/*
 * Frobenius norm over a vector of input vectors, kernel is applied to these.
 * This saves to construct the complete matrix.
 */
double KernelAlignment::frobeniusProduct(const lssvm_matrix &K, const std::vector<double> &y) {
  assert(K.size1() == y.size());
  assert(K.size1() == K.size2());
  assert(this->kernel != 0);
  double sum = 0.0;
  double eval;
  for(unsigned int i = 0; i < K.size1(); i++) {
    //for(unsigned int j = 0; j < x.size(); j++) {
    for(unsigned int j = 0; j <= i; j++) {
      eval = K(i, j) * (y[i] * y[j]);
      // add 1/this->c or not?
      //sum += eval;
      sum += (j < i) ? 2 * eval : eval;
      //sum += (j < i) ? 2 * eval : eval;
    }
  }
  return sum;

}


/*
 * Frobenius norm over a vector of input vectors, kernel is applied to these.
 * This saves to construct the complete matrix.
 */
double KernelAlignment::frobeniusNorm(const lssvm_matrix &K) {
  assert(K.size1() == K.size2());
  assert(this->kernel != 0);
  double sum = 0.0;
  double eval;
  for(unsigned int i = 0; i < K.size1(); i++) {
    //for(unsigned int j = 0; j < x.size(); j++) {
    for(unsigned int j = 0; j <= i; j++) {
      eval = K(i, j);
      //sum += eval * eval;
      sum += (j < i) ? 2 * eval * eval : eval * eval;
    }
  }
  return sum;
}

/*
 * Frobenius norm over a vector of doubles.
 * This saves calculating the outer prod y.y'
 */
double KernelAlignment::frobeniusNorm(const std::vector<double> &y) {
  double sum = 0.0;
  double eval;
  for(unsigned int i = 0; i < y.size(); i++) {
    //for(unsigned int j = 0; j < y.size(); j++) {
    for(unsigned int j = 0; j <= i; j++) {
      eval = y[i] * y[j];
      //sum +=	 eval * eval;
      sum += (j < i) ? 2 * eval * eval : eval * eval;
    }
  }
  return sum;

}

/*
 * Preprocess the labels y_i = y_i - y_mean, used for regression
 */
void KernelAlignment::preprocessLabels(std::vector<double> &y) {
  assert(y.size() > 0);
  double sum = 0.0;
  for(unsigned int i = 0; i < y.size(); i++) sum += y[i];
  double mean = sum / y.size();
  for(unsigned int i = 0; i < y.size(); i++) y[i] -= mean;
}

void KernelAlignment::preprocessMatrix(lssvm_matrix &K) {
  // calculate average
  double sum = 0.0;
  for(unsigned int i = 0; i < K.size1(); i++) {
    //for(unsigned int j = 0; j < x.size(); j++) {
    for(unsigned int j = 0; j <= i; j++) {
      sum += K(i, j);
      //sum += eval * eval;
      sum += (j < i) ? 2 * K(i, j) : K(i, j);
    }
  }
  double avg = sum / (K.size1() * K.size2());
  //std::cout << "Average of Matrix: " << avg << std::endl;
  for(unsigned int i = 0; i < K.size1(); i++) {
    //for(unsigned int j = 0; j < x.size(); j++) {
    for(unsigned int j = 0; j <= i; j++) {
      K(i, j) -= avg;
      //sum += eval * eval;
      if(i != j) K(j, i) = K(i, j);
    }
  }
}


double KernelAlignment::getAlignment() {
  return this->alignment;
}


double KernelAlignment::getAlignmentTime() {
  return this->alignmentTime;
}


Kernel& KernelAlignment::getKernel() {
  return *(this->kernel);
}

void KernelAlignment::setKernel(Kernel& kernel) {
  this->kernel = &kernel;
}


double KernelAlignment::getC() {
  return this->c;
}

void KernelAlignment::setPreprocessMatrix(bool matrix_pp) {
  this->matrix_pp = matrix_pp;
}

bool KernelAlignment::getPreprocessMatrix() {
  return this->matrix_pp;
}


// double KernelAlignment::calculate(const std::vector<double> &x, const std::vector<double> &y) {
//   boost::timer* timer = new boost::timer();
//   assert(x.size() == y.size());
// 
//   this->alignment = frobeniusProduct(x, y) / sqrt(frobeniusNorm(x) * frobeniusNorm(y));
// 
//   this->alignmentTime = timer->elapsed();
//   delete(timer);
//   return this->alignment;
// }

// double KernelAlignment::frobeniusProduct(const std::vector<double> &x, const std::vector<double> &y) {
//   assert(x.size() == y.size());
//   assert(this->kernel != 0);
//   double sum = 0.0;
//   double eval;
//   for(unsigned int i = 0; i < x.size(); i++) {
//     for(unsigned int j = 0; j < x.size(); j++) {
//     for(unsigned int j = 0; j <= i; j++) {
//       eval = (x[i] * x[j]) * (y[i] * y[j]);
//       sum += eval;
//       sum += (j < i) ? 2 * eval : eval;
//     }
//   }
//   return sum;
// 
// }

} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
