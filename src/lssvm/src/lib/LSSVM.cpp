/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Main LSSVM implementation file
 *
 * Todo: 
 *
 *
 */
#ifndef _LSSVM_CPP
#define _LSSVM_CPP


#include "iCub/LSSVM.h"
#include <iostream>
#include <iomanip>


namespace iCub {
namespace contrib {
namespace liblssvm {

LSSVM::LSSVM(double c) : c(c) {
  this->trainingTime = -1;
  this->kernelTime = -1;
  this->inversionTime = -1;
  this->predictionTime = -1;
  this->loo = -1;
  this->classification = false;
  this->kernel = (Kernel *) 0;
}

LSSVM::~LSSVM() {
  //delete(this->kernel);
}


void LSSVM::train(const std::vector< lssvm_vector > &x, const std::vector<double> &y) {
  assert(x.size() == y.size());
  assert(x.size() > 0);

  boost::timer* training_timer = new boost::timer();
  boost::timer* matrix_timer = new boost::timer();

  // start training timing
  training_timer->restart();

  // store x
  this->x = x;

  size_t m = this->x.size();

  lssvm_matrix K(m, m);

  // start kernel matrix construction timing
  matrix_timer->restart();
  for (unsigned int i = 0; i < K.size1(); i++) {
    for (unsigned int j = 0; j <= i; j++) {
      K(j, i) = this->getKernel()->eval(this->x[i], this->x[j]);
      // on diagonal add 1/c
      if(i == j) K(i, j) += (1 / this->c);
    }
  }

  //std::cout << "Timing: Kernel Matrix: " << training_timer->elapsed() << std::endl;

  // end kernel matrix construction timing
  this->kernelTime = matrix_timer->elapsed();

  // initialize alphas to size m
  this->alphas = lssvm_vector(m);

  // start kernel inversion timing
  matrix_timer->restart();

  // apply symmetric adapter
  lssvm_sym_matrix SK(K);

  // factorize and compute the inverse of K using cholesky
  atlas::cholesky_factor(SK);
  //std::cout << "Timing: Cholesky Factor: " << training_timer->elapsed() << std::endl;
  atlas::cholesky_invert(SK);
  //std::cout << "Timing: Cholesky Invert: " << training_timer->elapsed() << std::endl;

  // compute gamma and nu using atlas::symm
  lssvm_matrix gamma(m,1);
  lssvm_matrix_col gamma_col(gamma,0);
  std::fill(gamma_col.begin(), gamma_col.end(), 1.0);
  atlas::symm(SK, lssvm_matrix(gamma), gamma);
  //std::cout << "Timing: Gamma Calculation: " << training_timer->elapsed() << std::endl;

  lssvm_matrix nu(m,1);
  lssvm_matrix_col nu_col(nu,0);
  std::copy(y.begin(), y.end(), nu_col.begin());
  atlas::symm(SK, lssvm_matrix(nu), nu);
  //std::cout << "Timing: Nu Calculation: " << training_timer->elapsed() << std::endl;

  // compute gamma and nu using ublas::prod
//   lssvm_vector gamma_col = lssvm_vector(m);
//   for(int i = 0; i < m; i++) gamma_col(i) = 1;
//   gamma_col = ublas::prod(SK, gamma_col);

//   lssvm_vector nu_col = lssvm_vector(m);
//   for(int i = 0; i < m; i++) nu_col(i) = y[i];
//   nu_col = ublas::prod(SK, nu_col);

  // compute bias and alphas 
  this->bias = sum(nu_col) / sum(gamma_col);
  this->alphas = nu_col - (gamma_col * this->bias);

  // end kernel inversion timing
  this->inversionTime = matrix_timer->elapsed();

  // calculate loo: Avg (alpha_i / K_ii)^2, 0 <= i < m
  this->loo = 0.0;
  for(unsigned int i = 0; i < m; i++) {
    double pred_y = y[i] - this->alphas(i) / SK(i, i);
    this->loo += error(y[i], pred_y);
  }
  this->loo /= double(m);

  this->trainingTime = training_timer->elapsed();

  delete(training_timer);
  delete(matrix_timer);
}



double LSSVM::getLOO() {
  return this->loo;
}

void LSSVM::setLOO(double loo) {
  this->loo = loo;
}

double LSSVM::getRealLOO(const std::vector<double> &y) {
  double err = 0.0;
  for(unsigned int i = 0; i < x.size(); i++) {
    LSSVM* tmp_machine = new LSSVM(this->getC());
    tmp_machine->setKernel(this->getKernel());
    std::vector< lssvm_vector > new_x;
    new_x.assign(x.begin(), x.end());
    std::vector<double> new_y;
    new_y.assign(y.begin(), y.end());

    new_x.erase(new_x.begin() + i);
    new_y.erase(new_y.begin() + i);

    tmp_machine->train(new_x, new_y);

    double pred_y = tmp_machine->predict(x[i]);
    err += error(y[i], pred_y);
    delete(tmp_machine);
  }
  return err / x.size();
}

double LSSVM::getKFoldCV(const std::vector< lssvm_vector > &x, const std::vector<double> &y, const int k) {
  double err = 0.0;
  double loo = 0.0;
  double part_size = double(x.size()) / k;

  double begin, end;
  for(unsigned int i = 0; i < k; i++) {
    std::vector< lssvm_vector > fold_train_x, fold_test_x;
    std::vector<double> fold_train_y, fold_test_y;

    begin = i * part_size;
    end = begin + part_size;

    for(unsigned int j = 0; j < x.size(); j++) {
      // check if we are in within the boundaries of the test set of fold k
      if(j >= begin && j < end) {
        fold_test_x.push_back(x[j]);
        fold_test_y.push_back(y[j]);
      } else {
        fold_train_x.push_back(x[j]);
        fold_train_y.push_back(y[j]);
      }
    }
    this->train(fold_train_x, fold_train_y);
    loo += this->getLOO();
    err += this->predict(fold_test_x, fold_test_y);
  }
  this->setLOO(loo / k);
  return err / k;
}

double LSSVM::predict(const lssvm_vector &x_pred) {
  // null prediction if we didn't train on anything
  if(this->x.size() == 0) {
    return 0.;
  }

  assert(x_pred.size() == this->x[0].size());
  double pred = this->bias;
  for(unsigned int i = 0; i < this->alphas.size(); i++) {
    pred += alphas(i) * this->getKernel()->eval(this->x[i], x_pred);
  }
  return (this->classification) ? classify(pred) : pred;
}

std::vector<double> LSSVM::predict(const std::vector< lssvm_vector > &x_pred) {
  boost::timer* timer = new boost::timer();
  timer->restart();
  std::vector<double> y_pred = std::vector<double>(x_pred.size());
  for(unsigned int i = 0; i < x_pred.size(); i++) {
    y_pred[i] = this->predict(x_pred[i]);
  }
  this->predictionTime = timer->elapsed();

  delete(timer);
  return y_pred;
}

double LSSVM::predict(const std::vector< lssvm_vector > &x, const std::vector<double> &y) {
  std::vector<double> pred_y = this->predict(x);
  double err = 0.0;
  for(unsigned int i = 0; i < pred_y.size(); i++) {
    err += error(y[i], pred_y[i]);
  }
  err /= pred_y.size();
  return err;
}


double LSSVM::getTrainingTime() {
  return this->trainingTime;
}

double LSSVM::getKernelTime() {
  return this->kernelTime;
}

double LSSVM::getInversionTime() {
  return this->inversionTime;
}

double LSSVM::getPredictionTime() {
  return this->predictionTime;
}

void LSSVM::saveToFile(const std::string filename) {
  std::ofstream ofs(filename.c_str());
  if(!ofs.good()) {
    throw "Could not open file: " + filename;
  }
  this->save(ofs);
  ofs.close();
}

LSSVM& LSSVM::loadFromFile(const std::string filename) {
  std::ifstream ifs(filename.c_str());
  if(!ifs.good()) {
    throw "Could not open file: " + filename;
  }
  LSSVM* machine = new LSSVM();
  machine->load(ifs);
  ifs.close();
  return *machine;
}

std::string LSSVM::toString() {
  std::ostringstream str_format;
  str_format << "Standard(Cholesky): ";
  str_format << "C=" << this->getC();
  return str_format.str();
}

void LSSVM::save(std::ostream &ostream) {
  // machine_c: c_val
  ostream << std::setprecision(SERIAL_PRECISION);
  ostream << "machine_c: " << this->getC() << std::endl;

  // kernel: kernel_str
  ostream << "kernel: " << this->getKernel()->toString() << std::endl;

  // classification: 0|1
  ostream << "classification: " << this->getClassification() << std::endl;

  // machine_x: m n
  ostream << "machine_x: " << this->x.size() << " " << ((this->x.size() == 0) ? 0 : this->x[0].size()) << std::endl;
  for(unsigned int  i = 0; i < this->x.size(); i++) {
    for(unsigned int  j = 0; j < this->x[i].size(); j++) {
      ostream << this->x[i](j) << " ";
    }
    ostream << std::endl;
  }

  // machine_bias: bias
  ostream << "machine_bias: " << this->bias << std::endl;

  // machine_alphas: m
  ostream << "machine_alphas: " << this->alphas.size() << std::endl;
  for(unsigned int i = 0; i < this->alphas.size(); i++) {
    ostream << this->alphas(i) << std::endl;
  }
}

void LSSVM::load(std::istream &istream) {
  std::string token;
  size_t m, n;
  double value;

  // machine_c: c_val
  istream >> token >> this->c;

  // kernel: kernel_str
  istream >> token >> token;
  this->setKernel(Kernel::fromString(token));

  // classification: 0|1
  istream >> token >> this->classification;

  // machine_x: m n
  istream >> token >> m >> n;
  this->x = std::vector< lssvm_vector >(m); // m x n
  for(unsigned int  i = 0; i < m; i++) {
    this->x[i].resize(n);
    for(unsigned int  j = 0; j < n; j++) {
      istream >> value;
      this->x[i](j) = value;
    }
  }

  // machine_bias: bias
  istream >> token >> value;
  this->bias = value;

  // machine_alphas: m
  istream >> token >> m;
  this->alphas = lssvm_vector(m);
  for(unsigned int  i = 0; i < m; i++) {
    istream >> value;
    this->alphas(i) = value;
  }
}

double LSSVM::getC() {
  return this->c;
}

void LSSVM::setC(double c) {
  if(c < 0.0) c = 0.0;
  this->c = c;
}

Kernel* LSSVM::getKernel() {
  //assert(this->kernel != NULL);
  return this->kernel;
}

void LSSVM::setKernel(Kernel* kernel) {
/*  if(this->kernel != 0) {
    delete(this->kernel);
  }*/
  this->kernel = kernel;
}

bool LSSVM::getClassification() {
  return this->classification;
}

void LSSVM::setClassification(bool c) {
  this->classification = c;
}



/*
 * Reference LSSVM using standard inversion method.
 */
ReferenceLSSVM::ReferenceLSSVM(double c) {
  this->trainingTime = -1;
  this->kernelTime = -1;
  this->inversionTime = -1;
  this->predictionTime = -1;
  this->loo = -1;
  this->c = c;
}

void ReferenceLSSVM::train(const std::vector< lssvm_vector > &x, const std::vector<double> &y) {
  assert(x.size() == y.size());
  assert(x.size() > 0);

  boost::timer* training_timer = new boost::timer();
  boost::timer* matrix_timer = new boost::timer();

  // start training timing
  training_timer->restart();

  // store x locally
  this->x = x;

  size_t m = this->x.size();

  lssvm_matrix K(m + 1, m + 1);

  // start kernel matrix construction timing
  matrix_timer->restart();
  for (unsigned int i = 0; i < K.size1(); i++) {
    for (unsigned int j = 0; j <= i; j++) {
      // if not on edge, evaluate kernel, otherwise 1
      K(i, j) = (i < m) ? this->getKernel()->eval(this->x[i], this->x[j]) : 1;
      // could be improved by using symmetric adapter
      K(j, i) = K(i, j);
      // on diagonal add 1/c
      if(i == j) K(i, j) += (1 / this->c);
    }
  }
  K(m, m) = 0;

  // end kernel matrix construction timing
  this->kernelTime = matrix_timer->elapsed();

  // store [y 0]' in alphas, alphas is reused later for solution
  this->alphas = lssvm_vector(m + 1);
  std::copy(y.begin(), y.end(), this->alphas.begin());
  this->alphas(m) = 0;

  // start kernel inversion timing
  matrix_timer->restart();
  // solve K * [alphas b]' = [y 0]'
  ublas::vector<int> ipiv (m + 1);   // pivot vector
  // factorize and compute the inverse of K
  atlas::lu_factor(K, ipiv);  // alias for getrf()
  atlas::lu_invert(K, ipiv);  // alias for getri()
  // K^-1 * [y 0]' = [alphas b]'
  this->alphas = ublas::prod(K, this->alphas); 

  // split to seperate alphas and bias
  this->bias = this->alphas(m);
  this->alphas.resize(m);

  // end kernel inversion timing
  this->inversionTime = matrix_timer->elapsed();

  // calculate loo: Avg (alpha_i / K_ii)^2, 0 <= i < m
  this->loo = 0.0;
  for(unsigned int i = 0; i < m; i++) {
    double y_dist = this->alphas(i) / K(i, i);
    this->loo += y_dist * y_dist;
  }
  this->loo /= double(m);

  this->trainingTime = training_timer->elapsed();

  delete(training_timer);
  delete(matrix_timer);
}

std::string ReferenceLSSVM::toString() {
  std::ostringstream str_format;
  str_format << "Reference: ";
  str_format << "C=" << this->getC();
  return str_format.str();
}




/*
 * Partial LSSVM that trains on a partial random subset.
 */
PartialLSSVM::PartialLSSVM(double c, double p) : p(p) {
  this->trainingTime = -1;
  this->kernelTime = -1;
  this->inversionTime = -1;
  this->predictionTime = -1;
  this->loo = -1;
  this->c = c; // apparently attributed need to be accessed here
}

void PartialLSSVM::train(const std::vector< lssvm_vector > &x, const std::vector<double> &y) {
  assert(this->p > 0 && this->p <= 1.0);

  this->train(x, y, int(x.size() * this->p + 0.5));
}

void PartialLSSVM::train(const std::vector< lssvm_vector > &x, const std::vector<double> &y, int subsetSize) {
  assert(x.size() == y.size());
  assert(subsetSize > 0 && subsetSize <= x.size());
  assert(x.size() > 0);

  boost::timer* training_timer = new boost::timer();
  boost::timer* matrix_timer = new boost::timer();

  // start training timing
  training_timer->restart();

  // NB: Here M is size of subset, L is size of trainingset
  size_t l = x.size();
  size_t m = subsetSize;
  total_x.assign(x.begin(), x.end());
  // create pruned index set for M
  srand(std::time(NULL));
  std::vector<lssvm_vector> subset(x);
  // remove line below (i.e. resize) to enable shuffling
  subset.resize(m);
  while(subset.size() > m) {
    // rand() % size is bad... but the randomness is not too important
    subset.erase(subset.begin() + rand() % subset.size());
  }

  //std::cout << "Subset size: " << subset.size() << std::endl;
  // start kernel matrix construction timing
  matrix_timer->restart();

  // construct K_ML
  lssvm_matrix KML(m + 1, l);
  for (unsigned int i = 0; i < KML.size1(); i++) {
    for (unsigned int j = 0; j < KML.size2(); j++) {
      KML(i, j) = (i < m) ? this->getKernel()->eval(subset[i], x[j]) : 1;
    }
  }

  // store x (i.e. the subset that is used) locally
  this->x = subset;

  // construct K_MM
  lssvm_matrix KMM(m + 1, m + 1);
  for (unsigned int i = 0; i < KMM.size1(); i++) {
    for (unsigned int j = 0; j <= i; j++) {
      KMM(i, j) = (i < m) ? this->getKernel()->eval(subset[i], subset[j]) : 0;
      KMM(j, i) = KMM(i, j);
    }
  }
  KMM(m, m) = 0;

  // end kernel matrix construction timing
  this->kernelTime = matrix_timer->elapsed();

  // construct K
  lssvm_matrix K = lssvm_matrix(m + 1, m + 1);
  lssvm_matrix KLM = trans(KML);
  atlas::gemm(KML, KLM, K);
  // VERIFY THIS ISSUE, THE FIRST ALTERNATIVE SEEMS BEST
  //K += (KMM + (ublas::identity_matrix<double>(m + 1) * 0.0001)) * (1/this->c);
  K += (KMM + ublas::identity_matrix<double>(m + 1)) * (1/this->c);
  //K += (KMM) * (1/this->c);

  // store [y 0]' in alphas, alphas is reused later for solution
  this->alphas = lssvm_vector(l);
  std::copy(y.begin(), y.end(), this->alphas.begin());

  // start kernel inversion timing
  matrix_timer->restart();

  /*
   *  ALTERNATIVE 1
   */

//   lssvm_matrix tmp(m + 1,1);
//   lssvm_matrix_col tmp_col(tmp,0);
//   tmp_col = ublas::prod(KML, this->alphas);
//   ublas::vector<int> ipiv (m + 1);   // pivot vector
//
//   // solve alphas = K * tmp
//   atlas::gesv(K, ipiv, tmp);
//   this->alphas = lssvm_matrix_col(tmp,0);
//
//   int invert = atlas::lu_invert(K, ipiv);  // alias for getri()

  /*
   *  ALTERNATIVE 2
   */
//   ublas::vector<int> ipiv (m + 1);   // pivot vector
//   int factor = atlas::lu_factor(K, ipiv);  // alias for getrf()
//   int invert = atlas::lu_invert(K, ipiv);  // alias for getri()
//
//   lssvm_vector tmp = ublas::zero_vector<double>(m + 1);
//   tmp = ublas::prod(KML, this->alphas);
//   this->alphas = ublas::prod(K, tmp); 

  /*
   *  ALTERNATIVE 3
   */
  ublas::vector<int> ipiv (m + 1);   // pivot vector
  int factor = atlas::lu_factor(K, ipiv);  // alias for getrf()
  int invert = atlas::lu_invert(K, ipiv);  // alias for getri()

  lssvm_vector tmp = ublas::zero_vector<double>(m + 1);
  atlas::gemv(KML, this->alphas, tmp);
  atlas::gemv(K, tmp, this->alphas);


  // split to seperate alphas and bias
  this->bias = this->alphas(m);
  this->alphas.resize(m);

  this->inversionTime = matrix_timer->elapsed();

  // calculate loo: Avg (alpha_i / K_ii)^2, 0 <= i < l


  lssvm_matrix tmp_H = lssvm_matrix(l, m + 1);
  atlas::gemm(KLM, K, tmp_H);
  //std::cout << "Matrix time: " << matrix_timer->elapsed() << std::endl;
  lssvm_matrix H = lssvm_matrix(l, l);
  atlas::gemm(tmp_H, KML, H);
  //std::cout << "Matrix time: " << matrix_timer->elapsed() << std::endl;

  // end kernel inversion timing


  this->loo = 0.0;
  for(unsigned int i = 0; i < l; i++) {
    double e = y[i] - this->predict(this->total_x[i]);
    double y_dist = e / (1 - H(i, i));
    this->loo += y_dist * y_dist;
  }
  this->loo /= double(l);


  this->trainingTime = training_timer->elapsed();

  delete(training_timer);
  delete(matrix_timer);
}

/* QUICK FIX */
double PartialLSSVM::getRealLOO(const std::vector<double> &y) {
  double err = 0.0;
  for(unsigned int i = 0; i < total_x.size(); i++) {
    PartialLSSVM* tmp_machine = new PartialLSSVM(this->getC(), this->getP());
    tmp_machine->setKernel(this->getKernel());
    std::vector< lssvm_vector > new_x;
    new_x.assign(total_x.begin(), total_x.end());
    std::vector<double> new_y;
    new_y.assign(y.begin(), y.end());

    new_x.erase(new_x.begin() + i);
    new_y.erase(new_y.begin() + i);

    tmp_machine->train(new_x, new_y, x.size() - 1);

    double pred_y = tmp_machine->predict(total_x[i]);
    err += error(y[i], pred_y);
    delete(tmp_machine);
  }
  return err / total_x.size();
}


std::string PartialLSSVM::toString() {
  std::ostringstream str_format;
  str_format << "Partial: ";
  str_format << "C=" << this->getC() << "; ";
  str_format << "P=" << this->getP();
  return str_format.str();
}

double PartialLSSVM::getP() {
  return this->p;
}

void PartialLSSVM::setP(double p) {
  if(p < 0) p = SMALL_CONSTANT;
  if(p > 1.0) p = 1.0;
  this->p = p;
}





  /*
   *  LEGACY
   */

  // factorize and compute the inverse of K (extension: use Cholesky also here)
//  ublas::vector<int> ipiv (m + 1);   // pivot vector
//  int factor = atlas::lu_factor(K, ipiv);  // alias for getrf()
  //std::cout << "Factor result: " << factor << std::endl;

  //this->alphas = ublas::zero_vector<double>(m + 1);

//  lssvm_matrix tmp(m + 1,1);
//  lssvm_matrix_col tmp_col(tmp,0);
//  tmp_col = ublas::prod(KML, this->alphas);
  //atlas::lu_substitute(K, ipiv, tmp);
  //this->alphas = lssvm_matrix_col(tmp,0);

  //lssvm_matrix_col gamma_col(gamma,0);
  //std::fill(gamma_col.begin(), gamma_col.end(), 1.0);
  //atlas::symm(SK, lssvm_matrix(gamma), gamma);
//  atlas::lu_solve(K, tmp);
//  this->alphas = lssvm_matrix_col(tmp,0);


  /*
   * EXPERIMENTAL IMPLEMENTATION USING EXPLICIT INVERSION 
   */

//  int invert = atlas::lu_invert(K, ipiv);  // alias for getri()
  //std::cout << "Invert result: " << invert << std::endl;

//  lssvm_vector tmp = ublas::zero_vector<double>(m + 1);
//  tmp = ublas::prod(KML, this->alphas);
//  this->alphas = ublas::prod(K, tmp); 


  //******** BUGGY CODE? **********
//   lssvm_vector intermed = ublas::zero_vector<double>(m + 1);
//   atlas::gemv(KML, this->alphas, intermed);
//   this->alphas = ublas::zero_vector<double>(m + 1);
//   atlas::gemv(K, intermed, this->alphas);

  //K = ublas::prod(K, KML);
  //this->alphas = ublas::prod(K, this->alphas); 



} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
