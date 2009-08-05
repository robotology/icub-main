/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Abstract Kernel implementation file
 *
 * Note:
 *
 * Todo:
 *
 */
#ifndef _KERNEL_CPP
#define _KERNEL_CPP

#include "iCub/Kernel.h"

namespace iCub {
namespace contrib {
namespace liblssvm {

lssvm_matrix Kernel::getMatrix(const std::vector< lssvm_vector > &x) {
  return this->getMatrix(x, 0);
}

lssvm_matrix Kernel::getMatrix(const std::vector< lssvm_vector > &x, const double c) {
  assert(c != 0);
  lssvm_matrix K = lssvm_matrix(x.size(), x.size());
  for (unsigned int i = 0; i < K.size1(); i++) {
    for (unsigned int j = 0; j <= i; j++) {
      K(i, j) = this->eval(x[i], x[j]) + (1 / c);
      if(i != j) K(j, i) = K(i, j);
    }
  }
  return K;
}

Kernel* Kernel::fromString(std::string str) {
  char config[64] = "";
  int kernel_id;
  Kernel* kernel;
  // split string in kernel id and config, delimiter is ':'
  sscanf(str.c_str(), "%d:%s", &kernel_id, config);
  // create instance of kernel specified by id using KernelFactory
  kernel = KernelFactory::instance().create(kernel_id);
  // read config dynamically
  kernel->loadConfig(config);
  return kernel;
}

std::string Kernel::toString() {
  std::ostringstream str_format;
  str_format << getKernelID() << ":" << this->saveConfig();
  return str_format.str();
}

void Kernel::loadConfig(std::string const& str) {
  std::vector<std::string> str_tokens;
  // tokenize params on delimiter ','
  Kernel::tokenize(str, ",", str_tokens);
  // uncomment to issue warning instead of assertion
/*  if(str_tokens.size() > this->noParams) {
    std::cout << "Warning: found too many kernel parameters" << std::endl;
    str_tokens.resize(this->noParams);
  }*/
  assert(str_tokens.size() <= this->noParams);
  for(unsigned i = 0; i < str_tokens.size(); i++) {
    this->setParam(i, atof(str_tokens[i].c_str()));
    //this->params[i] = atof(str_tokens[i].c_str());
  }
}

std::string Kernel::saveConfig() {
  std::ostringstream str_format;
  // append parameters to string, comma separated
  for(unsigned i = 0; i < this->params.size(); i++) {
    if(i > 0) str_format << ",";
    str_format << this->getParam(i);
  }
  return str_format.str();
}

param_array Kernel::getParams() {
  return this->params;
}

void Kernel::setParams(param_array p) {
  assert(p.size() == this->getNoParams());
  this->params = p;
  this->sanitizeParams();
}

double Kernel::getParam(unsigned int index) {
  assert(index >= 0 && index < this->params.size());
  return this->params[index];
}

void Kernel::setParam(unsigned int index, double val) {
  assert(index >= 0 && index < this->params.size());
  this->params[index] = val;
  this->sanitizeParams();
}


size_t Kernel::getNoParams() {
  return this->noParams;
}

void Kernel::setNoParams(size_t pc) {
  this->noParams = pc;
}


double Kernel::dot_product(const lssvm_vector &v1, const lssvm_vector &v2) {
  assert(v1.size() == v2.size());
  return atlas::dot(v1, v2);
}

double Kernel::squared_distance(const lssvm_vector &v1, const lssvm_vector &v2) {
  assert(v1.size() == v2.size());
  lssvm_vector v3 = v1 - v2;
  return dot_product(v3, v3);
}

void Kernel::tokenize(const std::string& str, const std::string& delimiters, 
                      std::vector<std::string>& tokens) {
  // skip delimiters add beginning and move to first non-delimiter
  std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
  std::string::size_type pos = str.find_first_of(delimiters, lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos) {
    // add token to vector
    tokens.push_back(str.substr(lastPos, pos - lastPos));
    // skip delimiters
    lastPos = str.find_first_not_of(delimiters, pos);
    // find first next non-delimiter
    pos = str.find_first_of(delimiters, lastPos);
  }
}


// KernelFactory

KernelFactory& KernelFactory::instance() {
  static KernelFactory inst;
  return inst;
}

bool KernelFactory::regCreateFn(kernel_id_type key, kernel_creator func) {
  assert(this->kernel_map.find(key) == this->kernel_map.end());
  kernel_map[key] = func;
  return true;
}

Kernel* KernelFactory::create(kernel_id_type key) const {
  assert(this->kernel_map.find(key) != this->kernel_map.end());
  return this->kernel_map.find(key)->second();
}

} // namespace liblssvm
} // namespace contrib
} // namespace iCub

#endif
