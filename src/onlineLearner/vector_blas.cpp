/*
 * Program to evaluate using various BLAS implementations with STL vectors.
 *
 * (c) 2009, Arjan Gijsberts @ Italian Institute of Technology
 *
 */

#include <vector>
#include <iostream>
#include <sstream>
#include <time.h>
#include <cassert>


#ifdef GSLBLAS
#include <gsl/gsl_cblas.h>
#endif

#ifdef ATLAS
extern "C" {
  #include <cblas.h>
}
#endif

#ifdef REFBLAS
extern "C" {
  #include <cblas.h>
}
#endif

#ifdef SCALAPACK

#endif

#define VECSIZE 10000000
#define REPETITIONS 500

static double get_time() {
  struct timespec time;
  clock_gettime(CLOCK_REALTIME, &time);
  return (double) (time.tv_sec + ((double) time.tv_nsec / 1.0E9));
}

typedef std::vector<double> vec;

static std::string print_vector(const vec* v) {
  std::ostringstream output;
  output << "[";
  for(int i = 0; i < v->size(); i++) {
    if(i > 0) output << ",";
    output << (*v)[i];
  }
  output << "]";
  return output.str();
}

static double vector_dot(vec* v1, vec* v2) {
  assert(v1->size() == v2->size());
  //return cblas_ddot(v1->size(), (double*) &(*v1)[0], 1, (double*) &(*v2)[0], 1);
  return cblas_ddot(v1->size(), (double*) &v1->at(0), 1, (double*) &v2->at(0), 1);
}


vec* get_vector(int len) {
  vec* v = new vec(len);
  for(int i = 0; i < len; i++) {
    (*v)[i] = i;
  }
  return v;
}


int main(int argc, char* argv[]) {
  std::cout << "BLAS test" << std::endl;
  double time_start, time_end;
  double dot;
  
  //time_start = get_time();
  vec* v1 = get_vector(VECSIZE);
  //time_end = get_time();
  //std::cout << "V1 creation: " << (time_end - time_start) << " seconds" << std::endl;
  
  vec* v2 = get_vector(VECSIZE);
  
  //std::cout << "v1:" << print_vector(v1) << std::endl;
  //std::cout << "v2:" << print_vector(v2) << std::endl;
  
  std::cout << "v1.size(): " << v1->size() << std::endl;
  
  time_start = get_time();
  for(int i = 0; i < REPETITIONS; i++) {
    dot = vector_dot(v1, v2);
  }
  time_end = get_time();
  std::cout << "dot(v1, v2) = " << dot << " : " << (time_end - time_start) << " seconds" << std::endl;

    
  
  delete v1;
  delete v2;
}
