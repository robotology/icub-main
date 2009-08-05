/* LibLSSVM - Library for Least Squares Support Vector Machine Regression
 *
 * 2006-2007 Arjan Gijsberts, arjan@liralab.it
 *
 * Shared code for lssvm-train and lssvm-predict
 *
 * Mostly adapted code from LibSVM (svm-train.c)
 *
 * Todo:
 *
 */
#ifndef _LSSVM_INC_CPP
#define _LSSVM_INC_CPP

#include "iCub/Problem.h"
#include <iostream>
#include <iomanip>

namespace iCub {
namespace contrib {
namespace liblssvm {

Problem::Problem() {
  this->classification = true;
}

void Problem::readSparseFile(std::string filename) {
  int m, n; // m = no_samples, n = dimensionality input
  int reading_lines, reading_indices;
  double label, value;
  int index;
  int res;
  FILE *fp = fopen(filename.c_str(),"r");

  // no neat exception handling here unfortunately. The code was initially copied from 
  // libsvm, which uses C style file and string operations.
  if(fp == NULL) {
    fprintf(stderr,"can't open input file %s\n",filename.c_str());
    exit(1);
  }

  // determine dimensionality and number of the input
  m = 0;
  n = 0;
  reading_lines = 1;
  while(reading_lines) {
    res = fscanf(fp, "%lf", &label);
    if(res <= 0) {
      printf("Problem reading line %d, aborting\n", m + 1);
      exit(1);
    }
    int c = fgetc(fp);
    reading_indices = 1;
    while(reading_indices) {
      do {
        c = fgetc(fp);
        switch(c) {
          case EOF:
            reading_lines = 0;
            reading_indices = 0;
            break;
          case '\n':
            m++;
            reading_indices = 0;
            break;
        }
      } while(isspace(c));
      ungetc(c, fp);
      if(reading_indices) {
        res = fscanf(fp,"%d:%lf", &index, &value);
        if(res <= 0) {
          printf("Problem reading line %d, aborting\n", m + 1);
          exit(1);
        }
        c = fgetc(fp);
        if(!isspace(c)) {
          printf("Unexpected character '%c' at line %d, aborting\n", c, m + 1);
          exit(1);
        } else {
          ungetc(c, fp);
        }
        //printf("Read %d:%lf", index, value);
        if(index > n) n = index;
      }
    }
    //printf("\n");
  }

  rewind(fp);
  this->x = std::vector< lssvm_vector >(m); // m x n
  this->y = std::vector<double>(m);

  for(int i = 0; i < m; i++) {
    res = fscanf(fp, "%lf", &label);
    if(res <= 0) {
      printf("Problem reading line %d, aborting\n", i + 1);
      exit(1);
    }
    // regression/classification auto-sensing
    if(this->classification && label != 1 && label != -1) {
      this->classification = false;
    }
    y[i] = label;
    x[i].resize(n);

    for(int j = 0; j < n; j++) {
      x[i](j) = 0.0;
    }
    int c = fgetc(fp);
    reading_indices = 1;
    while(reading_indices) {
      do {
        c = fgetc(fp);
        switch(c) {
          case EOF:
            reading_lines = 0;
            reading_indices = 0;
            break;
          case '\n':
            reading_indices = 0;
            break;
        }
      } while(isspace(c));
      ungetc(c,fp);
      if(reading_indices) {
        res = fscanf(fp,"%d:%lf", &index, &value);
        if(res <= 0) {
          printf("Problem reading line %d, aborting\n", i + 1);
          exit(1);
        }
        x[i](index-1) = value;
      }
    }
  }
  fclose(fp);
}

void Problem::writeSparseFile(std::ostream& ostream) {
  assert(this->x.size() == this->y.size());

  //std::ofstream ostream(filename.c_str());
  ostream << std::setprecision(SERIAL_PRECISION);

  for(unsigned int i = 0; i < y.size(); i++) {
    ostream << y[i] << " ";
    for(unsigned int j = 0; j < x[i].size(); j++) {
      if(x[i][j] != 0) {
        ostream << (j + 1) << ":" << x[i][j] << " ";
      }
    }
    ostream << std::endl;

  }
}

std::pair<double, double> Problem::normalizeLabels() {
  double mean, sd;

  mean = 0.0;
  for(unsigned int i = 0; i < y.size(); i++) {
    mean += y[i];
  }
  mean /= y.size();

  sd = 0.0;
  for(unsigned int i = 0; i < y.size(); i++) {
    double diff = mean - y[i];
    sd += diff * diff;
  }
  sd = sqrt(sd / y.size());

  assert(sd != 0);

  for(unsigned int i = 0; i < y.size(); i++) {
    y[i] = (y[i] - mean) / sd;
  }
  
  return std::pair<double, double>(mean, sd);
}

void Problem::scaleLabels(const double lower, const double upper) {
  double min = 0.0; 
  double max = 0.0;
  bool min_set = false;
  bool max_set = false;
  for(unsigned int i = 0; i < y.size(); i++) {
    if(!min_set || y[i] < min) {
      min = y[i];
      min_set = true;
    }
    if(!max_set || y[i] > max) {
      max = y[i];
      max_set = true;
    }
  }
  assert(max != min);

  double factor = (max - min) / (upper - lower);
  for(unsigned int i = 0; i < y.size(); i++) {
    y[i] = lower + ((y[i] - min) / factor);
  }

}

void Problem::normalizeFeatures() {
  std::vector<double> mean(x[0].size());
  std::vector<double> sd(x[0].size());
  for(unsigned int f = 0; f < mean.size(); f++) mean[f] = 0.0;

  for(unsigned int i = 0; i < x.size(); i++) {
    for(unsigned int f = 0; f < mean.size(); f++) mean[f] += x[i][f];
  }
  for(unsigned int f = 0; f < mean.size(); f++) mean[f] /= x.size();

  for(unsigned int f = 0; f < sd.size(); f++) sd[f] = 0.0;
  for(unsigned int i = 0; i < x.size(); i++) {
    for(unsigned int f = 0; f < sd.size(); f++) {
      double diff = mean[f] - x[i][f];
      sd[f] += diff * diff;
    }
  }
  for(unsigned int f = 0; f < sd.size(); f++) sd[f] = sqrt(sd[f] / x.size());

  for(unsigned int i = 0; i < x.size(); i++) {
    for(unsigned int f = 0; f < sd.size(); f++) {
      assert(sd[f] != 0);
      x[i][f] = (x[i][f] - mean[f]) / sd[f];
    }
  }


}

} // namespace liblssvm
} // namespace contrib
} // namespace iCub
#endif
