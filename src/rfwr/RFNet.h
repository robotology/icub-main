#ifndef RFNET1_H
#define RFNET1_H

#include <YARPMath.h>
#include <string>
#include <iostream>
#include <vector>
#include "Rf.h"
#include "RFUtils.h"
#include "RFStruttura.h" //+


class RFNet 
{
 public:
  RFNet(void);
  RFNet(int n_in,int n_out, 
	bool diag_only, 
	bool meta, double meta_rate, 
	double penalty, 
	double init_alpha, const YVector &norm, const YVector &norm_out);

  RFNet(const RFNet &net);

  void print() const;
  void print(std::ofstream &  file) const;
  void Init (int n_in,int n_out, 
	     bool diag_only, 
	     bool meta, 
	     double meta_rate,	
	     double penalty,	
	     double init_alpha,
	     const YVector &norm,
	     const YVector &norm_out);

  void SetWGen(double w_gen);
  void SetWPrune(double w_prune);
  void SetMeta(bool m);
  void SetMetaRate(double mr);
  // Per settare le matrici bisogna indicarne le due dimensioni ed il vettore 
  // degli elementi, oppure, stando attenti alle dimensioni, indicare la matrice da copiare.
	
  void SetInitD(int d1, int d2, double * elem);
  void SetInitD(const YMatrix & m);
  void SetInitAlpha(int d1, int d2, double * elem);
  void SetInitAlpha(const YMatrix & m);
  int GetNData(void) { return n_data; }
  int GetNRfs(void) { return rfs.size();}

  double Train(const YVector &x, const YVector &y, bool composite_control = 0, double *e = 0, int alpha = 0);
  double Train(const YVector &x, const YVector &y, YVector &yp, bool composite_control = 0, double *e = 0, int alpha = 0);
	
  double Simulate (const YVector &x, double cutoff, YVector & yp);
  void Jacobian(const YVector &x, double cutoff, YMatrix &J);
	
  void SaveNet(const char * file_ini);
  int LoadNet(const char * file_ini, bool only_init=false); 

  int getInSize()
    { return n_in; }
  int getOutSize()
    { return n_out; }

 private:
  void _default(void);
	
  // Parameters
  int n_in;
  int n_out;
  bool diag_only;
  bool meta;
  double meta_rate;
  double penalty;
  double init_alpha_num;
  YVector norm;
  YVector norm_out;
    
  // add additional convenient variables
  int n_data; 
  double w_gen ;
  double w_prune ;
  double init_lambda ;
  double final_lambda ;
  double tau_lambda  ;
  double init_P;
  int n_pruned; 
      
  // other variables
  YMatrix   init_D;     
  YMatrix   init_M;      
  YMatrix   init_alpha;
  YVector   mean_x ;
  YVector   var_x ;
  YVector   _tempX; // temp vector
  YVector   _tempY; // temp vector
	
  std::vector <Rf *> rfs;
  std::string  kernel;
      
  // nuove variabili necessarie per il funzionamento della rete,
  // che vengono reinizializzate ad ogni Train

  YVector xn;
  YVector yn;
  YVector tms;
  YVector xmzp;
  YVector yp_i;

  YVector prediction;

};

#endif
