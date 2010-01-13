/*
  Copyright (c) 2008 Florent D'halluin , Sylvain Calinon, Eric Sauser
  LASA Lab, EPFL, CH-1015 Lausanne, Switzerland, 
  http://www.calinon.ch, http://lasa.epfl.ch

  The program is free for non-commercial academic use. 
  Please acknowledge the authors in any academic publications that have 
  made use of this code or part of it. Please use this BibTex reference: 
 
  @article{Calinon07SMC,
  title="On Learning, Representing and Generalizing a Task in a Humanoid 
  Robot",
  author="S. Calinon and F. Guenter and A. Billard",
  journal="IEEE Transactions on Systems, Man and Cybernetics, Part B. 
  Special issue on robot learning by observation, demonstration and 
  imitation",
  year="2007",
  volume="37",
  number="2",
  pages="286--298"
  }
*/

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include "float.h"

#include "GMR.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif


bool GaussianMixture::saveMuAndSigma(const char fileMu[], const char fileSigma[], 
                     Matrix outData, Matrix outSigma[])
{
  // Save the result of a regression
  std::ofstream Mu_file(fileMu); // regressed data
  std::ofstream Sigma_file(fileSigma); // covariances matrices
  for(unsigned int i=0;i<outData.RowSize();i++) {
    for(unsigned int j=0;j<outData.ColumnSize();j++) 
      Mu_file << outData(i,j) << " ";
    Mu_file << std::endl;
    for(unsigned int k=0;k<outData.ColumnSize();k++) {
      for(unsigned int j=0;j<outData.ColumnSize();j++)
    Sigma_file << outSigma[i](k,j) << " ";
    }
    Sigma_file << std::endl;
  }
  return 1;
}

bool GaussianMixture::loadParams(const char fileName[])
{
  // Load coefficient of a GMM from a file (stored by saveParams Method
  // or with matlab
  std::ifstream fich(fileName);
  if (!fich.is_open())
    return false;
  fich >> dim >> nState;
  priors.Resize(nState);
  for (int s = 0; s < nState; s++)
  {
    fich >> priors[s];
  }
  mu.Resize(nState,dim);
  for (int i = 0; i < nState; i++)
  {
    for (int j = 0;j<dim;j++)
      fich >> mu(i,j);
  }
  if(sigma!=NULL) delete [] sigma;
  sigma = new Matrix[nState];
  for (int s = 0; s < nState; s++)
  {
    sigma[s] = Matrix(dim,dim);
    for (int i = 0; i < dim; i++)
    {
      for (int j = 0;j<dim;j++)
    fich >> sigma[s](i,j);
    }
  }
  invSigma = new Matrix[nState];
  detSigma = new double[nState];
  
  return true;
}

void GaussianMixture::debug(void)
{
  /*display on std output info about current parameters */
  Vector v;
  Matrix smat;
  std::cout << "Nb state : "<< this->nState <<std::endl;
  std::cout << "Dimensions : "<< this->dim  <<std::endl;
  std::cout << "Priors : ";
  for(int i = 0;i<nState;i++) {
    std::cout << priors[i] ;
  }
  std::cout << std::endl;
  std::cout << "Means :";
  mu.Print();
  std::cout << "Covariance Matrices :";
  v=Vector(2);
  v(0)=0;
  v(1)=2;
  for(int i =0;i<nState;i++) {
    //double det;
    //Matrix inv;
    //sigma[i].GetMatrixSpace(v,v,inv);
    //inv.Print();
    //std::cout << det << std::endl;
    sigma[i].Print();
  }
}

void GaussianMixture::saveParams(const char filename [])
{
  // save the current GMM parameters, coefficents to a file, to be retrieved 
  // by the loadParams method
  std::ofstream file(filename);
  file << dim << " ";
  file << nState << std::endl;
  for(int i=0;i<nState;i++) file << priors[i] << " ";
  file << std::endl;
  for(int s=0;s<nState;s++) {
    for(int i=0;i<dim;i++) {
      file << mu(s,i) << " ";
    }
    file << std::endl;
  }
  for(int s=0;s<nState;s++) {
    for(int j=0;j<dim;j++) {
      for(int i=0;i<dim;i++) {
    file << sigma[s](i,j) << " ";
      }
      file << std::endl;
    }
  }
}


void GaussianMixture::initEM_random(int nState,Matrix DataSet)
{
  Vector * mean = new Vector[nState];
  int nData = DataSet.RowSize();
  this->nState = nState;
  this->dim = DataSet.ColumnSize();
  Matrix index(nState,nData);
  int * pop = new int[nState];
  priors.Resize(nState);
  mu.Resize(nState,dim);
  sigma = new Matrix[nState];
  invSigma = new Matrix[nState];
  detSigma = new double[nState];

  srand(time(0));

  Matrix unity(dim,dim); /* defining unity matrix */
  for(int k = 0;k<dim;k++) unity(k,k)=1.0;

  for(int s=0;s<nState;s++) /* clearing values */
    {
      mean[s].Resize(dim,true);
      pop[s]=0;
    }

  for(int n = 0;n<nData;n++) /* attribute each input vector to a state */
    {
      int s = rand()%nState;
      mean[s] += DataSet.GetRow(n);
      index(s,pop[s]) = n;
      pop[s] +=1;
    }
  
  for(int s =0;s<nState;s++)
    {
      mu.SetRow(mean[s]/pop[s],s); /* initiate the means computed before */
      sigma[s]=Matrix(dim,dim);
      priors[s]=1.0/(double)nState; /* set equi-probables states */
      for(int ind=0;ind<pop[s];ind++)
    {
      for(int i=0;i<dim;i++) /* Computing covariance matrices */
      {
      for(int j=0;j<dim;j++)
        {
          sigma[s](i,j) += (DataSet((int)index(s,ind),i) - mu(s,i)) \
        *(DataSet((int)index(s,ind),j)-mu(s,j));
        }
      }
    }
      sigma[s] *= 1.0/pop[s];
      sigma[s] += unity*1e-5; /* prevents this matrix from being non-invertible */
    }
  std::cerr << "== gmr >> Random initialisation ok" << std::endl;
  delete [] mean;
  delete [] pop;
}

void GaussianMixture::initEM_kmeans(int nState,Matrix Dataset)
{
  this->nState = nState;
  this->dim = Dataset.ColumnSize();
  sigma = new Matrix[nState];
  invSigma = new Matrix[nState];
  detSigma = new double[nState];
  priors.Resize(nState);
  mu.Resize(nState,dim);
  int * clusters = new int[Dataset.RowSize()];

  Matrix unity(dim,dim); /* defining unity matrix */
  for(int k = 0;k<dim;k++) unity(k,k)=1.0;
  for(int i=0;i<nState;i++)
    {
      int idx = i * Dataset.RowSize() / nState;
      mu.SetRow(Dataset.GetRow(idx),i);
    }
  bool changed = true;
  int iter_count = 0;
  while(changed and iter_count < 500)
    {
      iter_count++;
      changed = false;
      // check wich cluster each vector belongs to
      for(int i = 0;i<Dataset.RowSize();i++)
    {
      Vector current = Dataset.GetRow(i);
      double minDistance = DBL_MAX;
      int prev_clust = clusters[i];
      for(int clust = 0;clust<nState;clust++)
        {
          Vector _dist = current - mu.GetRow(clust);
          double currentDistance = _dist.Norm();
          if(currentDistance < minDistance)
        {
          minDistance = currentDistance;
          clusters[i] = clust;
        }
        }
      if(clusters[i] != prev_clust)
        changed=true;
    }
      // update means 
      for(int clust = 0;clust<nState;clust++)
    {
      Vector mean(dim);
      int count = 0;
      for(int i=0;i<Dataset.RowSize();i++)
        {
          if(clusters[i] == clust)
        {
          mean += Dataset.GetRow(i);
          count++;
        }
        }
      mu.SetRow(mean * (1.0f/count),clust);
    }
    }
  std::cerr << iter_count << "  kmeans iterations" << std::endl;
  // computing variance on each cluster .. 
  for(int clust = 0;clust<nState;clust++)
    {
      priors[clust] = 1.0f/nState;
      int count = 0;
      sigma[clust] = Matrix(dim,dim);
      for(int k =0 ;k<Dataset.RowSize();k++)
    {
      if(clusters[k] == clust)
        { 
          count++;
          for(int i=0;i<dim;i++) /* Computing covariance matrices */
        {
          for(int j=0;j<dim;j++)
            {
              sigma[clust](i,j) += (Dataset(k,i) - mu(clust,i))    \
            *(Dataset(k,j)-mu(clust,j));
            }
        }
        }
    }
      sigma[clust] *= (1.0f/count);
      sigma[clust] += unity*1e-6;    
     
    }           // too many nested loops to be truly honest .. a better idea ? 
  // inverseSigmaMatrices();
  delete [] clusters;
}


void GaussianMixture::initEM_TimeSplit(int nState,Matrix DataSet, int offCount)
{
  Vector * mean = new Vector[nState];
  int nData = DataSet.RowSize();
  this->nState = nState;
  this->dim = DataSet.ColumnSize();
  Matrix * var  = new Matrix[nState];
  for(int i=0;i<nState;i++){ var[i].Resize(dim,dim,false); var[i].Zero();}
  double tmax = 0;
  Matrix index(nState,nData);
  int * pop = new int[nState];
  priors.Resize(nState);
  mu.Resize(nState,dim);
  sigma = new Matrix[nState];
  invSigma = new Matrix[nState];
  detSigma = new double[nState];

  offCount = MAX(1,offCount);

  Matrix unity(dim,dim); /* defining unity matrix */
  for(int k = 0;k<dim;k++) unity(k,k)=1.0;

  for(int n = 0;n<nData;n++) /* getting the max value for time */
    {
      if(DataSet(n,0) > tmax) tmax = DataSet(n,0); 
    }
  for(int s=0;s<nState;s++) /* clearing values */
    {
      mean[s].Resize(dim,true);
      pop[s]=0;
    }

  /* divide the dataset into slices of equal time
       (tmax/nState) and compute the mean for each slice
       the pop table index to wich slice belongs each sample */
    for(int n = 0;n<nData;n++){
      int s = (int)((DataSet(n,0)/tmax)*(nState/offCount));
      if(s==nState) s= nState-1; /* this prevent an ugly segfault if t=tmax*/
      mean[s] += DataSet.GetRow(n);
      index(s,pop[s]) = n;
      pop[s] +=1;
    }
    for (int i=0;i<nState;i++){
        mean[i] /= pop[i];   
    }
    for(int n = 0;n<nData;n++){ 
      int s = (int)((DataSet(n,0)/tmax)*(nState/offCount));
      if(s==nState) s= nState-1; /* this prevent an ugly segfault if t=tmax*/
      Vector tmp;
      Matrix tmp2;
      DataSet.GetRow(n).Sub(mean[s],tmp);
      tmp.MultTranspose(tmp,tmp2);
      var[s] += tmp2;
    }
    for (int i=0;i<nState;i++){
        var[i] /= pop[i];   
    }
  
    if(offCount>1){
        
        
    }
    for(int s =0;s<nState;s++){
        //mean[s].Print();
        priors[s]=1.0/(double)nState; 
        sigma[s]  = var[s];
        sigma[s] += unity*1e-5;
        mu.SetRow(mean[s],s);
        //sigma[s].Print(); 
    }
  /*
  for(int s =0;s<nState;s++)
    {
      mu.SetRow(mean[s]/pop[s],s); 
      sigma[s]=Matrix(dim,dim);
      priors[s]=1.0/(double)nState; 
      for(int ind=0;ind<pop[s];ind++)
    {
      for(int i=0;i<dim;i++) /
      {
      for(int j=0;j<dim;j++)
        {
          sigma[s](i,j) += (DataSet((int)index(s,ind),i) - mu(s,i)) \
        *(DataSet((int)index(s,ind),j)-mu(s,j));
        }
      }
    }
      sigma[s] *= 1.0/pop[s];
      sigma[s] += unity*1e-5; 
    }*/
  std::cerr << "== gmr >> time split initialisation ok" << std::endl;
  delete [] mean;
  delete [] pop;
  delete [] var;
  //exit(0);
}

double GaussianMixture::stepEM(Matrix DataSet, bool update_covariances)
{
  int nData = DataSet.RowSize();
  double log_likelihood = 0.; 
  Vector sum_p(nData);
  Matrix pxi(nData,nState);
  Matrix pix(nData,nState);
  Vector E;

  // temporary variables for computation purposes ..
  Matrix tmptmu(nData,dim);
  Vector tmu(dim);
  


  // before Expectation, lets inverse the covariances matrices 
  if(inverseSigmaMatrices() < 1)
    {
      std::cout << "can't inverse covariances matrices" <<std::endl;
      return 0.;
    }
  /** Expectation Computing **/ 
  for(int i =0;i<nData;i++)
  {
     for(int j=0;j<nState;j++)
     { 
       double p = pdfState(DataSet.GetRow(i),j);  // P(x|i)
      
       // checking for numerical problems .. 
       if(p==0) {
           std::cerr << "Error: Null probability." << std::endl;
           return 0.;
       }
       if(isnan(p)) 
       {
          std::cerr << "Error: NAN probability." << std::endl;
          return 0.;
       }
       if(p<DBL_MIN)
           p = DBL_MIN;
       pxi(i,j)= p;
     }   
  } 
  sum_p = pxi * priors; // sum_p[i] = sum_i ( p(x|i) priors[i] )
     
  for(int i=0;i<nData;i++) 
    log_likelihood += log(sum_p[i]);

  for(int j=0;j<nState;j++)
  {
    for(int i=0;i<nData;i++)
    {
      pix(i,j) = pxi(i,j)*priors(j)/sum_p[i]; // then P(i|x) 
    }   
  } 
 
  /** Update Step **/
  pix.SumRow(E);
  for(int j=0;j<nState;j++) 
  {
    priors(j)=E(j)/nData; // new priors 
      
    // Preventing it from becoming totally null ..
    if(priors(j) == 0.0) 
    {
      priors[j]= 1e-5;
      std::cerr << "Numerical instability :: null priors ..." << std::endl;
      std::cerr << "please considers decreasing states numbers"<<std::endl;
    }
     
    DataSet.MultTranspose(pix.GetColumn(j),tmu); // new mean for state j
    // tmu = Dataset^T * pix.Column(j) 
    // ou encore .. tmu = sum_x ( x * p(x| j) ) pour tout les x du Dataset
      
    tmu = tmu / E(j);
    mu.SetRow(tmu,j);

    if(update_covariances) 
    {
          // (Dataset - tmu)^T * ((Dataset - tm) .* pix.getRow(j))

       for(int i=0;i<nData;i++)
       {
          tmptmu.SetRow(tmu,i);
       }
          // sigma[j] = sum _ x ( (x-mu_j)^T (x-m_j) * p(x|j) ) 

       tmptmu = DataSet - tmptmu;  // (x-mu_j)
       tmptmu.MultTranspose(tmptmu ^ pix.GetColumn(j) ,sigma[j]);
          // sigma_j = tmptmu ^T * tmptmu * p(x|j) 
          
       sigma[j] = sigma[j]/E(j);
     }
  }
  return log_likelihood/nData; 
}


void GaussianMixture::doEM(Matrix DataSet,int blank_run,int max_iter)
{
  /* perform Expectation/Maximization on the given Dataset :
     Matrix DataSet(nSamples,Dimensions).
     The GaussianMixture Object must be initialised before 
     (see initEM_TimeSplit method ) */
  int nData = DataSet.RowSize();  
  int iter = 0;
  double log_lik;
  double log_lik_threshold = EM_STOPPING_CRIT;
  double log_lik_old=-1e10;

  Matrix var_keep(dim,dim);
  Vector mean(dim);
  Vector var(dim);
  int success_iter = 0;
 // computing Dataset variance :
 
  for(int i = 0;i<nData;i++)
    {
      mean += DataSet.GetRow(i);
    }
  mean = mean/nData;
  
  for(int i = 0;i<nData;i++)
    {
      for(int j =0 ; j<dim;j++)
      {
      var(j) += pow(DataSet(i,j)-mean(j),2);
        }
    }
  var = var/nData;
 
  for(int k = 0;k<dim;k++) var_keep(k,k)=1/sqrt(var(k));
 
  while( iter < (max_iter+blank_run)) //EM loop
    {
      iter++;
      if(iter < blank_run)
    log_lik = stepEM(DataSet,false);
      else
    log_lik = stepEM(DataSet,true);
      for(int i=0;i<nState;i++)
    sigma[i] += var_keep*VARIANCE_KEEPER;

      if(log_lik != 0.)
    success_iter += 1;

      else
    {
      //for(int i=0;i<nState;i++)
      //  sigma[i] += var_keep;  // boosting covariances .. 
    }
      // std::cout << std::setprecision(10) <<  log_lik << std::endl;

      if(fabs((log_lik/log_lik_old)-1) < log_lik_threshold )
    {
      /* if log likehood hasn't move enough, the algorithm has
         converged, exiting the loop */
      break;
    }
      log_lik_old = log_lik;
      cout << log_lik<<endl;
    }

  std::cerr << "(EM finished with "<<success_iter<<" iterations)... "<< std::endl;

}

int GaussianMixture::inverseSigmaMatrices(void)
{ // for optimisation purposes ..   
  for(int i = 0;i<nState;i++)
    {
      sigma[i].InverseSymmetric(invSigma[i], &detSigma[i]);
      if((!sigma[i].IsInverseOk())||isnan(detSigma[i]))
    return 0;
    }
  return 1;
}

double GaussianMixture::pdfState(Vector Vin,int state)
{
  /* get the probability density for a given state and a given vector */
  // REQUIRE call to inverseSigmaMatrices first .. 
  double p;
  Vector dif;

  dif = Vin - mu.GetRow(state);
  p=(double)(dif*(invSigma[state]*dif));
  p=exp(-0.5*p)/sqrt(pow(2*3.14159,dim)*(fabs(detSigma[state]+DBL_MIN)));
  if(p < DBL_MIN) return DBL_MIN;
  else return p;
 
}        

double GaussianMixture::pdfState(Vector Vin,Vector Components,int state) 
{
  /* Compute the probability density function at vector Vin, 
     (given along the dimensions Components), for a given state */ 
  Vector mu_s;
  Matrix sig_s;
  Matrix inv_sig_s;
  double det_sig;
  double p;
  int dim_s;
  dim_s = Components.Size();
  mu.GetRow(state).GetSubVector(Components,mu_s);
  sigma[state].GetMatrixSpace(Components,Components,sig_s);
  sig_s.InverseSymmetric(inv_sig_s,&det_sig);
  if(sig_s.IsInverseOk())
    {
      if(det_sig == 0)
    {
      std::cout << "Warning: null determinant" << std::endl;
      for(unsigned int k=0;k<Components.Size();k++)
        sig_s(k,k)=sig_s(k,k)+1.0;
      sig_s.Inverse(inv_sig_s,&det_sig);
      std::cout << det_sig << std::endl;
    }
      p=(Vin-mu_s) * ( inv_sig_s*(Vin-mu_s));
      p=exp(-0.5*p)/sqrt(pow(2*3.14159,dim_s)*(fabs(det_sig)+DBL_MIN));
      if(p < DBL_MIN) return DBL_MIN;
      else return p;
    }
  else
    {
      std::cout << "Error inverting sigma" << std::endl;
      return 0;
    }
}
double GaussianMixture::distState(Vector Vin,Vector Components,int state) 
{
  /* Compute the probability density function at vector Vin, 
     (given along the dimensions Components), for a given state */ 
  Vector mu_s;
  Matrix sig_s;
  Matrix inv_sig_s;
  double det_sig;
  double p;
  int dim_s;
  dim_s = Components.Size();
  mu.GetRow(state).GetSubVector(Components,mu_s);
  sigma[state].GetMatrixSpace(Components,Components,sig_s);
  sig_s.InverseSymmetric(inv_sig_s,&det_sig);
  if(sig_s.IsInverseOk())
    {
      if(det_sig == 0)
    {
      std::cout << "Warning: null determinant" << std::endl;
      for(unsigned int k=0;k<Components.Size();k++)
        sig_s(k,k)=sig_s(k,k)+1.0;
      sig_s.Inverse(inv_sig_s,&det_sig);
      std::cout << det_sig << std::endl;
    }
      p=(Vin-mu_s) * ( inv_sig_s*(Vin-mu_s));
      //p=exp(-0.5*p)/sqrt(pow(2*3.14159,dim_s)*(fabs(det_sig)+DBL_MIN));
      if(p < DBL_MIN) return DBL_MIN;
      else return p;
    }
  else
    {
      std::cout << "Error inverting sigma" << std::endl;
      return 0;
    }
}

Matrix GaussianMixture::doRegression(Matrix in,Matrix * SigmaOut,Vector inComponents,Vector outComponents)
{
  int nData = in.RowSize();
  int outDim = outComponents.Size();
  Matrix Pxi(nData,nState);
  Matrix out(nData,outDim);
  Matrix * subSigma; // arrays of covariances matrices sub-matrices
  Matrix * subSigmaVar;
  Matrix subMu; // stores subvectors of means 
  Matrix subMuIn;
  Matrix subMuOut;
    
  for(int i=0;i<nData;i++)
  {
    double norm_f = 0.0f;
    for(int s=0;s<nState;s++){
      double p_i = priors[s]*pdfState(in.GetRow(i),inComponents,s);
      
      Pxi(i,s) = p_i;
      norm_f += p_i;
    }
    Pxi.SetRow(Pxi.GetRow(i)/norm_f,i);
  }

  subSigma = new Matrix[nState];
  subSigmaVar = new Matrix[nState];
  mu.GetColumnSpace(outComponents,subMuOut);
  mu.GetColumnSpace(inComponents,subMuIn);

  for(int s=0;s<nState;s++) // computing all sub-matrices and sub-vectors 
  {
    Matrix isubSigmaIn;
    Matrix subSigmaOut;
    sigma[s].GetMatrixSpace(inComponents,inComponents,subSigmaOut);
    subSigmaOut.Inverse(isubSigmaIn);
    sigma[s].GetMatrixSpace(outComponents,inComponents,subSigmaOut);
    subSigma[s] = subSigmaOut*isubSigmaIn;
    sigma[s].GetMatrixSpace(outComponents,outComponents,subSigmaOut);
    sigma[s].GetMatrixSpace(inComponents,outComponents,isubSigmaIn);
    subSigmaVar[s] = subSigmaOut - subSigma[s]*isubSigmaIn;
        
  }

  Matrix tmpM;
    
  for(int i=0;i<nData;i++) // performs GMR for each input vector
  {
    Vector sv(outDim,true);
    Vector sp(outDim,true);
    Vector sd(outDim,true);
    SigmaOut[i] = Matrix(outDim,outDim);
    
    double spxi  =0;
    double spxi2 =0;
    for(int s=0;s<nState;s++)
    {
      sp = subMuOut.GetRow(s); 
      sp = sp + subSigma[s]*(in.GetRow(i)-subMuIn.GetRow(s));

      // CoVariance Computation
      //ES SigmaOut[i]=SigmaOut[i] + subSigmaVar[s]*(Pxi(i,s)*Pxi(i,s));

      double tsum = spxi + Pxi(i,s);
      double c1   = spxi     / tsum;
      double c2   = Pxi(i,s) / tsum;
      sd = sp-sv;

      SigmaOut[i] = SigmaOut[i] * c1  + subSigmaVar[s] * c2;
      sd.MultTranspose(sd,tmpM);
      SigmaOut[i] = SigmaOut[i] + tmpM * (c1*c2); 

      sv = sv *c1 + sp * c2;

      spxi  += Pxi(i,s);
      spxi2 += Pxi(i,s)*Pxi(i,s);      


      
    }
    //cout << spxi<<" "<<spxi2<<endl;
    out.SetRow(sv,i);
  }
  delete [] subSigma;
  delete [] subSigmaVar;
  return out;
}



Matrix GaussianMixture::doOffsetRegression( Matrix in,
                                            Vector offset,
                                            Matrix* SigmaOut,
                                            Vector inComponents,
                                            Vector outComponents){
  int nData = in.RowSize();
  int outDim = outComponents.Size();
  Matrix Pxi(nData,nState);
  Matrix out(nData,outDim);
  Matrix * subSigma; // arrays of covariances matrices sub-matrices
  Matrix * subSigmaVar;
  Matrix subMu; // stores subvectors of means 
  Matrix subMuIn;
  Matrix subMuOut;
    
  for(int i=0;i<nData;i++)
  {
    double norm_f = 0.0f;
    for(int s=0;s<nState;s++){
      double p_i = priors[s]*pdfState(in.GetRow(i),inComponents,s);
      Pxi(i,s) = p_i;
      norm_f += p_i;
    }
    Pxi.SetRow(Pxi.GetRow(i)/norm_f,i);
  }

  subSigma = new Matrix[nState];
  subSigmaVar = new Matrix[nState];
  mu.GetColumnSpace(outComponents,subMuOut);
  mu.GetColumnSpace(inComponents,subMuIn);

  for(int s=0;s<nState;s++) // computing all sub-matrices and sub-vectors 
  {
    Matrix isubSigmaIn;
    Matrix subSigmaOut;
    sigma[s].GetMatrixSpace(inComponents,inComponents,subSigmaOut);
    subSigmaOut.Inverse(isubSigmaIn);
    sigma[s].GetMatrixSpace(outComponents,inComponents,subSigmaOut);
    subSigma[s] = subSigmaOut*isubSigmaIn;
    sigma[s].GetMatrixSpace(outComponents,outComponents,subSigmaOut);
    sigma[s].GetMatrixSpace(inComponents,outComponents,isubSigmaIn);
    subSigmaVar[s] = subSigmaOut - subSigma[s]*isubSigmaIn;
        
  }
  Matrix tmpM;
    
  for(int i=0;i<nData;i++) // performs GMR for each input vector
  {
    Vector sv(outDim,true);
    Vector sp(outDim,true);
    Vector sd(outDim,true);
    SigmaOut[i] = Matrix(outDim,outDim);
    
    double spxi  =0;
    double spxi2 =0;
    for(int s=0;s<nState;s++)
    {
      sp = subMuOut.GetRow(s); 
      sp = sp + subSigma[s]*(in.GetRow(i)-subMuIn.GetRow(s));

      // CoVariance Computation
      //ES SigmaOut[i]=SigmaOut[i] + subSigmaVar[s]*(Pxi(i,s)*Pxi(i,s));

      double tsum = spxi + Pxi(i,s);
      double c1   = spxi     / tsum;
      double c2   = Pxi(i,s) / tsum;
      sd = sp-sv;

      SigmaOut[i] = SigmaOut[i] * c1  + subSigmaVar[s] * c2;
      sd.MultTranspose(sd,tmpM);
      SigmaOut[i] = SigmaOut[i] + tmpM * (c1*c2); 

      sv = sv *c1 + sp * c2;

      spxi  += Pxi(i,s);
      spxi2 += Pxi(i,s)*Pxi(i,s);      


      
    }
    /*
    Vector sv(outDim,true);
    Vector sp(outDim,true);
    SigmaOut[i] = Matrix(outDim,outDim);
    for(int s=0;s<nState;s++)
    {
      sp = subMuOut.GetRow(s); 
      sp = sp + subSigma[s]*(in.GetRow(i)-subMuIn.GetRow(s));
      sv += sp*Pxi(i,s);

      // CoVariance Computation
      SigmaOut[i]=SigmaOut[i] + subSigmaVar[s]*(Pxi(i,s)*Pxi(i,s));
    }*/
    Matrix sqrtSigma;
    SigmaOut[i].SquareRoot(sqrtSigma);
    sv += sqrtSigma*offset;
    
    out.SetRow(sv,i);
  }
  
  delete [] subSigma;
  delete [] subSigmaVar;
  return out;                                                
}




Matrix GaussianMixture::doOffsetRegression( Matrix in,
                                            Vector offset,
                                            Matrix* SigmaOut,
                                            Vector inComponents,
                                            Vector outComponents,
                             Matrix inPrior,
                             Vector priorComponents){
  int nData = in.RowSize();
  int outDim = outComponents.Size();
  Matrix Pxi(nData,nState);
  Matrix out(nData,outDim);
  Matrix * subSigma; // arrays of covariances matrices sub-matrices
  Matrix * subSigmaVar;
  Matrix subMu; // stores subvectors of means 
  Matrix subMuIn;
  Matrix subMuOut;
    
  double norm_f = 0.0f;
  for(int i=0;i<nData;i++)
  {
    norm_f = 0.0f;
    for(int s=0;s<nState;s++){
      //inPrior.GetRow(i).Print();
      double p_i = priors[s]*pdfState(in.GetRow(i),inComponents,s);
      p_i *= pdfState(inPrior.GetRow(i),priorComponents,s);
      Pxi(i,s) = p_i;
      norm_f += p_i;
    }
    Pxi.SetRow(Pxi.GetRow(i)/norm_f,i);
    Pxi.GetRow(i).Print();

    /*
    Vector dist(nState);
    norm_f = 0.0;
    for(int s=0;s<nState;s++){
      //inPrior.GetRow(i).Print();
      if(Pxi(i,s)>1e-2){
          double p_i = pdfState(inPrior.GetRow(i),priorComponents,s);
          dist(s) = p_i;
          norm_f += p_i;
      }else{
          dist(s) = 0;
      }    
    }
    dist *= (1.0/norm_f);
    dist.Print();
    
    norm_f = 0.0;
    for(int s=0;s<nState;s++){
        if(Pxi(i,s)>1e-2){
            Pxi(i,s) *= dist(s);
        }else{
            Pxi(i,s) = 0;
        }
        norm_f += Pxi(i,s);    
    }    
    
    Pxi.SetRow(Pxi.GetRow(i)/norm_f,i);
    
    Pxi.GetRow(i).Print();
    */
    
    //Pxi.GetRow(i).Print();
  }
  //cout << "prior "<<norm_f<<" ";
  //Pxi.Print();

  subSigma = new Matrix[nState];
  subSigmaVar = new Matrix[nState];
  mu.GetColumnSpace(outComponents,subMuOut);
  mu.GetColumnSpace(inComponents,subMuIn);

  for(int s=0;s<nState;s++) // computing all sub-matrices and sub-vectors 
  {
    Matrix isubSigmaIn;
    Matrix subSigmaOut;
    sigma[s].GetMatrixSpace(inComponents,inComponents,subSigmaOut);
    subSigmaOut.Inverse(isubSigmaIn);
    sigma[s].GetMatrixSpace(outComponents,inComponents,subSigmaOut);
    subSigma[s] = subSigmaOut*isubSigmaIn;
    sigma[s].GetMatrixSpace(outComponents,outComponents,subSigmaOut);
    sigma[s].GetMatrixSpace(inComponents,outComponents,isubSigmaIn);
    subSigmaVar[s] = subSigmaOut - subSigma[s]*isubSigmaIn;
        
  }
  Matrix tmpM;
    
  for(int i=0;i<nData;i++) // performs GMR for each input vector
  {
    Vector sv(outDim,true);
    Vector sp(outDim,true);
    Vector sd(outDim,true);
    SigmaOut[i] = Matrix(outDim,outDim);
    
    double spxi  =0;
    double spxi2 =0;
    for(int s=0;s<nState;s++)
    {
      sp = subMuOut.GetRow(s); 
      sp = sp + subSigma[s]*(in.GetRow(i)-subMuIn.GetRow(s));

      // CoVariance Computation
      //ES SigmaOut[i]=SigmaOut[i] + subSigmaVar[s]*(Pxi(i,s)*Pxi(i,s));

      double tsum = spxi + Pxi(i,s);
      if(tsum>1e-6){
          double c1   = spxi     / tsum;
          double c2   = Pxi(i,s) / tsum;
          sd = sp-sv;
    
          SigmaOut[i] = SigmaOut[i] * c1  + subSigmaVar[s] * c2;
          sd.MultTranspose(sd,tmpM);
          SigmaOut[i] = SigmaOut[i] + tmpM * (c1*c2); 
    
          sv = sv *c1 + sp * c2;
    
      }
      spxi  += Pxi(i,s);
      spxi2 += Pxi(i,s)*Pxi(i,s);      

      
    }
    /*
    Vector sv(outDim,true);
    Vector sp(outDim,true);
    SigmaOut[i] = Matrix(outDim,outDim);
    for(int s=0;s<nState;s++)
    {
      sp = subMuOut.GetRow(s); 
      sp = sp + subSigma[s]*(in.GetRow(i)-subMuIn.GetRow(s));
      sv += sp*Pxi(i,s);

      // CoVariance Computation
      SigmaOut[i]=SigmaOut[i] + subSigmaVar[s]*(Pxi(i,s)*Pxi(i,s));
    }*/
    Matrix sqrtSigma;
    //cout << i<<endl;
    //SigmaOut[i].SquareRoot(sqrtSigma);
    //sv += sqrtSigma*offset;
    
    out.SetRow(sv,i);
  }
  
  delete [] subSigma;
  delete [] subSigmaVar;
  return out;                                                
}


Vector GaussianMixture::getRegressionOffset( Vector in, 
                                             Vector currOut,
                                             Vector inComponents,
                                             Vector outComponents){
    Vector regOut,out;
    Matrix sigma,iSqrtSigma;
    regOut = doRegression(in,sigma,inComponents,outComponents);
    sigma.InverseSquareRoot(iSqrtSigma);
    iSqrtSigma.Mult((currOut-regOut),out);
    return out;
}
Vector GaussianMixture::getRegressionOffset( Vector in, 
                                             Vector currOut,
                                             Vector inComponents,
                                             Vector outComponents,
                                             Vector inPrior,
                                             Vector priorComponents){
    Matrix regOutM;
    Vector regOut, out;
    Matrix sigma,iSqrtSigma;
    
    Matrix inM(1,in.Size()); inM.SetRow(in,0);
    Matrix inPriorM(1,inPrior.Size()); inPriorM.SetRow(inPrior,0);
    
    Vector offset(outComponents.Size());offset.Zero();
    regOutM = doOffsetRegression(inM,offset,&sigma,inComponents,outComponents,inPriorM,priorComponents);
    regOutM.GetRow(0,regOut);
    sigma.InverseSquareRoot(iSqrtSigma);
    iSqrtSigma.Mult((currOut-regOut),out);
    return out;
}




Vector GaussianMixture::doRegression(Vector in,Matrix& Sigma,
                     Vector inComponents,Vector outComponents)
{
  int outDim = outComponents.Size();
  Vector Pxi(nState);
  Matrix * subSigma;
  Matrix * subSigmaVar;
  Matrix subMu;
  Matrix subMuIn;
  Matrix subMuOut;
  double norm_f = 0.0f;
  for(int s=0;s<nState;s++){
    double p_i = priors[s]*pdfState(in,inComponents,s);
    Pxi(s) = p_i;
    norm_f += p_i;
  }
  Pxi= Pxi*(1/norm_f);
  subSigma = new Matrix[nState];
  subSigmaVar = new Matrix[nState];
  mu.GetColumnSpace(outComponents,subMuOut);
  mu.GetColumnSpace(inComponents,subMuIn);

  for(int s=0;s<nState;s++)
    {
      Matrix isubSigmaIn;
      Matrix subSigmaOut;
      sigma[s].GetMatrixSpace(inComponents,inComponents,subSigmaOut);
      subSigmaOut.Inverse(isubSigmaIn);
      sigma[s].GetMatrixSpace(outComponents,inComponents,subSigmaOut);
      subSigma[s] = subSigmaOut*isubSigmaIn;
      sigma[s].GetMatrixSpace(outComponents,outComponents,subSigmaOut);
      sigma[s].GetMatrixSpace(inComponents,outComponents,isubSigmaIn);
      subSigmaVar[s] = subSigmaOut - subSigma[s]*isubSigmaIn;
        
    }
    
  
  Vector sv(outDim,true);
  Vector sp(outDim,true);
  Vector sd(outDim,true);
  Sigma.Resize(outDim,outDim,true);
  Sigma.Zero();
  Matrix tmpM;

    
  double spxi  =0;
  double spxi2 =0;
  for(int s=0;s<nState;s++)
  {
      sp = subMuOut.GetRow(s); 
      sp = sp + subSigma[s]*(in-subMuIn.GetRow(s));

      // CoVariance Computation
      //ES SigmaOut[i]=SigmaOut[i] + subSigmaVar[s]*(Pxi(i,s)*Pxi(i,s));

      double tsum = spxi + Pxi(s);
      double c1   = spxi     / tsum;
      double c2   = Pxi(s) / tsum;
      sd = sp-sv;

      Sigma = Sigma * c1  + subSigmaVar[s] * c2;
      sd.MultTranspose(sd,tmpM);
      Sigma = Sigma + tmpM * (c1*c2); 

      sv = sv *c1 + sp * c2;

      spxi  += Pxi(s);
      spxi2 += Pxi(s)*Pxi(s);      

      
  }
  /*
  for(int s=0;s<nState;s++)
    {
      sp = subMuOut.GetRow(s); 
      sp = sp + subSigma[s]*(in-subMuIn.GetRow(s));
      sv += sp*Pxi(s);

      // CoVariance Computation
      Sigma=Sigma + subSigmaVar[s]*(Pxi(s)*Pxi(s));
    }
    */  

  delete [] subSigma;
  delete [] subSigmaVar;
  return sv;
}

long double GaussianMixture::GaussianPDF(Vector input,Vector Mean,Matrix covariance)
{
  /* get the probability density for a given state and a given vector */

  long double p;
  Vector dif;
  int dim=input.Size();
  Matrix inv;
  double det;

  covariance.Inverse(inv,&det);
  if(det < DBL_MIN){
    det = DBL_MIN;
  }
  if(covariance.IsInverseOk()){
    dif = input - Mean;
    p = (long double)(dif*(inv*dif));
    p = expl(-0.5*p) / sqrtl(pow(2.0*3.14159,dim)*(fabs(det)+1e-300));
    if(p < DBL_MIN){
      
      return DBL_MIN;
    }
    else 
      return p;
  }
  else{
    std::cout << "Error inverting sigma" << std::endl;
    return 0;
  }
}

double GaussianMixture::likelihood(Vector input)
{
  double lik=0.;
  if(input.Size() < this->dim)
    {
      Vector components(input.Size());
      for(int k=0;k<input.Size();k++)
    components(k) = k;
      for( int i=0;i<this->nState;i++) 
    lik += pdfState(input,components,i)*priors[i];
    }
  else
    {
      for( int i=0;i<this->nState;i++) 
    lik += pdfState(input,i)*priors[i];
    }
  return lik;
}

double GaussianMixture::log_likelihood(Vector input)
{
  return log(likelihood(input));
}

double GaussianMixture::log_likelihood(Matrix inputs)
{
  double log_lik=0.;
  for(int j=0.;j<inputs.RowSize();j++)
    {
      log_lik+=log_likelihood(inputs.GetRow(j));
    }
  return log_lik;
}


Vector GaussianMixture::doRegressionPartial(Vector in,Matrix& Sigma,
                        Vector inComponents,Vector outComponents,
                        Vector states)
{
  int outDim = outComponents.Size();
  int p_states = states.Size();
  Vector Pxi(p_states);
  int cs;
  Matrix * subSigma;
  Matrix * subSigmaVar;
  Matrix subMu;
  Matrix subMuIn;
  Matrix subMuOut;
  double norm_f = 0.0f;
  for(int s=0;s<p_states;s++){
    cs = (int) states(s);
    double p_i = priors[cs]*pdfState(in,inComponents,cs);
    Pxi(s) = p_i;
    //std::cout << p_i << "  ";
    norm_f += p_i;
  }
  // std::cout << 0./norm_f ;
  if(norm_f  != 0)
    {
    for(int s= 0;s<p_states;s++)
      Pxi(s) = Pxi(s)/norm_f;
    }
  else
    {
      Pxi(0) = 0.5;
      Pxi(1) = 0.5;
    }
  //Pxi.Print();

  subSigma = new Matrix[p_states];
  subSigmaVar = new Matrix[p_states];
  mu.GetColumnSpace(outComponents,subMuOut);
  mu.GetColumnSpace(inComponents,subMuIn);

  for(int s=0;s<p_states;s++)
    {
      Matrix isubSigmaIn;
      Matrix subSigmaOut;
      cs = (int) states(s);
      sigma[cs].GetMatrixSpace(inComponents,inComponents,subSigmaOut);
      subSigmaOut.Inverse(isubSigmaIn);
      sigma[cs].GetMatrixSpace(outComponents,inComponents,subSigmaOut);
      subSigma[s] = subSigmaOut*isubSigmaIn;
      sigma[cs].GetMatrixSpace(outComponents,outComponents,subSigmaOut);
      sigma[cs].GetMatrixSpace(inComponents,outComponents,isubSigmaIn);
      subSigmaVar[s] = subSigmaOut - subSigma[s]*isubSigmaIn;
        
    }
    
  
  Vector sv(outDim,true);
  Vector sp(outDim,true);
  Sigma.Resize(outDim,outDim,true);
  Sigma.Zero();
  for(int s=0;s<p_states;s++)
    {
      cs = (int) states(s);
      sp = subMuOut.GetRow(cs); 
      sp = sp + subSigma[s]*(in-subMuIn.GetRow(cs));
      sv += sp*Pxi(s);

      // CoVariance Computation
      Sigma=Sigma + subSigmaVar[s]*(Pxi(s)*Pxi(s));
    }
      

  delete [] subSigma;
  delete [] subSigmaVar;
  return sv;
}


void GaussianMixture::addDataPointPartial(const Vector& dataPoint, double lambda, int state, const Vector& inComponents){
    if((state<0)||(state>=nState))
        return;

    Vector muBack;
    

    lambda = TRUNC(lambda,0.0,1.0);
    
    Vector cMu(dim);
    Vector dMu(dim);    
    mu.GetRow(state,cMu);
    muBack = cMu;
    dMu = cMu;
    cMu *= (1.0-lambda);
    Vector cPt(dataPoint);
    cPt *= lambda;
    cMu += cPt;
    for(int i=0;i<inComponents.Size();i++){
        int id=int(ROUND(inComponents.At(i)));
        //cMu(id) = muBack(id);         
    }
    mu.SetRow(cMu,state);

    dMu -= dataPoint;
    Matrix dSigma;
    dMu.MultTranspose(dMu,dSigma);
    dSigma *= lambda*(1.0-lambda);
    Matrix cSigma(sigma[state]);
    cSigma *= (1.0-lambda);
    cSigma += dSigma;
    sigma[state] = cSigma;

    /*
    sigma[state].InverseSymmetric(invSigma[state], &detSigma[state]);
    if((!sigma[state].IsInverseOk())||isnan(detSigma[state]))
        cout << "Error while inversing cov matrix"<< endl;
        */
}

void GaussianMixture::addDataPoint(const Vector& dataPoint, double lambda, const Vector& inComponents){
  Vector Pxi(nState);
  Vector in;
  
  dataPoint.GetSubVector(inComponents,in);
  
  double norm_f = 0.0f;
  for(int s=0;s<nState;s++){
    double p_i = priors[s]*pdfState(in,inComponents,s);
    Pxi(s) = p_i;
    norm_f += p_i;
  }
  Pxi= Pxi*(1/norm_f);
  
  for(int s=0;s<nState;s++){
    addDataPointPartial(dataPoint,lambda*Pxi(s),s,inComponents);    
  }    
}

void GaussianMixture::addDataSet(const Matrix& dataSet, double lambda, const Vector& inComponents){
  int N = dataSet.RowSize();
  Matrix PxiM(N,nState);
  Vector Pxi(nState);
  Vector in;
  Vector dataPoint;

  lambda = TRUNC(lambda,0.0,1.0);
  double Np = 1.0/lambda;

  
  for(int i=0;i<N;i++){
      dataSet.GetRow(i,dataPoint);
      dataPoint.GetSubVector(inComponents,in);
      
      double norm_f = 0.0f;
      for(int s=0;s<nState;s++){
        double p_i = priors[s]*pdfState(in,inComponents,s);
        Pxi(s) = p_i;
        norm_f += p_i;
      }
      Pxi= Pxi*(1.0/norm_f);
      PxiM.SetRow(Pxi,i);
  }
  
  for(int i=0;i<N;i++){
      for(int s=0;s<nState;s++){
        lambda= 1.0/Np;
        dataSet.GetRow(i,dataPoint);
        PxiM.GetRow(i,Pxi);
        addDataPointPartial(dataPoint,lambda*Pxi(s),s,inComponents);
        Np+=1.0;    
    }
  }    
  
}























double GaussianMixture::stepEM(Matrix DataSet, Vector weights, bool update_covariances)
{
  int nData = DataSet.RowSize();
  double nWeightedData = weights.Sum(); 
  double log_likelihood = 0.; 
  Vector sum_p(nData);
  Matrix pxi(nData,nState);
  Matrix pix(nData,nState);
  Vector E;

  // temporary variables for computation purposes ..
  Matrix tmptmu(nData,dim);
  Vector tmu(dim);
  


  // before Expectation, lets inverse the covariances matrices 
  if(inverseSigmaMatrices() < 1)
    {
      std::cout << "can't inverse covariances matrices" <<std::endl;
      return 0.;
    }
  /** Expectation Computing **/ 
  for(int i =0;i<nData;i++)
  {
     for(int j=0;j<nState;j++)
     { 
       double p = pdfState(DataSet.GetRow(i),j);  // P(x|i)
      
       // checking for numerical problems .. 
       if(p==0) {
           std::cerr << "Error: Null probability." << std::endl;
           return 0.;
       }
       if(isnan(p)) 
       {
          std::cerr << "Error: NAN probability." << std::endl;
          return 0.;
       }
       if(p<DBL_MIN){
           p = DBL_MIN;
           std::cerr << "Warning: too small probability." << std::endl;
       }
       //pxi(i,j)= p;
       pxi(i,j)= weights(i) * p;
     }   
  } 
  sum_p = pxi * priors; // sum_p[i] = sum_i ( p(x|i) priors[i] )
     
  for(int i=0;i<nData;i++) 
    log_likelihood += log(weights(i) *sum_p[i]);

  for(int j=0;j<nState;j++)
  {
    for(int i=0;i<nData;i++)
    {
      pix(i,j) = weights(i)*pxi(i,j)*priors(j)/sum_p[i]; // then P(i|x) 
    }   
  } 
 
  /** Update Step **/
  pix.SumRow(E);
  for(int j=0;j<nState;j++) 
  {
    priors(j)=E(j)/nWeightedData; // new priors 
      
    // Preventing it from becoming totally null ..
    if(priors(j) == 0.0) 
    {
      priors[j]= 1e-5;
      std::cerr << "Numerical instability :: null priors ..." << std::endl;
      std::cerr << "please considers decreasing states numbers"<<std::endl;
    }
     
    DataSet.MultTranspose(pix.GetColumn(j),tmu); // new mean for state j
    // tmu = Dataset^T * pix.Column(j) 
    // ou encore .. tmu = sum_x ( x * p(x| j) ) pour tout les x du Dataset
      
    tmu = tmu / E(j);
    mu.SetRow(tmu,j);

    if(update_covariances) 
    {
          // (Dataset - tmu)^T * ((Dataset - tm) .* pix.getRow(j))

       for(int i=0;i<nData;i++)
       {
          tmptmu.SetRow(tmu,i);
       }
          // sigma[j] = sum _ x ( (x-mu_j)^T (x-m_j) * p(x|j) ) 

       tmptmu = DataSet - tmptmu;  // (x-mu_j)
       tmptmu.MultTranspose(tmptmu ^ pix.GetColumn(j) ,sigma[j]);
          // sigma_j = tmptmu ^T * tmptmu * p(x|j) 
          
       sigma[j] = sigma[j]/E(j);
     }
  }
  return log_likelihood/nWeightedData; 
}


void GaussianMixture::doEM(Matrix DataSet, Vector weights, int blank_run,int max_iter)
{
  /* perform Expectation/Maximization on the given Dataset :
     Matrix DataSet(nSamples,Dimensions).
     The GaussianMixture Object must be initialised before 
     (see initEM_TimeSplit method ) */
  int nData = DataSet.RowSize();  
  int iter = 0;
  double log_lik;
  double log_lik_threshold = EM_STOPPING_CRIT;
  double log_lik_old=-1e10;

  Matrix var_keep(dim,dim);
  Vector mean(dim);
  Vector var(dim);
  int success_iter = 0;
 // computing Dataset variance :
 
  for(int i = 0;i<nData;i++)
    {
      mean += DataSet.GetRow(i);
    }
    
  mean = mean/nData;
  
  for(int i = 0;i<nData;i++)
    {
      for(int j =0 ; j<dim;j++)
      {
        var(j) += pow(DataSet(i,j)-mean(j),2);
      }
    }
  var = var/nData;
 
  for(int k = 0;k<dim;k++) var_keep(k,k)=1/sqrt(var(k));
 
  while( iter < (max_iter+blank_run)) //EM loop
    {
      iter++;
      //std::cout << "EM current iteration: "<<iter<<std::endl; 
      if(iter < blank_run)
        log_lik = stepEM(DataSet,weights,false);
      else
        log_lik = stepEM(DataSet,weights,true);
        
      for(int i=0;i<nState;i++)
        sigma[i] += var_keep*VARIANCE_KEEPER;

      if(log_lik != 0.)
        success_iter += 1;
      else
        {
          //for(int i=0;i<nState;i++)
          //  sigma[i] += var_keep;  // boosting covariances .. 
        }
      // std::cout << std::setprecision(10) <<  log_lik << std::endl;

      if(fabs((log_lik/log_lik_old)-1) < log_lik_threshold )
        {
        /* if log likehood hasn't move enough, the algorithm has
           converged, exiting the loop */
        break;
        }
      log_lik_old = log_lik;
      cout << log_lik<<endl;
    }

  std::cerr << "(EM finished with "<<success_iter<<" iterations)... "<<std::endl;

}

void GaussianMixture::balancePriors(){
    for(int i=0;i<dim;i++){
        priors(i) = 1.0/double(dim);    
    }
}

