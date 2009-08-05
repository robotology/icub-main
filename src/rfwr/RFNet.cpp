#include "RFNet.h"
#include "YARPConfigFile.h"

using namespace std;

bool wLower(Rf * a, Rf * b)
{
  return (a->getW() < b->getW());
}

double max(double in1, double in2)
{ 
	if (in1>=in2)
		return in1;
	else
		return in2;
}


//-------------------------------------------------
// Costruttori
//-------------------------------------------------

RFNet::RFNet(void)
{
  _default();
}

void RFNet::_default()
{
  n_in = 0;
  n_out = 0;
  diag_only = 0;
  meta = 0;
  meta_rate = 0;
  penalty = 0;
  init_alpha_num = 0;
         
  n_data = 0;
  w_gen = 0.1;
  w_prune = 0.9;
  init_lambda = 0.999;
  final_lambda = 0.9999;
  tau_lambda = 0.99999;
  init_P = 1.e+10;
  n_pruned = 0;
  kernel = "Gaussian";
  //	 rfs.reserve (100);
}

RFNet::RFNet(int n_inp,int n_outp, bool diag_onlyp, bool metap, double meta_ratep, double penaltyp, double init_alphap, const YVector  &normp, const YVector &norm_outp)
{
  _default();
  Init(n_inp,n_outp,diag_onlyp, metap, meta_ratep, penaltyp,init_alphap, normp, norm_outp);
}

RFNet::RFNet(const RFNet &net)
{
  n_data = net.n_data;
  w_gen = net.w_gen;
  w_prune = net.w_prune ;
  init_lambda = net.init_lambda;
  final_lambda = net.final_lambda;
  tau_lambda = net.tau_lambda;
  init_P = net.init_P;
  n_pruned = net.n_pruned;

  n_in       = net.n_in;
  n_out      = net.n_out;
  diag_only  = net.diag_only;
  meta       = net.meta;
  meta_rate  = net.meta_rate;
  penalty    = net.penalty;
  init_alpha_num = net.init_alpha_num;
  norm       = net.norm;
  norm_out   = net.norm_out;
     
  init_D       = net.init_D;
  init_M       = net.init_M;
  init_alpha   = net.init_alpha;
  mean_x	= net.mean_x;	
  var_x		= net.var_x;
  rfs     = net.rfs;
  kernel       = net.kernel;
}

void RFNet::Init (int n_inp,int n_outp, bool diag_onlyp, bool metap, double meta_ratep,	double penaltyp, double init_alphap, const YVector  &normp, const YVector &norm_outp)
{
  n_in       = n_inp;
  n_out      = n_outp;
  diag_only  = diag_onlyp;
  meta       = metap;
  meta_rate  = meta_ratep;
  penalty    = penaltyp;
  init_alpha_num = init_alphap;
  norm       = normp;
  norm_out   = norm_outp;
      
  // other variables

  init_D = identity(n_in)*25;

  init_M = init_D;	
  choldc(init_M,n_in);

  init_alpha   = ones(n_in)*init_alpha_num;
  mean_x.Resize(n_in);
  var_x.Resize(n_in);
  prediction.Resize(n_out);
  xn.Resize(n_in);
  yn.Resize(n_out);
  yp_i.Resize(n_in);
  _tempX.Resize(n_in);
  _tempY.Resize(n_out);
  //  tms.Resize();
  //      rfs.reserve (100);
}

//-------------------------------------------------------------
// Funzioni che permettono di settare determinati parametri.
//-------------------------------------------------------------

void RFNet::SetWGen(double wgen){w_gen = wgen;}
void RFNet::SetWPrune(double wprune){w_prune = wprune;}
void RFNet::SetMeta(bool m){meta = m;}
void RFNet::SetMetaRate(double mr){meta_rate = mr;}
void RFNet::SetInitD(int d1, int d2, double * elem) {init_D.Resize(d1,d2,elem);}
void RFNet::SetInitD(const YMatrix & m) {init_D = m;}
void RFNet::SetInitAlpha(int d1, int d2, double * elem) {init_alpha.Resize(d1,d2,elem);}
void RFNet::SetInitAlpha(const YMatrix & m) {init_alpha = m;}

//-------------------------------------------------------------
// Addestra la rete, data una coppia x, y.
// Salva una prima previsione in yp, che deve essere allocato
// anticipatamente nel programma.
//-------------------------------------------------------------

double  RFNet::Train(const YVector &x, const YVector &y, bool composite_control ,double * e_stor, int alpha )
{
	return Train(x, y, prediction, composite_control, e_stor, alpha);
}

double  RFNet::Train(const YVector &x, const YVector &y, YVector & yp, bool composite_control ,double * e_stor, int alpha )
{
  if (composite_control)
    {
      for(unsigned int f=0; f< rfs.size (); f ++)
	rfs[f]->setE (e_stor,n_in);
    }
	
  Rf * nuovo;
  nuovo = new Rf;// Serve poi (alla riga 265...ogni volta inizializz. diversam.)

  indicaRf nu; //+

  // update the global mean and variance of the training data for
  // information purposes
  mean_x = (mean_x * n_data + x)/(n_data + 1);

  square(x-mean_x, _tempX);
  var_x = (var_x * n_data + _tempX)/(n_data + 1);
  n_data = n_data+1;

  // normalize the inputs
  ratio(x,norm,xn); 
    
  // normalize the outputs
  ratio(y,norm_out,yn);
    
  // check all RFs for updating
  // wv is a vector of 3 weights, ordered [w; sec_w; max_w]
  // iv is the corresponding vector containing the RF indices
    
  vector <indicaRf> wv; //+
	
  Rf * falso = new Rf;
  indicaRf fittizio; //+
  setIndRf(fittizio,falso,0);//+

  wv.push_back (fittizio); //+
  wv.push_back (fittizio); //+
  wv.push_back (fittizio); //+

  double sum_w = 0;
    
  tms.Resize(rfs.size());

  // Al primo giro non ci sono rfs, quindi il for viene saltato.	
  unsigned int i;
  for (i=0; i < rfs.size(); i++)
    {
      
      //	if(! rfs[i]->isErased()) { //**

      // compute the weight and keep the three larget weights sorted
      // Attraverso la funzione modifico comunque w, ma se poi non va bene lo azzero.
	
      double w  = rfs[i]->compute_weight(diag_only,kernel,xn);
		
      // Realizzo wv con un vettore di puntatori a Rf
      setIndRf(nu,rfs[i],i ); //+ 
      *wv.begin()  = nu; //+

      sort(wv.begin (),wv.end (),wMinore); //+

      // only update if activation is high enough
      if (w > 0.001)
	{
	  // update weighted mean for xn and y, and create mean-zero 
	  // variables
	  
	  rfs[i]->update_means(xn,yn,w);
		
	  // update the regression
	  yp_i = rfs[i]->update_regression(w);

	  // e_cv ed e passati per riferimento

	  if (rfs[i]->isTrustworthy())
	    {
	      yp = w*yp_i + yp;
	      sum_w = sum_w + w;
	    }	

	  // update the distance metric
	  double tm = rfs[i]->update_distance_metric(w, xn, penalty, meta, meta_rate, kernel, diag_only);

	  tms(i+1) = 1; 
	  // Interviene cmq solo in composite control...
	  // Eventualmente in seguito monitorarne il funzionam!

	  rfs[i]->update_statistical(w,tau_lambda,final_lambda);
	}
      else 
	{
	  rfs[i]->setW(0);

	}  // if (w > 0.001)

    } // chiusura del for

  // if RFWR is used for control, incorporate the tracking error
  
  if (composite_control)
    {
      int * inds,numOcc;
      inds = new int[tms.Length()];

      numOcc = findMag(tms,0,inds); 
		
      if(numOcc != 0)
	{

	  int j;
	  for(j=1; j<=numOcc; j++)
	    {

	      i = inds[j]-1; // In quanto rfs parte da 0 essendo un vector.

	      rfs[i]->update_composite(alpha,tms(j),sum_w,xn);
							
	    }
	}
    
      rfs[i] ->setE(0); // Per evitare sovrapposizioni tra training successivi
    }

  // Da qui in poi istruzioni che si verificano una volta sola per ogni train;

  double maxW = wv[2].point ->getW(); // Val da ritornare alla fine del train
  // In questo modo esiste anche quando pruno
  // questo rf!
	
  // do we need to add a new RF?

  if (wv[2].point -> getW() <= w_gen) //+
    { 

      if (wv[2].point ->getW() > 0.1 * w_gen && wv[2].point ->isTrustworthy()) //+
		
	{	
	  nuovo->init(wv[2].point ,xn,yn, n_in, n_out, init_P,  init_lambda);
	  rfs.push_back (nuovo);
	}
      else 
	{	  
	  if (rfs.size()==0)
	    {
	      nuovo->init(xn,y, init_D,  init_M,  init_alpha, n_in, n_out,init_P, init_lambda);
	      rfs.push_back (nuovo); 
	    }
	  else
	    {
	      nuovo->init(xn,yn, init_D,  init_M,  init_alpha, n_in, n_out,init_P, init_lambda);	
	      rfs.push_back (nuovo);
				
	    }
	}
    }

  // do we need to prune a RF? Prune the one with smaller D (((???)))
  //Erase the one with larger D!
    
  if (wv[1].point  ->getW() > w_prune && wv[2].point  ->getW() > w_prune) //+
    {
      if (sumTot(wv[1].point  ->getD()) > sumTot(wv[2].point  ->getD())) //+
	{
	  rfs.erase (rfs.begin() + wv[1].ind ); //+
				  
	  cout << "Pruned #RF = " << wv[1].ind + 1 << endl; //+
	}
      else
	{

	  rfs.erase (rfs.begin() + wv[2].ind ); //+
	  cout << "Pruned #RF = " << wv[2].ind + 1<< endl;
	}

      n_pruned = n_pruned + 1;
    }

  if (sum_w > 0)
    {
      times(yp,norm_out,_tempY);
      
      yp = _tempY/sum_w;
    }

  return maxW;
}

    
//----------------------------------------------------------------------------------
// Fornisce la predizione, a partire da un vettore di ingresso x e da una soglia
// di attivazione cutoff - di default pari a 0. Salva la yp in un vettore yp 
// precedentemente allocato.
//-----------------------------------------------------------------------------------

double RFNet::Simulate (const YVector &x, double cutoff, YVector &yp)
{
  // Dati comodi
  double w; // serve a riga 401
  double max_w = 0;
  double sum_w = 0;
	
  ratio(x,norm,xn);

  yp=0.0;

  for(unsigned int i=0; i < rfs.size(); i++)
    {     
      w  = rfs[i]->compute_weight(diag_only,kernel,xn);
      
      max_w = max(max_w,w);		
      
      if (w > cutoff && rfs[i]->isTrustworthy())
	{
	
	  // the mean zero input
	  xmzp = xn - rfs[i]->getmean_x();
	
	  // the prediction
	  yp = yp + (rfs[i]->getB().Transposed() * xmzp + rfs[i]->getb0()) * w;
			  
	  sum_w = sum_w + w;
	
	}// if (w > cutoff)
      
    }

  if (sum_w > 0)
    {
      times(yp,norm_out,yp);
      yp/=sum_w;
    }

  return max_w; 
}

void RFNet::Jacobian(const YVector &x, double cutoff, YMatrix &J)
{
    int M=rfs.size();

    YVector w(M), yp(n_out), yn_num(n_out);
    double sum_w=0.0, sum_w_2;

    ratio(x,norm,xn);
    Simulate(x,cutoff,yp);
    ratio(yp,norm_out,yn);

    for (int m=0; m<M; m++)
    {
        w[m]=rfs[m]->compute_weight(diag_only,kernel,xn);
        sum_w+=w[m];
    }

    yn_num =sum_w*yn;
    sum_w_2=sum_w*sum_w;

    J=0.0;

    // Given yp = norm_out * (yn_num / sum_w), it follows that:
    // J=dyp/dx = norm_out * (dyn_num*sum_w-yn_num*sum_dw) / sum_w_2

    for (int j=1; j<=n_out; j++)
        for (int k=1; k<=n_in; k++)
        {
            double sum_dw=0.0, dyn_num=0.0;

            for (int m=0; m<M; m++)
                if (w[m]>cutoff && rfs[m]->isTrustworthy())
                {
                    const YVector &b0_m=rfs[m]->getb0();
                    const YMatrix &B_m=rfs[m]->getB();
                    const YVector &c_m=rfs[m]->getC();
                    const YMatrix &D_m=rfs[m]->getD();

                    // compute dw
                    YVector v=D_m*(xn-c_m);
                    double dw=-w[m]*v(k)/norm(k);
                    sum_dw+=dw;

                    // compute dyn_num
                    xmzp=xn-rfs[m]->getmean_x();
                    dyn_num+=(w[m]*B_m(k,j))/norm(k) + dw*((B_m.Transposed()*xmzp+b0_m)(j));
                }

            J(j,k)=norm_out(j)*(dyn_num*sum_w-yn_num(j)*sum_dw)/sum_w_2;
        }
}

void RFNet::print() const
{
  cout << "RfNet:\n\n" 
       << "n_in: " << n_in << '\n'
       << "n_out: " << n_out  << '\n'
       << "diag_only: " << diag_only << '\n'
       << "meta: " << meta << '\n' 
       << "meta_rate: "  << meta_rate << '\n'
       << "penalty: "	<< penalty << '\n'
       << "init_alpha: "	<< init_alpha << '\n'
       << "norm: "	<< norm 
       << "norm_out: "	<< norm_out 
       << "n_data: "	<< n_data << '\n' 
       << "w_gen: " << w_gen << '\n' 		
       << "w_prune: " << w_prune << '\n'
       << "init_lambda: " << init_lambda << '\n'
       << "final_lambda: " << final_lambda << '\n'
       << "tau_lambda: " << tau_lambda << '\n'
       << "init_P: " << init_P << '\n'
       << "n_pruned: " << n_pruned << '\n'
       << "init_D: " << init_D << '\n'
       << "init_M" << init_M << '\n'
       << "mean_x: " << mean_x 
       << "var_x: " << var_x
       << "kernel: " << kernel <<  '\n'
       << "numero rfs: " << rfs.size() << '\n' << endl;
#if 0
  for (int i = 0; i <rfs.size(); i++)
    {	
      cout << i+1 << ")\t";
      rfs[i]->print();
      cout << '\n';
    }
#endif
}

// Funzione di stampa su file. Stampa anche a schermo.

void RFNet::print(ofstream & file) const
{
  file << "RfNet:\n\n" 
       << "n_in: " << n_in << '\n'
       << "n_out: " << n_out  << '\n'
       << "diag_only: " << diag_only << '\n'
       << "meta: " << meta << '\n' 
       << "meta_rate: "  << meta_rate << '\n'
       << "penalty: "	<< penalty << '\n'
       << "init_alpha: "	<< init_alpha << '\n'
       << "norm: "	<< norm 
       << "norm_out: "	<< norm_out 
       << "n_data: "	<< n_data << '\n' 
       << "w_gen: " << w_gen << '\n' 		
       << "w_prune: " << w_prune << '\n'
       << "init_lambda: " << init_lambda << '\n'
       << "final_lambda: " << final_lambda << '\n'
       << "tau_lambda: " << tau_lambda << '\n'
       << "init_P: " << init_P << '\n'
       << "n_pruned: " << n_pruned << '\n'
       << "init_D: " << init_D << '\n'
       << "init_M" << init_M << '\n'
       << "mean_x: " << mean_x 
       << "var_x: " << var_x
       << "kernel: " << kernel <<  '\n'
       << "numero rfs: " << rfs.size() << '\n' << endl;

	 
  for (unsigned int i = 0; i <rfs.size(); i++)
    {	
      file << i + 1 << ")\t";
      rfs[i]->print(file);
      file << '\n'; 
    }  
	
  print();
}

void RFNet::SaveNet(const char * file_ini) 
{
  ofstream file;

  file.open(file_ini);

  file << "[RFNET]\n\n" 
       << "n_in  " << n_in << '\n'
       << "n_out  " << n_out  << '\n'
       << "diag_only  " << diag_only << '\n'
       << "meta  " << meta << '\n' 
       << "meta_rate  "  << meta_rate << '\n'
       << "penalty  "	<< penalty << '\n';

  file << "init_alpha  " ;

  outIni(init_alpha,file);

  file << "norm  " ;
  outIni(norm,file);
	
  file << "norm_out  " ;
  outIni(norm_out,file);
		  
  file  << "n_data  "	<< n_data << '\n' 
	<< "w_gen  " << w_gen << '\n' 		
	<< "w_prune  " << w_prune << '\n'
	<< "init_lambda  " << init_lambda << '\n'
	<< "final_lambda  " << final_lambda << '\n'
	<< "tau_lambda  " << tau_lambda << '\n'
	<< "init_P  " << init_P << '\n'
	<< "n_pruned  " << n_pruned << '\n';
          
  file  << "init_D  " ;
  outIni(init_D,file);

  file << "init_M  ";
  outIni(init_M,file);
          			
  file << "mean_x  " ;
  outIni(mean_x,file);
	
  file << "var_x  " ;
  outIni(var_x,file);
          
  file  << "kernel  " << kernel <<  '\n'
	<< "numero_rfs  " << rfs.size() << '\n' << endl;

	 
  for (unsigned int i = 0; i <rfs.size(); i++)
    {
      file << "[RF" << i + 1 << "]\n";
      rfs[i]->SaveRf(file);
      file << '\n'; 
    }  
	
  file << "[ENDINI]" << endl;

  file.close();
}

int RFNet::LoadNet(const char *file_ini, bool only_init)
{
  YARPConfigFile file;

  file.set(file_ini);
	

  int ris = file.get("[RFNET]", "n_in", &n_in, 1);
  // Viene eseguito solo questo primo controllo sull'apertura del file.

  if (ris == 0)
    {
      cout << "Impossibile aprire il file o trovare la var. indicata" << endl;
      return ris;
    }

  // Verifico che la rete da cui si vuole copiare i dati sia inizializzata.
	
  if (n_in == 0)
    {
      cout << " La rete che si vuole caricare non e' inizializzata. " << endl;
      return 0;	
    }
	
	
  file.get("[RFNET]", "n_out", &n_out, 1);

  int temp;
	
  file.get("[RFNET]", "diag_only", &temp, 1);
  diag_only = static_cast<bool>(temp);

  file.get("[RFNET]", "meta", &temp, 1);
  meta = static_cast<bool>(temp);

  file.get("[RFNET]", "meta_rate", &meta_rate, 1);
  file.get("[RFNET]", "penalty", &penalty, 1);
  file.get("[RFNET]", "init_alpha_num", &init_alpha_num, 1);

  int nn_in = n_in * n_in;
  double * array_in =  new double[n_in];
  double * array_out = new double[n_out];
  double * array_in_in = new double [nn_in];

  file.get("[RFNET]", "norm", array_in, n_in);
  norm.Resize(n_in,array_in);

  file.get("[RFNET]", "norm_out", array_out, n_out);
  norm_out.Resize(n_out,array_out);
    
  file.get("[RFNET]", "init_alpha", array_in_in, nn_in);
  init_alpha.Resize(n_in,n_in,array_in_in);

  file.get("[RFNET]", "init_D", array_in_in, nn_in);
  init_D.Resize(n_in,n_in,array_in_in);

  file.get("[RFNET]", "init_M", array_in_in, nn_in);
  init_M.Resize(n_in,n_in,array_in_in);

  if(only_init) 
    {
      mean_x.Resize(n_in);
      var_x.Resize(n_in);
      return ris;
    }

  // Proseguo a caricare gli altri dati solo se 
  // il bool only_init è settato a false (default).
	
  file.get("[RFNET]", "n_data", &n_data, 1);
  file.get("[RFNET]", "w_gen", &w_gen, 1);
  file.get("[RFNET]", "w_prune", &w_prune, 1);
  file.get("[RFNET]", "init_lambda", &init_lambda, 1);
  file.get("[RFNET]", "final_lambda", &final_lambda, 1);
  file.get("[RFNET]", "tau_lambda", &tau_lambda, 1);
  file.get("[RFNET]", "init_P", &init_P, 1);
  file.get("[RFNET]", "n_pruned", &n_pruned, 1);
	
  file.get("[RFNET]", "mean_x", array_in, n_in);
  mean_x.Resize(n_in,array_in);

  file.get("[RFNET]", "var_x", array_in, n_in);
  var_x.Resize(n_in,array_in);

  char k[255];
   
  file.getString("[RFNET]", "kernel",k);
  kernel = k;
   
  delete [] array_in;
  delete [] array_out;
  delete [] array_in_in;

  int rfs_size;
  file.get("[RFNET]", "numero_rfs", &rfs_size, 1);

  Rf * rf;
  char title[255];

  for(int i = 1; i <= rfs_size; i++)
    {
      rf = new Rf;
      sprintf(title, "[RF%d]", i );
      ris = rf->LoadRf(file,title,n_in,n_out);
      rfs.push_back (rf);
    }

  xn.Resize(n_in);
  yp_i.Resize(n_in);
  _tempX.Resize(n_in);
  yn.Resize(n_out);
  prediction.Resize(n_out);
  _tempY.Resize(n_out);

  return ris;
}

