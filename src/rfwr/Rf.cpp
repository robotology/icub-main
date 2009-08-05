#include "Rf.h"

//-------------------------------------------------
// Costruttori
//-------------------------------------------------

using namespace std;

Rf::Rf(void):D(1,1), M(1,1), alpha(1,1), b0(1), P(1,1),B(1,1), H(1,1),R(1,1), h(1,1), b(1,1), c(1),vif(1),mean_x(1),var_x(1), tempX2(1), tempX(1), diagD(1)
{	
  mean_x = 0;
  var_x = 0;

  init();
}

Rf::Rf(const Rf & templ):D(templ.D), M(templ.M), alpha(templ.alpha), b0(templ.b0), P(templ.P),B( templ.B), H( templ.H),R(templ.R), h(templ.h), b(templ.b), c(templ.c),vif(templ.vif),mean_x(templ.mean_x),var_x(templ.var_x), tempX(templ.tempX), tempX2(templ.tempX2), diagD(templ.diagD)
{
  sum_w = templ.sum_w;
  sum_e_cv2 = templ.sum_e_cv2;
  sum_e2 = templ.sum_e2;
  n_data = templ.n_data;
  trustworthy = templ.trustworthy;
  lambda = templ.lambda;
	 
  w = templ.w;

  erased = templ.erased ;
}

Rf::Rf(const YVector &myc, const YMatrix &myD):c(myc),D(myD), M(1,1), alpha(1,1), b0(1), P(1,1),B(1,1), H(1,1),R(1,1), h(1,1), b(1,1),vif(1),mean_x(1),var_x(1), tempX(1), tempX2(1), diagD(1)
{
  init();
}

//-------------------------------------------------
//  Funzioni di inizializzazione
//-------------------------------------------------

void Rf::init(void)
{
  sum_w = 0;
  sum_e_cv2 = 0;
  sum_e2 = 0;
  n_data = 0;
  trustworthy = 0;
  lambda = 0;
  w = 0;
  erased = 0;
}


void Rf::init(Rf * templ, const YVector &myc, const YVector &myy, int n_in, int n_out, double init_P, double init_lambda)
{
  D = templ->D;
  M = templ->M;
  tempX2.Resize(D.NRows());
  diagD.Resize(D.NRows());

  alpha = templ->alpha; 
  b0 = myy;

  P = identity(n_in) * init_P;
  B.Resize(n_in,n_out);
  H.Resize(n_in,n_out);
  R.Resize(n_in,n_in);
  h.Resize(alpha.NRows(),alpha.NCols());
  b = log(alpha+0.0000000001);
  c = myc;
  vif = onesV(n_in) * 1000000000;
  sum_w = 0;
  sum_e_cv2 = 0;
  sum_e2 = 0;
  n_data = 0;
  trustworthy = 0;
  lambda = init_lambda;
  w = 0;

  mean_x.Resize(n_in);
  var_x.Resize(n_in);
  tempX.Resize(n_in);

  erased = templ->erased;
}

void Rf::init(const YVector &myc, const YVector &myy, const YMatrix &init_D, const YMatrix &init_M, const YMatrix &init_alpha, int n_in, int n_out, double init_P, double init_lambda)
{
  D = init_D;
  M = init_M;
  tempX2.Resize(D.NRows());
  diagD.Resize(D.NRows());

  alpha = init_alpha; 
  b0 = myy;

  P = identity(n_in) * init_P;
  B.Resize(n_in,n_out);
  H.Resize(n_in,n_out);
  R.Resize(n_in,n_in);
  h.Resize(alpha.NRows(),alpha.NCols());
  b = log(alpha+0.0000000001);
  c = myc;
  vif = onesV(n_in) * 1000000000;
  sum_w = 0;
  sum_e_cv2 = 0;
  sum_e2 = 0;
  n_data = 0;
  trustworthy = 0;
  lambda = init_lambda;
  w = 0;
	
  mean_x.Resize(n_in);
  var_x.Resize(n_in);
  tempX.Resize(n_in);
	
  erased = 0;
}

//-------------------------------------------------
// Funzione di eliminazione dell'Rf
//-------------------------------------------------

// Il numero passato in argomento sarà indice dell'ordine di pruning.

void Rf::cancel(int num)
{
  D = 0;
  M = 0;
  alpha = 0;
  b0 = 0;

  P = 0;
  B = 0;
  H = 0;
  R = 0;
  h = 0;
  b = 0;
  c = 0;
  vif = 0;

  sum_w = 0;
  sum_e_cv2 = 0;
  sum_e2 = 0;
  n_data = 0;
  trustworthy = 0;
  lambda = 0;
  w = 0;

  mean_x = 0;
  var_x = 0;

  erased = num;

}

//-------------------
// Calcola il peso
//-------------------

double Rf::compute_weight(bool diag_only, const string &kernel, const YVector &x)
{
  double d2;
	
  tempX = x - c;
	
  if (diag_only)		
    {
      diag(D, diagD);
      times(diagD, tempX, tempX2);
      
      d2 = tempX * tempX2;
    }      
  else
    d2 =(D * tempX) * tempX;
	
  if (kernel == "Gaussian")	
    w=exp(-0.5*d2);
	
  else if (kernel == "BiSquare")
    if ((0.5 * d2) > 1)		
      w = 0;
    else
      w = pow((1-0.5*d2),2);

  return w;			
}

//-----------------------------------------------------
//  Aggiorna le medie e calcola variabili a media zero.
//-----------------------------------------------------

void Rf::update_means(const YVector &xn, const YVector &yn, double myw)
{
  mean_x = (sum_w * mean_x * lambda + myw*xn) / (sum_w * lambda + myw);
  square(xn-mean_x, tempX);
  var_x = (sum_w * var_x * lambda + myw * tempX) / (sum_w * lambda + myw);
  b0 =  (sum_w * b0 * lambda + myw*yn) / (sum_w * lambda + myw);

  xmz = xn - mean_x;
  ymz = yn - b0;
}


//--------------------------------------------------------------
//  Aggiorna i parametri della regressione lineare ed il modello
//--------------------------------------------------------------
YVector Rf::update_regression(double myw)
{
  YVector Px = P*xmz;
  YMatrix Pxc = conversionC(Px); // 3x1 (Px)= 3x3 * 3x1
  YMatrix xPr = conversionR(Px); // 1x3 (xP)  xP = Px';
  YVector xP = conversion(xPr); 
 
  e_cv   = ymz - B.Transposed() * xmz; // e_cv(n_out, 1) perche B(n_in,n_out) e x(n_in,1)
  P      = (P - Pxc*xPr/(lambda/myw + xP*xmz))/lambda;
  
  // Dal momento che ho bisogno di realizzare un prodotto vettore tra 
  // (P*x) (n_in, 1) e e_cv(n_out,1) dopo averlo trasposto, per ottenere
  // la matrice B(n_in,n_out) trasformo e_cv in un matrice e lo stesso faccio per (P * x)
  
  YMatrix e_cvMat = conversionR(e_cv); 
  Pxc = conversionC(P * xmz);

  B = B + myw * (Pxc) * e_cvMat;
  
  // the new predicted output after updating
  YVector yp_i = B.Transposed() * xmz;
  e  = ymz - B.Transposed() * xmz;
  yp_i = yp_i + b0;

  // is the RF trustworthy: use variance inflation factor to judge
  times(diag(P),var_x,vif);

  if (maximum(vif) < 10 && n_data > xmz.Length() * 2)
    trustworthy = 1;
	
  return yp_i; 
}


//-----------------------------------------------------------------------------
// Update the distance metric.
//-----------------------------------------------------------------------------

double Rf::update_distance_metric(double myw,YVector xn, double pen, bool meta, double meta_rate, string kernel, bool diag_only)
{ 
  double transient_multiplier ;

  if (minimum(vif) > 5)
      return 0 ;

  double penalty = pen/xmz.Length(); // normalizes penality w.r.t. number of inputs
  
  //penalty   = sdcs(ID).penalty/length(x); // normalizes penality w.r.t. number of inputs
  //meta      = sdcs(ID).meta;
  //meta_rate = sdcs(ID).meta_rate;
  //kernel    = sdcs(ID).kernel;
  //diag_only = sdcs(ID).diag_only;
  
  // useful pre-computations: they need to come before the updates

  double e_cv2 = e_cv * e_cv;
  double e2    = e * e;
  
  // Sarebbe h nel testo, ma per non confonedrlo con rf.h lo chiamo hh. Risulta essere un numero.
  double  hh = myw * xmz * (P * xmz);

  double W = sum_w * lambda + myw;
  sum_e_cv2 = sum_e_cv2 * lambda + myw * e_cv2;
  sum_e2    = sum_e2 * lambda + myw * e2;
  double E  = sum_e_cv2;
  int  n_out = ymz.Length();

  transient_multiplier = pow((sum_e2/(sum_e_cv2 + 0.0000000001)),4); // this is a numerical safety heuristic
  // the derivative dJ1/dw
  YVector Px    = P * xmz; // Px ( 3x1)
  
  YMatrix et = conversionR(e);  
  YMatrix Pxr = conversionR(Px);
  YMatrix Pxc = conversionC(Px);

  YMatrix Pxe   = Pxc * et; 
  //e ha le dim di y --> (nrighe x 1)----> e' (1xnrighe) ---> Pxe (3xnrighe)

  double dJ1dw = -E/(W*W) + (1/W) * ( e_cv2 - sumTot(2 * times(Pxe,H)) - sumTot(times(2*Pxc*Pxr,R)));
 
  YMatrix  dwdM, dJ2dM, dwwdMdM, dJ2J2dMdM;

  // the derivatives dw/dM and dJ2/dM
  dist_derivatives(myw,xn-c,diag_only,kernel,penalty,meta,dwdM, dJ2dM, dwwdMdM, dJ2J2dMdM);

  // the final derivative becomes (note this is upper triangular)
  
  YMatrix dJdM = dwdM * dJ1dw/n_out + (myw/W) * dJ2dM;

  YMatrix xr = conversionR(xmz);
  // the second derivative if meta learning is required, and meta learning update
  if (meta)
    {
      double dJ1J1dwdw = -e_cv2/(W*W) - (2/W) * sumTot(-Pxe/W - times(2*Pxc*(xr *Pxe),H) ) + (2/W) * e2 * (hh/myw) - 1/(W*W) * (e_cv2 - 2 * sumTot(times(Pxe,H))) + 2*E/(W*W*W);
    
      YMatrix dJJdMdM = (dwwdMdM * dJ1dw + square(dwdM) * dJ1J1dwdw)/n_out + (myw / W) * dJ2J2dMdM;
    
      // update the learning rates
      YMatrix aux = meta_rate * transient_multiplier * times(dJdM,h); // aux (3x3)
    
      // limit the update rate
      if (maximum(abs(aux)) > 0.1)
    	  ReplSup(aux,0.1);
		
      b = b - aux;
    
      // prevent numerical overflow

      if (maximum(abs(b)) > 10)
	  ReplSup(b,10);

      exp(b,alpha);
    
      aux =  - times(alpha,dJJdMdM) * transient_multiplier + 1;
   
      aux = PartPos(aux);
			
      h = times(h,aux) - times(alpha,dJdM) * transient_multiplier;
    
    } // end if(meta)

  // update the distance metric, use some caution for too large gradients

  double maxM = maximum(abs(M));
  YMatrix delta_M = times(alpha,dJdM) * transient_multiplier;
  
  // Per adesso uso 2 for, poi si vedrà:
  for (int i=1; i<=delta_M.NRows(); i++)
    for(int j=1; j<=delta_M.NCols(); j++)
      if( delta_M(i,j) > 0.1*maxM)
	{
	  alpha(i,j) = alpha (i,j)/2;
	  delta_M(i,j) = 0;
	  cout << "Reduced learning rate" << endl;
	}

  M = M - times(alpha,dJdM) * transient_multiplier;
  D = M.Transposed() * M;

  // update sufficient statistics: note this must come after the updates

  // e_cv ha le dimensioni di y --> vett (nrighey x 1)

  YMatrix xc,e_cvr;
  xc = conversionC(xmz);
  e_cvr = conversionR(e_cv);

  H = lambda * H + (myw/(1-hh)) * xc * e_cvr * transient_multiplier;
  R = lambda * R + (myw * myw * e_cv2/(1-hh))*(xc*xr) * transient_multiplier;

  return transient_multiplier;
}


//-----------------------------------------------------------------------------
// Compute derivatives of distance metric: note that these will be upper
// triangular matrices for efficiency
//-----------------------------------------------------------------------------

void Rf::dist_derivatives(double myw,YVector dx,bool diag_only,string kernel,double penalty, bool meta,YMatrix & dwdM,YMatrix & dJ2dM,YMatrix & dwwdMdM,YMatrix & dJ2J2dMdM)
{
  int n_in = dx.Length();
  dwdM.Resize(n_in,n_in);
  dJ2dM.Resize(n_in,n_in);
  dJ2J2dMdM.Resize(n_in,n_in);
  dwwdMdM.Resize(n_in,n_in);

  int n,m,i;
  double sum_aux, sum_aux1;
  double aux;
   
  for(n=1; n <= n_in; n++)
    {
      for(m = n; m <= n_in; m++)
	{
	  sum_aux    = 0;
	  sum_aux1   = 0;
      
	  // take the derivative of D with respect to nm_th element of M 
      
	  if (diag_only && n==m)
	    {	
	      aux = 2 * M(n,n);
	      dwdM(n,n) = pow(dx(n),2) * aux;
	      sum_aux = D(n,n) * aux;
		
	      if (meta) 
		{
		  sum_aux1 = sum_aux1 + aux * aux; 
		}
	    }
	 
	  else if (!diag_only)
	    {
	      for (i=n; i <= n_in; i++)
		{
	  
		  // aux corresponds to the in_th (= ni_th) element of dDdm_nm 
		  // this is directly processed for dwdM and dJ2dM
	  
		  if (i == m)
		    {
		      aux = 2 * M(n,i);
		      dwdM(n,m) = dwdM(n,m) + dx(i) * dx(m) * aux;
		      sum_aux = sum_aux + D(i,m) * aux;
					
		      if (meta) 
			{
			  sum_aux1 = sum_aux1 + aux*aux;
			}
				
		    }
			
		  else
		    {
		      aux = M(n,i);
		      dwdM(n,m) = dwdM(n,m) + 2 * dx(i) * dx(m) * aux;
		      sum_aux = sum_aux + 2 * D(i,m) * aux;
						
		      if (meta)
			{
			  sum_aux1 = sum_aux1 + 2*aux*aux;
			}
		    }
		} // chiuso for centrale
	    } // chiuso else if
		 
	  if( kernel == "Gaussian")  
	    dwdM(n,m)  = -0.5 * w * dwdM(n,m);
	  else if( kernel == "BiSquare")     
	    dwdM(n,m)  = -sqrt(w) * dwdM(n,m);
	  dJ2dM(n,m)  = 2 * penalty * sum_aux;
	  if (meta)
	    {
	      dJ2J2dMdM(n,m) = 2 * penalty *( 2 * D(m,m) + sum_aux1);
	      dJ2J2dMdM(m,n) = dJ2J2dMdM(n,m);
	
	      if( kernel == "Gaussian")  
		dwwdMdM(n,m)   = pow(dwdM(n,m),2)/w - w * pow(dx(m),2);
	      else if( kernel == "BiSquare")     
		dwwdMdM(n,m)   = (pow(dwdM(n,m),2)/w)/2 - 2 * sqrt(w) * pow(dx(m),2);

	      dwwdMdM(m,n)   = dwwdMdM(n,m);
	    }
      
 	} // chiuso for
    }// chiuso for
}

//--------------------------------------------------------------------------------
// Aggiorna le variabili statistiche.
//--------------------------------------------------------------------------------

void Rf::update_statistical(double myw, double mytau, double myfin)
{
  sum_w = sum_w * lambda + myw;
  n_data = n_data * lambda +1;
  lambda = mytau * lambda + myfin * (1 - mytau);
}

void Rf::print() const
{

  cout << "Rf:\n\n" 
       << "D: " << D  << '\n'
    //		  << "sumD: " << sumTot(D) << "\n\n";
       << "M: " << M  << '\n'
       << "alpha: " << alpha << '\n'
       << "b0: " << b0 
       << "P: "  << P << '\n'
       << "B: "	<< B << '\n'
       << "H: "	<< H << '\n'
       << "R: "	<< R << '\n'
       << "h: "	<< h << '\n'
       << "b: "	<< b << '\n'
       << "c: "	<< c 
       << "vif: " << vif 		
       << "sum_w: " << sum_w << '\n'
       << "sum_e_cv2: " << sum_e_cv2 << '\n'
       << "sum_e2: " << sum_e2 << '\n'
       << "n_data: " << n_data << '\n'
       << "trustworthy: " << trustworthy << '\n'
       << "lambda: " << lambda << '\n'
       << "mean_x: " << mean_x 
       << "var_x: " << var_x 
       << "w: " << w << '\n'
       << "erased: " << erased << "\n\n";
	

}

// Funzione di stampa su file. Stampa contemporaneamente anche a schermo!

void Rf::print(ofstream & file) const
{
  file << "Rf:\n\n" 
       << "D: " << D  << '\n'
    //	      << "sumD: " << sumTot(D) << "\n\n";
       << "M: " << M  << '\n'
       << "alpha: " << alpha << '\n'
       << "b0: " << b0 
       << "P: "  << P << '\n'
       << "B: "	<< B << '\n'
       << "H: "	<< H << '\n'
       << "R: "	<< R << '\n'
       << "h: "	<< h << '\n'
       << "b: "	<< b << '\n'
       << "c: "	<< c 
       << "vif: " << vif 		
       << "sum_w: " << sum_w << '\n'
       << "sum_e_cv2: " << sum_e_cv2 << '\n'
       << "sum_e2: " << sum_e2 << '\n'
       << "n_data: " << n_data << '\n'
       << "trustworthy: " << trustworthy << '\n'
       << "lambda: " << lambda << '\n'
       << "mean_x: " << mean_x 
       << "var_x: " << var_x 
       << "w: " << w  << '\n'
       << "erased: " << erased << "\n\n";
	
  print();
}


// Funzione creata per le modifiche apportate da composite_control.
// Rem: comp_alpha sarebbe in realtà alpha , così ridenom per non confonderla
// con l'alpha degli Rf, idem per comp_sum_w. tmsj è il j-esimo elem di tms in RFnet.cpp

void Rf::update_composite(double comp_alpha, double tmsj, double comp_sum_w, YVector xn)

{
  B  = B + comp_alpha * tmsj * P * w/comp_sum_w * (xn-c) * e;
  b0 = b0 + comp_alpha * tmsj / sum_w * w / comp_sum_w * e;
}

void Rf::SaveRf(ostream & file) 
{

  file  << "D  " ;    
  outIni(D,file);
	
  file  << "M  " ;
  outIni(M,file);
		  
  file << "alpha  " ;
  outIni(alpha,file);
		  
  file << "b0  " ;
  outIni(b0,file);
	  
  file << "P  " ;
  outIni(P,file);
	  
  file << "B  " ;
  outIni(B,file);
	  
  file << "H  " ;
  outIni(H,file);
	  
  file << "R  " ;
  outIni(R,file);
	  
  file << "h  " ;
  outIni(h,file);
	  
  file << "b  " ;
  outIni(b,file);
	  
  file << "c  " ;
  outIni(c,file);
	  
  file << "vif  " ;
  outIni(vif,file);
		  
	
  file  << "sum_w  " << sum_w << '\n'
	<< "sum_e_cv2  " << sum_e_cv2 << '\n'
	<< "sum_e2  " << sum_e2 << '\n'
	<< "n_data  " << n_data << '\n'
	<< "trustworthy  " << trustworthy << '\n'
	<< "lambda  " << lambda << '\n';

  file << "mean_x	 " ;
  outIni(mean_x,file);
	
  file << "var_x  " ;
  outIni(var_x,file);
          
  file << "w  " << w  << '\n'
       << "erased  " << erased << "\n\n";
}

int Rf::LoadRf(YARPConfigFile & file,char * title, int n_in, int n_out)
{
  int nn_in = n_in * n_in;
  int nn_io = n_in * n_out;
  double * a_in_in = new double[nn_in];
  double * a_in_out = new double [nn_io];
  double * a_out = new double[n_out];
  double * a_in = new double[n_in];

  int ris = file.get(title, "D", a_in_in, nn_in);

  if (ris == 0)
    {
      cout << "Impossibile trovare l'RF indicato" << endl;
      return ris;
    }
  
  D.Resize(n_in,n_in,a_in_in);
  file.get(title, "M",a_in_in, nn_in);

  M.Resize(n_in,n_in,a_in_in);
  file.get(title, "alpha",a_in_in, nn_in);

  alpha.Resize(n_in,n_in,a_in_in);

  file.get(title, "b0",a_out, n_out);
  b0.Resize(n_out,a_out);

  file.get(title, "P",a_in_in, nn_in);
  P.Resize(n_in,n_in,a_in_in);

  file.get(title, "B",a_in_out, nn_io);
  B.Resize(n_in,n_out,a_in_out);

  file.get(title, "H",a_in_out, nn_io);
  H.Resize(n_in,n_out,a_in_out);

  file.get(title, "R",a_in_in, nn_in);
  R.Resize(n_in,n_in,a_in_in);
	
  file.get(title, "h",a_in_in, nn_in);
  h.Resize(n_in,n_in,a_in_in);
	
  file.get(title, "b",a_in_in, nn_in);
  b.Resize(n_in,n_in,a_in_in);
	
  file.get(title, "c",a_in, n_in);
  c.Resize(n_in,a_in);

  file.get(title, "vif",a_in, n_in);
  vif.Resize(n_in,a_in);

  file.get(title, "sum_w",&sum_w, 1);
  file.get(title, "sum_e_cv2",&sum_e_cv2, 1);
  file.get(title, "sum_e2",&sum_e2, 1);
  file.get(title, "n_data",&n_data, 1);

  int temp;
  file.get(title,"trustworthy",&temp,1);
  trustworthy = bool(temp);

  file.get(title, "lambda",&lambda, 1);
  file.get(title, "erased",&erased, 1);
	
  file.get(title, "mean_x",a_in, n_in);
  mean_x.Resize(n_in,a_in);

  file.get(title, "var_x",a_in, n_in);
  var_x.Resize(n_in,a_in);
  
  file.get(title, "w",&w, 1);

  tempX.Resize(n_in);
  tempX2.Resize(D.NRows());
  diagD.Resize(D.NRows());

  delete [] a_in_in ;
  delete [] a_in_out;
  delete [] a_out;
  delete [] a_in;

  return ris;
}
