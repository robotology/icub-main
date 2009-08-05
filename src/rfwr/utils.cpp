#include "RFUtils.h"

//--------------------------------------------------
// Crea matrice identità quadrata di dimensione dim.
//--------------------------------------------------

using namespace std;

YMatrix identity(int dim)
{
  YMatrix ident(dim,dim);

  for(int i=1;i<=dim;i++)
    ident(i,i) = 1.0;

  return ident;	
}

//--------------------------------------------------
// Crea vettore di 1 di dimensione dim.
//--------------------------------------------------

YVector onesV(int dim)
{
  YVector v(dim);
  v = 1.0;
  return v;
}

//--------------------------------------------------
// Crea matrice di 1 quadrata di dimensione dim.
//--------------------------------------------------

YMatrix ones(int dim)
{
  YMatrix o(dim,dim);

  o = 1.0 ;

  return o;	
}

//--------------------------------------------------
// Crea matrice di 1 di dimensioni dim1, dim2.
//--------------------------------------------------

YMatrix ones(int dim1, int dim2)
{
  YMatrix o(dim1,dim2);

  o = 1.0 ;

  return o;	
}

//--------------------------------------------------
// Dato un vettore, crea una matrice che ha i valori
// di quel vettore sulla diagonale.
//--------------------------------------------------

YMatrix diag(const YVector &val)
{
  int dim = val.Length();

  YMatrix m(dim,dim);

  for(int i=1;i<=dim;i++)		
    m(i,i) = val(i);
	
  return m;
}

void diag(const YVector &val, YMatrix &m)
{
  assert(m.NRows()==m.NCols() && m.NCols()==val.Length());

  int dim=val.Length();

  for(int i=1;i<=dim;i++)
    m(i,i)=val(i);
}

//-------------------------------------------------------
// Data una matrice, ne estrae la diagonale in un vettore
//-------------------------------------------------------

YVector diag(const YMatrix &m)
{
  assert(m.NCols() == m.NRows());

  int n = m.NCols();

  YVector y(n);

  for (int i=0; i<n; i++)
    y.data()[i]= m.data()[i][i];
	
  return y;
}

void diag(const YMatrix &m, YVector &res)
{
  assert(m.NCols() == m.NRows() && res.Length() == m.NRows());

  int n = m.NCols();

  for (int i=0; i<n; i++)
    res.data()[i] = m.data()[i][i];
}

//------------------------------------------------------------------------
// Data una matrice con una dimensione unitaria, la converte in un vettore
//------------------------------------------------------------------------

YVector conversion(const YMatrix &m)
{
  YVector v;
  if ( m.NCols() == 1)
    v.Resize(m.NRows(),*m.data());
  else if (m.NRows() == 1)
    v.Resize(m.NCols(),*m.data());

  return v;
}

//-------------------------------------------------------
// Dato un vettore lungo n, lo trasforma in una matrice di 
// dimensioni (1,n) 
//-------------------------------------------------------

YMatrix conversionR(const YVector &v)
{ 
  YMatrix m(1,v.Length(),v.data());
  return m;
}

//-------------------------------------------------------
// Dato un vettore lungo n, lo trasforma in una matrice di 
// dimensioni (n,1) 
//-------------------------------------------------------

YMatrix conversionC(const YVector &v)
{ 
  YMatrix m(v.Length(),1,v.data());
  return m;
}

//-------------------------------------------------------
// Ritorna la matrice ottenuta calcolando il prod. elemento
// per elemento di due altre matrici di uguali dim.
//-------------------------------------------------------

YMatrix times(const YMatrix &m1, const YMatrix &m2)
{
  assert(m1.NCols() == m2.NCols() && m1.NRows() == m2.NRows());

  YMatrix mret(m1.NRows(),m1.NCols());
  int i,j;

  for(i=0; i < m1.NRows(); i++)
    for(j=0; j < m1.NCols(); j++)
      mret[i][j] = m1[i][j]*m2[i][j];

  return mret;
}

//---------------------------------------------------------
// Ritorna il vettore ottenuto calcolando il prod. elemento
// per elemento di due altri vettori di uguale lunghezza.
//---------------------------------------------------------

void times(const YVector &v1, const YVector &v2, YVector &res)
{
  int n=v1.Length();
  assert(v1.Length()==v2.Length());
  assert(res.Length()==v1.Length());

  for (int k=0;k<n;k++)
    res[k]=v1[k]*v2[k];
}

void times(const YMatrix &m1, const YMatrix &m2, YMatrix &res)
{
  assert(m1.NRows()==m2.NRows() && m1.NCols()==m2.NCols() && res.NRows()==m1.NRows() && res.NCols()==m1.NCols());

  int i,j;

  for(i=0; i < m1.NRows(); i++)
    for(j=0; j < m1.NCols(); j++)
      res[i][j] = m1[i][j]*m2[i][j];
}

//---------------------------------------------------------
// Ritornano il vettore o la matrice, ottenuti calcolando 
// il rapp. elemento per elemento di due altri vettori o di
// due altre matrici di uguali dimensioni.
//---------------------------------------------------------

void ratio(const YVector &v1, const YVector &v2, YVector &res)
{
  int n=v1.Length();
  assert(v1.Length()==v2.Length());
  assert(res.Length()==v1.Length());

  for (int k=0;k<n;k++)
    {
      assert(v2[k]!=0);
      res[k]=v1[k]/v2[k];
    }
}

void ratio(const YMatrix &m1, const YMatrix &m2, YMatrix &res)
{
  assert(m1.NCols()==m2.NCols()&& m1.NRows() == m2.NRows());

  for (int r=1; r<=m1.NRows(); r++)
    for (int c=1; c<=m1.NCols(); c++)
      {
	assert(m2(r,c)!=0);
	res(r,c)=m1(r,c)/m2(r,c);
      }
}

//------------------------------------------------------------
// Ritornano il vettore o la matrice ottenuti calcolando il 
// quadrato elemento per elemento di un vettore o di una mat.
//------------------------------------------------------------

void square(const YVector &v1, YVector &res)
{
  times(v1,v1,res);
}

void square(const YMatrix &m, YMatrix &res)
{
  times(m,m,res);
}

YMatrix square(const YMatrix &m)
{
  YMatrix ret(m.NRows(), m.NCols());
  square(m, ret);
  return ret;
}

YVector square(const YVector &v)
{
  YVector ret(v.Length());
  square(v, ret);
  return ret;
}

//------------------------------------------------------------
// Ordina un vettore in modo decrescente
//------------------------------------------------------------

void sortGr(YVector &v)
{
  sort(v.data(),v.data()+ v.Length(), greater <double> ());
}

//------------------------------------------------------------
// Estrae la riga num dalla matrice m in un vettore
//------------------------------------------------------------

YVector Row(const YMatrix &m, int num)
{
  assert(num <= m.NRows());
  const double **rp=m.data();
  
  YVector row(m.NCols(),rp[num-1]);//*(m.data()+(num-1)));
  return row;
}

//------------------------------------------------------------
// Estrae la colonna num dalla matrice m in un vettore
//------------------------------------------------------------

YVector Col(const YMatrix &m, int num)
{
  // postmoltiplico per un vett della base canonica
	
  assert(num <= m.NCols());

  YVector bc(m.NCols());
  bc(num)=1;	
  YVector col = m*bc;
  return col;
}

//------------------------------------------------------------
// Calcolano il massimo elemento di un vettore o di una matrice.
//------------------------------------------------------------

double maximum (const YVector &v)
{
  return *( max_element(v.data(), v.data() + v.Length()) );
}

double maximum(const YMatrix &m)
{
  return *( max_element(m.data()[0], m.data()[0]+(m.NRows()*m.NCols())) );
}


//------------------------------------------------------------------------
// Versione con divisione in colonne (utile per avere max su ogni colonna)
//------------------------------------------------------------------------

YVector max_col(YMatrix m)

{
  YVector v(m.NCols());
	
  for(int i=1; i<=m.NCols(); i++)
    v(i) = maximum(Col(m,i));

  return v;
}

//------------------------------------------------------------
// Calcolano il minimo elemento di un vettore o di una matrice.
//------------------------------------------------------------

double minimum (const YVector &v)
{
  return *( min_element(v.data(), v.data() + v.Length()) ) ;
}

double minimum(const YMatrix &m)
{
  return *( min_element(m.data()[0], m.data()[0] + (m.NRows()*m.NCols())) );
}

//---------------------------------------------------------------
// Calcola la somma degli elementi di un vettore o di una matrice.
//---------------------------------------------------------------

double sumTot(const YVector& v)
{
  return accumulate(v.data(),v.data() + v.Length(), 0.0);
}


double sumTot(const YMatrix &m)
{
  return accumulate(*m.data(), *m.data() + m.NCols()*m.NRows(),0.0);
}

//---------------------------------------------------------------
// Calcola l'esponenziale, elemento per elemento, di una matrice.
//---------------------------------------------------------------

void exp (const YMatrix &m, YMatrix &res)
{
  assert(m.NRows()==res.NRows() && m.NCols()==res.NCols());

  for(int i=0; i<m.NRows(); i++)
    for(int j=0; j<m.NCols(); j++)
      res.data()[i][j] = exp(m.data()[i][j]);
}

//-------------------------------------------------------------------
// Calcola il valore assoluto, elemento per elemento, di una matrice.
//-------------------------------------------------------------------

YMatrix abs (const YMatrix &m)
{
  YMatrix ret;
  ret.Resize(m.NRows(), m.NCols());

  for(int i=0; i<m.NRows(); i++)
    for(int j=0; j<m.NCols(); j++)
      ret.data()[i][j] = fabs(m.data()[i][j]);

  return ret;
}

//---------------------------------------------------------------
// Calcola il logaritmo, elemento per elemento, di una matrice.
//---------------------------------------------------------------

// Non fa controlli di positività!

YMatrix log (const YMatrix &m)
{
  YMatrix ret;
  ret.Resize(m.NRows(), m.NCols());

  for(int i=0; i<m.NRows(); i++)
    for(int j=0; j<m.NCols(); j++)
      ret.data()[i][j] = log(m.data()[i][j]);

  return ret;
}

//---------------------------------------------------------------
// Calcolano la parte positiva e negativa di una matrice.
//---------------------------------------------------------------

YMatrix PartPos(const YMatrix &m)
{
  return (m+abs(m))/2;
}

YMatrix PartNeg(const YMatrix &m)
{
  return (m-abs(m))/2;
}

//------------------------------------------------------------------
// Calcolano il prodotto degli elementi del vettore o della matrice.
//------------------------------------------------------------------

// Ritorna 0 se il vettore contiene zeri, e cmq ritorna sempre il prod di tutti gi elem.

double prod(const YVector &v)
{
  return accumulate(v.data(),v.data()+ v.Length(), 1.0, multiplies <double> ());
}


double prod(const YMatrix &m)
{
  return accumulate(*m.data(), *m.data() + m.NCols()*m.NRows(),1.0, multiplies <double> ());
}

//------------------------------------------------------------------
// Ritorna una matrice fatta di 1 (in corrispondenza di num pos in m)
// e di -1 (in corrispondenza di num neg in m)
//------------------------------------------------------------------

YMatrix segno(const YMatrix &m)
{
  YMatrix ret(m.NRows(), m.NCols());
  for (int r=1; r<=m.NRows(); r++)
    for (int c=1; c<=m.NCols(); c++)
      {
	if (m(r,c)>=0)
	  ret(r,c)=1;
	else
	  ret(r,c)=-1;
      }

  return ret;
}

//-------------------------------------------------------------------
// Sostituisce agli elementi di una matrice superiori in modulo ad un 
// certo valore tale valore, conservandone il segno originale.
//-------------------------------------------------------------------

void ReplSup(YMatrix &m, double val)
{
  for (int i=0; i<m.NCols()*m.NRows(); i++)
    if (*(*m.data() + i)  > val)
      *(*m.data() + i) = val;
    else if ( -*(*m.data() + i) < -val)
      *(*m.data() + i) = -val;
}

//-----------------------------------------------------------------
// Cholesky factorization: trasforma a - def. pos. - in una matrice 
// triangolare sup tale che aT * a = a originale.
//-----------------------------------------------------------------
/*
  Commento della versione originale tratta da Numerical Recipes

  Given a positive-de.nite symmetric matrix a[1..n][1..n], this routine constructs its Cholesky
  decomposition, A = L · LT . On input, only the upper triangle of a need be given; it is not
  modi.ed. The Cholesky factor L is returned in the lower triangle of a, except for its diagonal
  elements which are returned in p[1..n].
*/
void choldc(YMatrix & a, int n)
{
  int i,j,k;
  double sum;
  double * p;
  p = new double[n+1];
  for (i=1;i<=n;i++) 
    {
      for (j=i;j<=n;j++) 
	{
	  for (sum=a(i,j),k=i-1;k>=1;k--) sum -= a(i,k)*a(j,k);
	  if (i == j) 
	    {
	      if (sum <= 0.0) //a, with rounding errors, is not positive de.nite.
		{
		  cout << "errore!" << endl;
		  a = 0;
		  return;
		}
	      p[i]=sqrt(sum);
	      a(i,i) = p[i]; 
	    } 
	  else
	    {
	      a(j,i)=sum/p[i];
	      a(i,j) = 0;
	    }
			
	}
    }

  a.Transpose();
  delete [] p;
}

//--------------------------------------------------------------------------------
// Ritorna il numero di occorrenze (val. vett > val) trovate nel vettore v e salva
// in inds[] gli indici corrisp. Anche in inds ragiona a partire da 1.
// E' consigliabile allocare un vettore di dim pari a v.Length()
// Non è per nulla ottimizzato!
//--------------------------------------------------------------------------------

int findMag(const YVector &v, double val, int *inds)
{
  assert(inds!=0);
  int i,j=1;

  for (i=1; i<= v.Length(); i++)
    if ( v(i) > val)
      inds[j++] = i;
	
  return j-1;
}

//-----------------------------------------------------------------
// Funzioni utili per l'output su file ini di vettori e matrici.
//-----------------------------------------------------------------

void outIni(const YVector &v, ostream & file)
{
  for (int i=0; i<v.Length(); i++)
    file << v[i] << ' ' ;
  file << endl;
}

void outIni(const YMatrix &m, ostream & file)
{
  for (int i=0; i<m.NCols()*m.NRows(); i++)
    file << *(*m.data() + i) << ' ' ;
  file << endl;
}


//---------------------------------------------------
// Calcola la media campione : somma N elementi / N.
//---------------------------------------------------
double mean(const YVector &v)
{
  return (accumulate(v.data(),v.data() + v.Length(), 0.0)/ v.Length());
}

//-----------------------------------------------------------
// Calcola la varianza campionaria:  somma N (x-xm)^2 / N-1.
//-----------------------------------------------------------

double var(const YVector& v)
{
  YVector temp(v.Length());
  
  square(v-mean(v), temp);
  return sumTot(temp)/(v.Length()-1);
}

//---------------------------------------------------------------
// Ritorna il primo elemento di un vettore. Utile per trasformare
// un vettore di dim 1 in un numero.
//---------------------------------------------------------------

double num(const YVector& v)
{
  return *v.data();
}
