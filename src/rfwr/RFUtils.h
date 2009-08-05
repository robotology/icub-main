#include <YARPMath.h>
#include <cassert>
#include <functional>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <fstream>

YMatrix identity(int dim);
YMatrix ones(int dim);
YMatrix ones(int dim1, int dim2);
YMatrix diag(const YVector &val);
void diag(const YVector &val, YMatrix &res);
YVector diag(const YMatrix &m);
void diag(const YMatrix &val, YVector &rest);
YVector conversion(const YMatrix &m);

// YVector times(YVector v1, YVector v2);
void times(const YVector &v1, const YVector &v2, YVector &res);
void times(const YMatrix &m1, const YMatrix &m2, YMatrix &res);
void ratio(const YVector &v1, const YVector &v2, YVector &res);
void ratio(const YMatrix &m1, const YMatrix &m2, YMatrix &res);

void square(const YVector &v1, YVector &res);
void square(const YMatrix &m, YMatrix &res);

YVector square(const YVector &v1);
YMatrix square(const YMatrix &m);
YMatrix times(const YMatrix &m1, const YMatrix &m2);

// YVector ratio(YVector v1, YVector v2);
void sortGr(YVector & v);

YVector Row(const YMatrix &m, int num);
YVector Col(const YMatrix &m, int num);
YMatrix conversionR(const YVector &v);
YMatrix conversionC(const YVector &v);
double maximum (const YVector &v);
YVector max_col(const YMatrix &m);
YVector onesV(int dim);
double sumTot(const YMatrix &m);
//YMatrix exp (const YMatrix &m);
void exp(const YMatrix &m, YMatrix &res);
YMatrix abs (const YMatrix &m);
YMatrix log (const YMatrix &m);
double maximum(const YMatrix &m);
double minimum (const YVector &v);
double minimum (const YMatrix &m);

YMatrix PartPos(const YMatrix &m);
void choldc(YMatrix &a, int n);
void ReplSup(YMatrix &m, double val);
YMatrix segno(const YMatrix &m);
YMatrix PartNeg(const YMatrix &m);
double prod(const YMatrix &m);
double prod(const YVector &v);
int findMag(const YVector &v, double val, int inds[]);
void outIni(const YVector &v, std::ostream & file);
void outIni(const YMatrix &m, std::ostream & file);
double sumTot(const YVector& v);
double mean(const YVector& v);
double var(const YVector& v);
double num(const YVector& v);

