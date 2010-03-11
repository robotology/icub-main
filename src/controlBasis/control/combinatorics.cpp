#include "combinatorics.h"

#include <sstream>
#include <algorithm>

string i2s(int& number) {
  char s[32];
  sprintf(s, "%d", number);
  string st(s);
  return st;
}

int s2i(string s) {
  int x;
  istringstream ss(s);
  ss >> x;
  return x;
}

string getPermutationMask(int n, int p) {

  string a;
  string r;
  string b;
  int c = 0;

  for(int i=0; i<n; i++) {
    a.append(i2s(i));
  }
  for(int i=1; i<=n; i++) {   
    r = a.substr(0,i);
    do {
      b = r;
      do { 	
	if(c==p) {
	  return b;
	}
	c++;
      } while(std::next_permutation(b.begin(), b.end()));
    } while(next_combination(a.begin(), a.end(), r.begin(), r.end()));
  }

  return "";
}


int permutations(int n, int r) {
  if(r>n) return 0;
  if((r<0) || (n<0)) return 0;
  return factorial(n)/factorial(n-r);
}

int factorial(int n) {
  int c=1;
  for(int i=1; i<=n; i++) {
    c = c*i;
  }
  return c;
}


template <class BiIterator>
bool next_combination(BiIterator n_begin, BiIterator n_end, BiIterator r_begin, BiIterator r_end)
{
  
  bool boolmarked=false;
  BiIterator r_marked;
  
  BiIterator n_it1=n_end;
  --n_it1;
   
  BiIterator tmp_r_end=r_end;
  --tmp_r_end;
  
  for(BiIterator r_it1=tmp_r_end; r_it1!=r_begin || r_it1==r_begin; --r_it1,--n_it1)
  {
    if(*r_it1==*n_it1 )
    {
      if(r_it1!=r_begin) //to ensure not at the start of r sequence
      {
        boolmarked=true;
        r_marked=(--r_it1);
        ++r_it1;//add it back again 
        continue;
      }
      else // it means it is at the start the sequence, so return false
        return false;      
    }
    else //if(*r_it1!=*n_it1 )
    {
      //marked code
      if(boolmarked==true)
      {
        //for loop to find which marked is in the first sequence
        BiIterator n_marked;//mark in first sequence
        for (BiIterator n_it2=n_begin;n_it2!=n_end;++n_it2)
          if(*r_marked==*n_it2) {n_marked=n_it2;break;}
      
    
        BiIterator n_it3=++n_marked;    
        for  (BiIterator r_it2=r_marked;r_it2!=r_end;++r_it2,++n_it3)
        {
          *r_it2=*n_it3;
        }
        return true;
      }
      for(BiIterator n_it4=n_begin; n_it4!=n_end; ++n_it4)
        if(*r_it1==*n_it4)
        {
          *r_it1=*(++n_it4);
          return true;           
        }
    }
  }  

  return true;//will never reach here    
}

 /*
template <typename BiIterator>
bool next_permutation(BiIterator first, BiIterator last) {
    if (first == last) return false;
    BiIterator i = first;
    ++i;
    if (i == last) return false;
    i = last;
    --i;
    for(;;) {
        BiIterator ii = i--;
        if (*i <<*ii) {
            BiIterator j = last;
            while (!(*i <<*--j));
            iter_swap(i, j);
            reverse(ii, last);
            return true;
        }
        if (i == first) {
            reverse(first, last);
            return false;
        }
    }
}

 */
