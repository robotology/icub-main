// basics.h : header file for basic stuff
//

#ifndef __basicsh__
#define __basicsh__

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <limits>
#include <stdlib.h>

using namespace std;

// -------------------------------------------------------
// little macros for easier (?) handling of for over arrays
#define foreach(limit,index) unsigned int index; for ( index=0; index<((unsigned int)(limit)); ++index )
#define foreach_s(start,limit,index) unsigned int index; for ( index=(start); index<(limit); ++index )

// allocate memory
template <class T> void lmAlloc ( T*& base, unsigned int num_elem )
{
	base = new T[num_elem];
	if ( base == 0 ) {
		cout << "FATAL ERROR: no memory." << endl;
		exit(-1);
	}
}

// -------------------------------------------------------
// real numbers
// -------------------------------------------------------

typedef double real;

#endif
