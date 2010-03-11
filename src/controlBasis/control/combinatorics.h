#ifndef _COMBINATORICS__H_
#define _COMBINATORICS__H_

#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

using namespace std;

int permutations(int n, int r);
int factorial(int n);
string i2s(int& number);
int s2i(string s);
string getPermutationMask(int n, int p);

template <typename BiIterator>
bool next_combination(BiIterator n_begin, BiIterator n_end, BiIterator r_begin, BiIterator r_end);

#endif
