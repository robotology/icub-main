#include "expected_values.h"
#include <fstream>
#include <string>

using namespace std;

signed_elem_class::signed_elem_class(const signed_elem_class& val)
{
	for (int i=0; i<6;i++)
		{
			this->dat[i]=val.dat[i];
		}
}


signed_elem_class& signed_elem_class::operator=(const signed_elem_class &rhs)
{
    if (this != &rhs)
	{
		for (int i=0; i<6;i++)
		{
			this->dat[i]=rhs.dat[i];
		}
    }
    return *this;
 }

unsigned_elem_class::unsigned_elem_class(const unsigned_elem_class& val)
{
	for (int i=0; i<6;i++)
		{
			this->dat[i]=val.dat[i];
		}
}

unsigned_elem_class& unsigned_elem_class::operator=(const unsigned_elem_class &rhs)
{
    if (this != &rhs)
	{
		for (int i=0; i<6;i++)
		{
			this->dat[i]=rhs.dat[i];
		}
    }
    return *this;
 }

bool expected_values_handler_class::init  (const char* filename)
{
	fstream filestr;
	filestr.open (filename, fstream::in);
    if (!filestr.is_open())
        {
            printf ("ERR: Error opening calibration file!\n");
            return false;
        }
    string dummy;
    getline(filestr,dummy);
	for (int iy=0; iy<24; iy++)
		for (int ix=0; ix<6; ix++)
		{
			filestr >> expected_values[ix][iy] ;
		}
    filestr >> threshold1;
    filestr >> threshold2;
	filestr.close();
	return true;
}

expected_values_handler_class::expected_values_handler_class()
{
}

bool expected_values_handler_class::check_vals(signed_elem_class values, int trial, signed_elem_class& diff, signed_elem_class& in_bound)
{
	if (trial<0 || trial>=25) trial=0;

	int count=0;
	for (int i=0; i<6; i++)
	{
		in_bound.dat[i]=0;
		diff.dat[i]=(values.dat[i]-expected_values[i][trial]);
		if (abs(diff.dat[i])< threshold1) 
			{	
				in_bound.dat[i]=1; //green condition
				count++;
			}
        else if (abs(diff.dat[i])> threshold1 && abs(diff.dat[i])< threshold2) 
			{	
				in_bound.dat[i]=2; //yellow condition
				count++;
			}
        else
            {
                in_bound.dat[i]=0; //red condition, no counter increment
            }
	}

	if (count==6) return true;
	else return false;
}

void expected_values_handler_class::get_current_expected_values (signed_elem_class& values, int trial)
{
	int i=0;
	if (trial<0 || trial>=25) trial=0;
	for (i=0; i<6; i++)
	{
		values.dat[i]=expected_values[i][trial];
	}
}
