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

const int expected_values_handler_class::def_expected_values[24][6] =
{
    { 931, -507, -307, 14885, -12784, -343},
    { 16631, 230, 136, -7092, -6761, 411},
    { 993, 498, 306, -12746, 14284, 305},
    { -14751, -192, -157, 8300, 9075, -318},
    { 1878, -11736, -12750, -1082, -1094, -10751},
    { 21743, 34, -1466, -10741, -11008, 1930},
    { 2412, 11706, 10117, -1084, -1100, 13498 },
    { 165, -13228, -10733, -1710, 1244, -11357 },
    { 317, 104, 2290, -19036, 18501, 2058 },
    { -197, 10213, 12250, -1024, 1475, 12757 },
    { -1667, -11801, -10300, 529, 861, -13321 },
    { -21130, -548, 715, 10458, 10580, -1909 },
    { -1823, 11652, 12624, 1238, 850, 10776 },
    { -126, -10361, -12255, 1081, -1241, -12982 },
    { -248, 1125, -770, 19444, -18813, -1028 },
    { -40, 13101, 10689, 1579, -1282, 11221 },
    { -4547, -67, 127, -4147, -3907, -58 },
    { 3606, 9, -113, 5524, 3660, -4 },
    { 5998, -175, -6057, -3030, -2944, 5825 },
    { -6024, -671, 5128, 3146, 3065, -6575 },
    { -3555, 5211, -781, 6951, -3239, -6910 },
    { 3410, -5896, 75, -6527, 3191, 6047 },
    { -3013, -6115, 5825, -3059, 6219, -53 },
    { 3143, 5366, -6349, 3156, -6222, -691 }
};

const int expected_values_handler_class::def_expected_values_thresholds[2] =
{
    2000,
    4000
};

bool expected_values_handler_class::init()
{
    for(int iy=0; iy<24; iy++)
    {
        for(int ix=0; ix<6; ix++)
        {
            expected_values[ix][iy] = def_expected_values[iy][ix];
        }
    }

    threshold1 = def_expected_values_thresholds[0];
    threshold2 = def_expected_values_thresholds[1];

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
