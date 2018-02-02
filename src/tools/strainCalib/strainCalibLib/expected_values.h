#ifndef EXPECTED_VALUES_H
#define EXPECTED_VALUES_H

struct signed_elem_class
{
	signed   short dat[6];
	signed_elem_class (const signed_elem_class& val);
	signed_elem_class& operator=(const signed_elem_class &rhs);
	signed_elem_class () {for (int i=0; i<6; i++) {dat[i]=0;}}
	void remove_bias (signed_elem_class bias) {for (int i=0; i<6; i++) {dat[i]-=bias.dat[i];}}
};

struct unsigned_elem_class
{
	unsigned_elem_class (const unsigned_elem_class& val);
	unsigned_elem_class& operator=(const unsigned_elem_class &rhs);
	unsigned   short dat[6];
	unsigned_elem_class () {for (int i=0; i<6; i++) {dat[i]=0;}}
};

class expected_values_handler_class
{
	private:
		int expected_values [6][24];
		int threshold1;
		int threshold2;

	public:
		expected_values_handler_class();
        bool init (const char* filename);
	    bool check_vals(signed_elem_class values, int trial, signed_elem_class& diff, signed_elem_class& in_bound);
		void get_current_expected_values (signed_elem_class& values, int trial);
};

#endif
