
// Copyright: 2003 Lorenzo Natale
// Author: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0. (2010)
// CopyPolicy: Academic Free License Version 10 (2003)

#ifndef __YARP3DHISTOHH__
#define __YARP3DHISTOHH__

#include <yarp/sig/Image.h>

#include <iostream>
using namespace std;

// use HistoKey for historical reasons
typedef yarp::sig::PixelRgb HistoKey;

class HistoEntry
{
public:
	HistoEntry()
	{ _acc = 0.0; }
	
/*	HistoEntry &operator ++ (int)
	{ 
		_acc++;
		return *this;
	}*/

	void accumulate(double w)
	{ 
		_acc += w; 
	}

	double value()
	{ return _acc; }

	double setValue(double nv)
	{
		_acc = nv;
		return _acc;
	}

	void reset()
	{ _acc = 0.0; }

private:
	double _acc;
};

class Histo3D
{
public:
	Histo3D();
	~Histo3D();
	
	// resize methods
	void resize(unsigned char max, unsigned char min, unsigned char size);
	void resize(unsigned char max, unsigned char min, unsigned char *size);
	void resize(unsigned char *max, unsigned char *min, unsigned char *size);
	
	// clean histo
	void clean();

	// query histo
	inline int find(unsigned int it, HistoEntry **v)
	{
		if (it < _nElem)
		{
			*v = &_lut[it];
			return 0;
		}
		else
		{
			return -1;
		}
	}

	// query histo
	inline int find(unsigned int it, HistoEntry &v)
	{
		// LATER: CHECK INDEX 
		if (it < _nElem)
		{
			v = _lut[it];
			return 0;
		}
		else
			return -1;
	}

	double _maximum;

	unsigned char _max[3];
	unsigned char _min[3];
	float _delta[3];
	unsigned char _size[3];

	// return first element
	unsigned int begin()
	{ return 0; }

	inline unsigned int pixelToKey(unsigned char r, unsigned char g, unsigned char b, unsigned int *key)
	{
		int tmpR = (int) (r/_delta[0]);
		int tmpG = (int) (g/_delta[1]);
		int tmpB = (int) (b/_delta[2]);

		unsigned int tmp;
		tmp = (tmpR)*_size[1]*_size[2];
		tmp += (tmpG)*_size[2];
		tmp += (tmpB);

		*key = tmp;

		return tmp;
	}

private:
	HistoEntry	*_lut;
	unsigned int _nElem;

};

class Histo1D
{
public:
	Histo1D();
	
	void resize(unsigned char max, unsigned char min, unsigned char size);
	void clean();
	
	// find methods
	inline int find(unsigned int it, HistoEntry **v)
	{
		if (it < _nElem)
		{
			*v = &_lut[it];
			return 0;
		}
		else
			return -1;
	}
	
	inline int find(unsigned int it, HistoEntry &v)
	{
		if (it < _nElem)
		{
			v = _lut[it];
			return 0;
		}
		else
			return -1;
	}

	unsigned int begin()
	{ return 0; }

	double _maximum;

	unsigned char _max;
	unsigned char _min;
	unsigned char _delta;
	unsigned char _size;

	inline unsigned int pixelToKey(unsigned char v, unsigned int *key)
	{
		unsigned int tmp = (v/_delta);
		*key = tmp;
		return tmp;
	}

	HistoKey _key;
private:
	HistoEntry *_lut;
	unsigned int _nElem;
};

class Histogram
{
public:
	Histogram();

	Histogram(unsigned char max, unsigned char min, unsigned char n);
	Histogram(unsigned char max, unsigned char min, unsigned char *n);
	
	~Histogram(){};

	void Resize(unsigned char max, unsigned char min, unsigned char n);
	void Resize(unsigned char max, unsigned char min, unsigned char *n);
	void Apply(unsigned char r, unsigned char g, unsigned char b, double w = 1.0);

	inline int find(unsigned int it, HistoEntry &v)
	{
		return _3dlut.find(it, v);
	}

	inline double maximum()
	{ return _3dlut._maximum; }
		
	int dump(const char *basename);
	
    inline double backProjection(const yarp::sig::PixelRgb &p)
	{ return _backProjection(p.r, p.g, p.b); }

	inline double backProjection(const yarp::sig::PixelBgr &p)
	{ return _backProjection(p.r, p.g, p.b); }

	inline double backProjection(const yarp::sig::PixelHsv &p)
	{ return _backProjection(p.h, p.s, p.v); }

	inline double backProjection(unsigned char r, unsigned char g, unsigned char b)
	{ return _backProjection(r,g,b); }

	int load(const char *basename);
	
	void clean();
	
private:
	int _dumpFull(const char *file);
	int _dump1D(const char *file, Histo1D &lut);
	int _load1D(const char *file, Histo1D &lut);
	int _loadFull(const char *file);

	inline double _backProjection(unsigned char r, unsigned char g, unsigned char b)
	{
		HistoEntry *tmpEntryP = NULL;
		unsigned int it;
		
		_3dlut.pixelToKey(r, g, b, &it);
		if (_3dlut.find(it, &tmpEntryP) != -1)
		{
			double max = _3dlut._maximum;
			double val = tmpEntryP->value();
			return val/max; // tmpEntryP->value()/_3dlut._maximum;
		}
		else
		{
			cout << "Warning, index out of limit this should not happen. Check bin size.\n";
			return 0;
		}
		return 0;
	}

private:
	Histo3D _3dlut;
	Histo1D _rlut;
	Histo1D _glut;
	Histo1D _blut;
};

#endif
