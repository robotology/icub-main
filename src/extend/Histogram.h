#ifndef SEQ_HISTOGRAM_INC
#define SEQ_HISTOGRAM_INC

#include <vector>

#include <math.h>
#include <iostream>

using namespace std;

class Histogram  {
private:
  vector<double> past;
  int N;
  vector<int> dist;
  double bot;
  double top;
  double diff;
  bool dirty;
  int gapWidth;
  int gapCenterIndex;
  double mean;
  double var;

public:
  Histogram() {
    N = 0;
    dirty = true;
  }

  int size() {
    calc();
    return N;
  }

  int getCount(int index) {
    calc();
    if (index>=N||index<0) {
      return 0;
    }
    return dist[index];
  }

  void clear() {
    dirty = true;
    past.clear();
  }

  void add(double v) {
    dirty = true;
    past.push_back(v);
  }

  int getIndex(double v) {
    calc();
    int index = (int)(N*((v-bot)/diff));
    if (index>=N) index = N-1;
    if (index<0) index = 0;
    return index;
  }

  double getValue(int index) {
    double v = (index*diff)/N + bot;
    return v;
  }

  int getGapWidth() {
    calc();
    return gapWidth;
  }
  
  int getGapCenterIndex() {
    calc();
    return gapCenterIndex;
  }

  double getMean() {
    calc();
    return mean;
  }

  double getDeviation() {
    calc();
    return var;
  }

  int getLength() const {
    return past.size();
  }

  double getPast(int index) const {
    return past[index];
  }

  void calc() {
    if (!dirty) {
      return;
    }
    dirty = false;
    N = 100;
    dist = vector<int>(N);
    for (int i=0; i<N; i++) {
      dist[i] = 0;
    }
    if (past.size()==0) {
      top = 0;
      bot = 0;
      diff = 1;
      gapWidth = 0;
      gapCenterIndex = 50;
      mean = 0;
      var = 1;
      return;
    }
    top = past[0];
    bot = past[0];
    int itop = 0;
    int ibot = 0;
    mean = 0;
    double mean2 = 0;
    var = 0;
    for (int i=0; i<past.size(); i++) {
      double v = past[i];
      mean += v;
      mean2 += v*v;
      if (v<bot) {
	bot = v;
	ibot = i;
      }
      if (v>top) {
	top = v;
	itop = i;
      }
    }
    mean /= past.size();
    mean2 /= past.size();
    var = sqrt(fabs(mean2-mean*mean));
    diff = top-bot;
    if (diff<0.00001) {
      diff = 0.00001;
    }
    for (int i=0; i<past.size(); i++) {
      double v = past[i];
      int index = (int)(N*((v-bot)/diff));
      if (index>=N) index = N-1;
      if (index<0) index = 0;
      dist[index]++;
    }
    gapWidth = 0;
    gapCenterIndex = 0;
    int w = 0;
    //for (int i=0; i<N; i++) {
    //cerr << ((dist[i]<10)?dist[i]:9);
    //}
    //cerr << endl;
    for (int i=0; i<N; i++) {
      if (dist[i]>0) {
	if (w>gapWidth) {
	  gapWidth = w;
	  gapCenterIndex = i-w/2;
	  if (gapCenterIndex<0) {
	    gapCenterIndex = 0;
	  }
	}
	w = 0;
      } else {
	w++;
      }
    }
  }
};


#endif
