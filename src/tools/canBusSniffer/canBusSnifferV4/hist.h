#ifndef HIST_H
#define HIST_H

#define BINS 65535

class hist_performer
{
public:
  signed short hist[BINS];
  long unsigned int number_of_samples;

	hist_performer();
	~hist_performer();
	bool add_sample(signed short sample);
	void do_hist();
};

#endif