
#ifndef FFT_INC
#define FFT_INC

class FFT {
private:
  void *system_resources;

public:
  FFT(int n);

  void apply(float *input, float *mag, float *phase);

  virtual ~FFT();
};


#endif
