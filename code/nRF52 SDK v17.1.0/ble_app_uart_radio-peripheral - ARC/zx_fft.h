#ifndef ZX_FFT_H_
#define ZX_FFT_H_

typedef float FFT_TYPE;

#ifndef PI
#define PI (3.14159265f)
#endif

typedef struct complex_st
{
    FFT_TYPE real;
    FFT_TYPE img;
} complex;

int fft(complex *x, int N);
int ifft(complex *x, int N);
void zx_fft(void);

#define SAMPLE_NODES 8

#endif /* ZX_FFT_H_ */