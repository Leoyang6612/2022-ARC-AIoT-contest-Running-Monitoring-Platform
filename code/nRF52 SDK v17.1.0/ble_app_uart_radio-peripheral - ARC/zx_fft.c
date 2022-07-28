/*
 * zx_fft.c
 *
 * Implementation of Fast Fourier Transform(FFT)
 * and reversal Fast Fourier Transform(IFFT)
 *
 *  Created on: 2013-8-5
 *      Author: monkeyzx
 */

#include "zx_fft.h"
#include <math.h>
#include <stdlib.h>

/*
 * Bit Reverse
 * === Input ===
 * x : complex numbers
 * n : nodes of FFT. @N should be power of 2, that is 2^(*)
 * l : count by bit of binary format, @l=CEIL{log2(n)}
 * === Output ===
 * r : results after reversed.
 * Note: I use a local variable @temp that result @r can be set
 * to @x and won't overlap.
 */
static void BitReverse(complex *x, complex *r, int n, int l)
{
    int i = 0;
    int j = 0;
    short stk = 0;
    static complex *temp = 0;

    temp = (complex *)malloc(sizeof(complex) * n);
    if (!temp)
    {
        return;
    }

    for (i = 0; i < n; i++)
    {
        stk = 0;
        j = 0;
        do
        {
            stk |= (i >> (j++)) & 0x01;
            if (j < l)
            {
                stk <<= 1;
            }
        } while (j < l);

        if (stk < n)
        { /* 滿足��序輸*/
            temp[stk] = x[i];
        }
    }
    /* copy @temp to @r */
    for (i = 0; i < n; i++)
    {
        r[i] = temp[i];
    }
    free(temp);
}

/*
 * FFT Algorithm
 * === Inputs ===
 * x : complex numbers
 * N : nodes of FFT. @N should be power of 2, that is 2^(*)
 * === Output ===
 * the @x contains the result of FFT algorithm, so the original data
 * in @x is destroyed, please store them before using FFT.
 */
int fft(complex *x, int N)
{
    int i, j, l, ip;
    static int M = 0;
    static int le, le2;
    static FFT_TYPE sR, sI, tR, tI, uR, uI;

    M = (int)(log(N) / log(2));

    /*
     * bit reversal sorting
     */
    BitReverse(x, x, N, M);

    /*
     * For Loops
     */
    for (l = 1; l <= M; l++)
    { /* loop for ceil{log2(N)} */
        le = (int)pow(2, l);
        le2 = (int)(le / 2);
        uR = 1;
        uI = 0;
        sR = cos(PI / le2);
        sI = -sin(PI / le2);
        for (j = 1; j <= le2; j++)
        { /* loop for each sub DFT */
            // jm1 = j - 1;
            for (i = j - 1; i <= N - 1; i += le)
            { /* loop for each butterfly */
                ip = i + le2;
                tR = x[ip].real * uR - x[ip].img * uI;
                tI = x[ip].real * uI + x[ip].img * uR;
                x[ip].real = x[i].real - tR;
                x[ip].img = x[i].img - tI;
                x[i].real += tR;
                x[i].img += tI;
            } /* Next i */
            tR = uR;
            uR = tR * sR - uI * sI;
            uI = tR * sI + uI * sR;
        } /* Next j */
    }     /* Next l */

    return 0;
}

/*
 * Inverse FFT Algorithm
 * === Inputs ===
 * x : complex numbers
 * N : nodes of FFT. @N should be power of 2, that is 2^(*)
 * === Output ===
 * the @x contains the result of FFT algorithm, so the original data
 * in @x is destroyed, please store them before using FFT.
 */
int ifft(complex *x, int N)
{
    int k = 0;

    for (k = 0; k <= N - 1; k++)
    {
        x[k].img = -x[k].img;
    }

    fft(x, N); /* using FFT */

    for (k = 0; k <= N - 1; k++)
    {
        x[k].real = x[k].real / N;
        x[k].img = -x[k].img / N;
    }

    return 0;
}

// static void MakeInput()
// {
//     int i;

//     for (i = 0; i < SAMPLE_NODES; i++)
//     {
//         x[i].real = sin(PI * 2 * i / SAMPLE_NODES);
//         x[i].img = 0.0f;
//         INPUT[i] = sin(PI * 2 * i / SAMPLE_NODES) * 1024;
//     }
// }

// static void MakeOutput()
// {
//     int i;

//     for (i = 0; i < SAMPLE_NODES; i++)
//     {
//         OUTPUT[i] = sqrt(x[i].real * x[i].real + x[i].img * x[i].img) * 1024;
//     }
// }