// fir.c
// David_Harris@hmc.edu 20 January 2022
// Finite Impulse Response Filter

#include <stdio.h>  // supports printf
#include <math.h>   // supports fabs
#include "util.h"   // supports verify
extern void fir(int x[], int c[], int y[], int n, int m);


// Add two Q1.31 fixed point numbers
int add_q31(int a, int b) {
    return (a + b);
}

// Multiply two Q1.31 fixed point numbers
int mul_q31(int a, int b) {
    int64_t res = (int64_t) a * (int64_t) b; // to ensure an overflow and widening doesn't happen insead
    int32_t result = (res << 1) >> 32;
    // printf("mul_q31: a = %x, b = %x, res = %lx, result = %x\n", a, b, res, result);
    return result;
}


// low pass filter x with coefficients c, result in y
// n is the length of x, m is the length of c
// inputs in Q1.31 format
// void fir(int x[], int c[], int y[], int n, int m) {
//     int i, j;
//     int32_t sum;

//     for (i=0; i<=(n-m); i++) {
//         sum = 0;
//         for (j=0; j<m; j++) {
//             sum = add_q31(sum, mul_q31(c[j],x[i-j+(m-1)]));
//         }
//         y[i] = sum;
//     }
// }

int main(void) {
    int32_t sin_table[20] = { // in Q1.31 format
        0x00000000, // sin(0*2pi/10)
        0x4B3C8C12, // sin(1*2pi/10)
        0x79BC384D, // sin(2*2pi/10)
        0x79BC384D, // sin(3*2pi/10)
        0x4B3C8C12, // sin(4*2pi/10)
        0x00000000, // sin(5*2pi/10)
        0xB4C373EE, // sin(6*2pi/10)
        0x8643C7B3, // sin(7*2pi/10)
        0x8643C7B3, // sin(8*2pi/10)
        0xB4C373EE, // sin(9*2pi/10)
        0x00000000, // sin(10*2pi/10)
        0x4B3C8C12, // sin(11*2pi/10)
        0x79BC384D, // sin(12*2pi/10)
        0x79BC384D, // sin(13*2pi/10)
        0x4B3C8C12, // sin(14*2pi/10)
        0x00000000, // sin(15*2pi/10)
        0xB4C373EE, // sin(16*2pi/10)
        0x8643C7B3, // sin(17*2pi/10)
        0x8643C7B3, // sin(18*2pi/10)
        0xB4C373EE  // sin(19*2pi/10)
    };
    int lowpass[4] = {0x20000001, 0x20000002, 0x20000003, 0x20000004}; // 1/4 in Q1.31 format
    int y[17];
    int expected[17] = { // in Q1.31 format
        0x4fad3f2f,
        0x627c6236,
        0x4fad3f32,
        0x1e6f0e17,
        0xe190f1eb,
        0xb052c0ce,
        0x9d839dc6,
        0xb052c0cb,
        0xe190f1e6,
        0x1e6f0e12,
        0x4fad3f2f,
        0x627c6236,
        0x4fad3f32,
        0x1e6f0e17,
        0xe190f1eb,
        0xb052c0ce,
        0x9d839dc6
    };
    setStats(1);        // record initial mcycle and minstret
    fir(sin_table, lowpass, y, 20, 4);
    setStats(0);        // record elapsed mcycle and minstret
    for (int i=0; i<17; i++) {
        printf("y[%d] = %x\n", i, y[i]);
    }
    return verify(16, y, expected);
// check the 1 element of s matches expected. 0 means success
}














// void fir(int N, int M, double X[], double c[], double Y[]) {
//   int i, n;
//   double sum;

//   for (n=0; n<N; n++) {
//       sum = 0;
//       for (i=0; i<M; i++) {
//           sum += c[i]*X[n-i+(M-1)];
//       }
//       Y[n] = sum;
//   }
// }

// int main(void) {
//     double X[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
//     double c[5] = {0.2, 0.2, 0.2, 0.2, 0.2};
//     double Y[15];
//     double Yexpected[15] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

//     setStats(1);
//     fir(15, 5, X, c, Y);
//     setStats(0);
//     // library linked doesn't support printing doubles, so convert to integers to print
//     for (int i=0; i<15; i++)  {
//         int tmp = Y[i];
//         printf("Y[%d] = %d\n", i, tmp);
//     }
//     // verifyDouble doesn't work exactly because of rounding, so check for almost equal
//     for (int i=0; i<15; i++) {
//         if (fabs(Y[i] - Yexpected[i]) > 1e-10) {
//             return 1;
//         }
//     }
//     return 0;
// }
