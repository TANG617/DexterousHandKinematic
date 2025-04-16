#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "dexterous_hand.h"

// #define ORG
int main() {
    clock_t start, end;
    start = clock();

#ifdef ORG
    // Initialize parameters
    float ax = 10, ay = 20, bx = 10, by = 20;
    float p = 65, l1 = 20, l2 = 20, l3 = 30, l4 = 15;
    float cy = 0, h_ = 30, h = 20;
    float r1 = 13, r2 = 26, r3 = 7, r4 = 28.5;
    float u1 = 28, u2 = 6.5, u3 = 7, u4 = 31;
    float beta1 = 45.0/360.0 * 2*M_PI;
    float beta2 = 43.0/360.0 * 2*M_PI;
    float beta3 = 50.0/360.0 * 2*M_PI;
    float theta4 = 68.0/360.0 * 2*M_PI;

    float theta3 = 121.8/360.0 * 2*M_PI;//q3
    // d1,d2,d3 = 45,45,39.17
    // float theta2 = 25.5/360.0 * 2*PI;
#else
    // Updated values from main.py
    double ax = 7, ay = 12;
    double bx = (18.15-2.5)/2, by = 14.14; // bx = bu; by = bv
    double p = 14.48;
    double l1 = 24.19, l2 = 24.19;
    double l3 = 13.75, l4 = 13.07;
    double cy = 0;
    double h = by; // TODO
    double r1 = 10.3, r2 = 26.10, r3 = 5.15, r4 = 30.59;
    double u1 = 26.1, u2 = 6.08, u3 = 6, u4 = 27;
    double beta1 = 52.61 / 180 * M_PI;
    double beta2 = 47.84 / 180 * M_PI;
    double beta3 = 60.95 / 180 * M_PI;
    double theta4 = 56.2 / 180 * M_PI;
#endif

    DexterousHand hand;
    init_dexterous_hand(&hand, ax, ay, bx, by, p, l1, l2, l3, l4, cy, h,
                        r1, r2, r3, r4, u1, u2, u3, u4,
                        beta1, beta2, beta3, theta4);

#ifndef ORG
    // Updated values from main.py test case
    update_kinematics(&hand, 13.61/180.0*M_PI, 0, (180-45.02)/180.0*M_PI);
    printf("1. d1,d2,d3: [%.2f, %.2f, %.2f]\n   ref       [-7.16, -7.16, -5.21]\n",
           hand.d[0][2], hand.d[1][2], hand.d[2][2]);
    
    // Keep the other test cases as they were
    update_kinematics(&hand,52.3/180.0*M_PI,0.0f,(180-35)/180.0*M_PI);
    printf("2. d1,d2,d3: [%.2f, %.2f, %.2f]\n   ref       [-10, -10, -4]\n",
         hand.d[0][2], hand.d[1][2], hand.d[2][2]);
    update_kinematics(&hand,46.18/180.0*M_PI,11.8/180.0*M_PI,(180-35)/180.0*M_PI);
    printf("3. d1,d2,d3: [%.2f, %.2f, %.2f]\n   ref       [-7, -5, -4]\n",
         hand.d[0][2], hand.d[1][2], hand.d[2][2]);

#else
  update_kinematics(&hand,0.0f,0.0f,theta3);
  printf("ORG. d1,d2,d3: [%.2f, %.2f, %.2f]\n",
         hand.d[0][2], hand.d[1][2], hand.d[2][2]);
#endif

    // printf("dip position: [%.2f, %.2f, %.2f]\n",
    //        hand.Pp_dip[0], hand.Pp_dip[1], hand.Pp_dip[2]);

    end = clock();
    double time_taken = ((double)(end - start)) / CLOCKS_PER_SEC;
    printf("Time taken: %f seconds\n", time_taken);

    return 0;
}
