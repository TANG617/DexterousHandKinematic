#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "dexterous_hand.h"

#define PI 3.14159265358979323846

int main() {
    clock_t start, end;
    start = clock();
    
    // Initialize parameters
    float ax = 10, ay = 20, bx = 10, by = 20;
    float p = 65, l1 = 20, l2 = 20, l3 = 30, l4 = 15;
    float cy = 0, h = 30;
    float r1 = 13, r2 = 26, r3 = 7, r4 = 28.5;
    float u1 = 28, u2 = 6.5, u3 = 7, u4 = 31;
    float beta1 = 45.0/360.0 * 2*PI;
    float beta2 = 43.0/360.0 * 2*PI;
    float beta3 = 50.0/360.0 * 2*PI;
    float theta2 = 25.5/360.0 * 2*PI;
    float theta4 = 68.0/360.0 * 2*PI;

    DexterousHand hand;
    init_dexterous_hand(&hand, ax, ay, bx, by, p, l1, l2, l3, l4, cy, h,
                        r1, r2, r3, r4, u1, u2, u3, u4,
                        beta1, beta2, beta3, theta2, theta4);


    update_kinematics(&hand,0.0f,0.0f,0.0f);

    // Plot points (you'll need to implement this based on your plotting library)
    // Print d1, d2, d3 values from last calculation
    printf("\nFinal d values:\n");
    printf("d1: [%.2f, %.2f, %.2f]\n", 
           hand.d[0][0], hand.d[0][1], hand.d[0][2]);
    printf("d2: [%.2f, %.2f, %.2f]\n", 
           hand.d[1][0], hand.d[1][1], hand.d[1][2]); 
    printf("d3: [%.2f, %.2f, %.2f]\n",
           hand.d[2][0], hand.d[2][1], hand.d[2][2]);
    // Free memory
    // free(x_points);
    // free(y_points);
    // free(z_points);

    printf("dip position: [%.2f, %.2f, %.2f]\n", 
           hand.Pp_dip[0], hand.Pp_dip[1], hand.Pp_dip[2]);

    end = clock();
    double time_taken = ((double)(end - start)) / CLOCKS_PER_SEC;
    printf("Time taken: %f seconds\n", time_taken);

    return 0;
}
