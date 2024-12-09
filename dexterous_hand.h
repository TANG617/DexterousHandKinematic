#ifndef DEXTEROUS_HAND_H
#define DEXTEROUS_HAND_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    float beta1, beta2, beta3;
    float ax, ay, bx, by, p;
    float l1, l2, l3, l4;
    float cy, h;
    float r1, r2, r3, r4;
    float u1, u2, u3, u4;
    float theta4;
    
    // Internal states
    float q1, q2, theta3;
    float alpha;
    float gamma1, gamma2;
    float theta1;
    float R[3][3];
    float k[2][3];
    float d[3][3];
    float Pp_dip[3];
} DexterousHand;

// Initialize the dexterous hand with parameters
void init_dexterous_hand(DexterousHand* hand, 
                        float ax, float ay, float bx, float by, float p,
                        float l1, float l2, float l3, float l4,
                        float cy, float h,
                        float r1, float r2, float r3, float r4,
                        float u1, float u2, float u3, float u4,
                        float beta1, float beta2, float beta3, float theta4);

// Update kinematics with new joint angles
void update_kinematics(DexterousHand* hand, float q1, float q2, float q3);

#endif
