#define _USE_MATH_DEFINES
#include "dexterous_hand.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void create_rotation_matrix(DexterousHand* hand) {
    hand->R[0][0] = cos(hand->q2);
    hand->R[0][1] = sin(hand->q2) * sin(hand->q1);
    hand->R[0][2] = sin(hand->q2) * cos(hand->q1);
    
    hand->R[1][0] = 0;
    hand->R[1][1] = cos(hand->q1);
    hand->R[1][2] = -sin(hand->q1);
    
    hand->R[2][0] = -sin(hand->q2);
    hand->R[2][1] = cos(hand->q2) * sin(hand->q1);
    hand->R[2][2] = cos(hand->q2) * cos(hand->q1);
}

void create_argument(DexterousHand* hand) {
    float a[2][3] = {{hand->ax, hand->ay, 0}, {-hand->ax, hand->ay, 0}};
    float b[2][3] = {{hand->bx, hand->by, 0}, {-hand->bx, hand->by, 0}};
    float p[3] = {0, 0, hand->p};
    
    // Calculate k = p + R @ b - a
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 3; j++) {
            hand->k[i][j] = p[j];
            for(int k = 0; k < 3; k++) {
                hand->k[i][j] += hand->R[j][k] * b[i][k];
            }
            hand->k[i][j] -= a[i][j];
        }
    }
}

void calculate_d12(DexterousHand* hand) {
    hand->alpha = M_PI - hand->beta1 - hand->theta1; 
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            hand->d[i][j] = 0;
        }
    }
    
    for(int i = 0; i < 2; i++) {
        float k_norm_squared = 0;
        for(int j = 0; j < 3; j++) {
            k_norm_squared += hand->k[i][j] * hand->k[i][j];
        }
        k_norm_squared = sqrt(k_norm_squared);
        
        hand->d[i][2] = hand->k[i][2] -
            sqrt(hand->l1 * hand->l1 + hand->k[i][2] * hand->k[i][2] - k_norm_squared * k_norm_squared);
    }
}

void calculate_d3(DexterousHand* hand) {
    float P12_x = hand->h * sin(hand->q2) * sin(hand->q1) - 
                  hand->l4 * sin(hand->q2) * sin(hand->q1 - hand->alpha);
    float P12_y = hand->h * cos(hand->q1) - 
                  hand->l4 * cos(hand->q1 - hand->alpha) - hand->cy;
    float P12_z = hand->h * cos(hand->q2) * sin(hand->q1) - 
                  hand->l4 * cos(hand->q2) * sin(hand->q1 - hand->alpha) + hand->p;
    float A1 = P12_z;
    float B1 = P12_x * P12_x + P12_y * P12_y + P12_z * P12_z - hand->l3 * hand->l3;
    

    float discriminant = A1 * A1 - B1;
    if (discriminant >= 0) {
        hand->d[2][2] = A1 - sqrt(discriminant);
    } else {
        hand->d[2][2] = A1 - sqrt(-discriminant);
    }
}

void calculate_theta1(DexterousHand* hand) {
    float A2 = 2 * hand->r1 * (hand->r3 * sin(hand->theta3) - hand->r4 * sin(hand->theta4));
    float B2 = 2 * hand->r1 * (hand->r3 * cos(hand->theta3) - hand->r4 * cos(hand->theta4));
    float C2 = hand->r2 * hand->r2 - hand->r1 * hand->r1 - hand->r3 * hand->r3 - 
               hand->r4 * hand->r4 + 2 * hand->r3 * hand->r4 * cos(hand->theta3 - hand->theta4);
    
    hand->theta1 = asin(C2 / sqrt(A2 * A2 + B2 * B2)) - atan2(B2, A2);
}

void calculate_gamma2(DexterousHand* hand) {
    hand->gamma1 = hand->theta3 - hand->beta2 + hand->beta3;//omit -1/2MPI
    
    float A3 = 2 * hand->u1 * hand->u2 * sin(hand->gamma1);
    float B3 = 2 * hand->u1 * hand->u2 * cos(hand->gamma1) + 2 * hand->u2 * hand->u3;
    float C3 = hand->u4 * hand->u4 - hand->u1 * hand->u1 - hand->u2 * hand->u2 - 
               hand->u3 * hand->u3 - 2 * hand->u1 * hand->u3 * cos(hand->gamma1);
    
    hand->gamma2 = asin(C3 / sqrt(A3 * A3 + B3 * B3)) - atan2(B3, A3);
}

// void calculate_dip_position(DexterousHand* hand) {
//     // Calculate DY and DZ
//     float DY = hand->u1 * cos(hand->gamma1);
//     float DZ = hand->u1 * sin(hand->gamma1);
//     float Ppip_dipSS[3] = {0, DY, DZ};
//
//     // Calculate delta and rotation matrix r
//     float delta = - hand->theta3 + hand->gamma1 - hand->beta3 + M_PI;
//     float r[3][3] = {
//         {1, 0, 0},
//         {0, cos(delta), sin(delta)},  // 修正符号
//         {0, -sin(delta), cos(delta)}    // 修正符号
//     };
//
//     // Calculate Ppip_dipS = r @ Ppip_dipSS
//     float Ppip_dipS[3] = {0};
//     for(int i = 0; i < 3; i++) {
//         for(int j = 0; j < 3; j++) {
//             Ppip_dipS[i] += r[i][j] * Ppip_dipSS[j];
//         }
//     }
//
//
//
//     // Calculate CV and CW
//     float CV = hand->r1 * cos(hand->theta1) +
//                hand->r2 * cos(hand->theta2) +
//                hand->r3 * cos(hand->theta3);
//     float CW = hand->r1 * sin(hand->theta1) +
//                hand->r2 * sin(hand->theta2) +
//                hand->r3 * sin(hand->theta3);
//
//
//     printf("theta1:%f\n",hand->theta1);
//     printf("theta2:%f\n",hand->theta2);
//     printf("theta3:%f\n",hand->theta3);
//
//     // Calculate Pp_pipS
//     float Pp_pipS[3] = {0, CV + hand->h, CW};
//
//     printf("Pp_pipS:%f,%f,%f\n",Pp_pipS[0],Pp_pipS[1],Pp_pipS[2]);
//     // Calculate Pp_dipS = Pp_pipS + Ppip_dipS
//     float Pp_dipS[3];
//     for(int i = 0; i < 3; i++) {
//         Pp_dipS[i] = Pp_pipS[i] + Ppip_dipS[i];
//     }
//
//     printf("Pp_dipS:%f,%f,%f\n",Pp_dipS[0],Pp_dipS[1],Pp_dipS[2]);
//     // Calculate final position Pp_dip = R @ Pp_dipS + [0,0,p]
//     for(int i = 0; i < 3; i++) {
//         hand->Pp_dip[i] = 0;
//         for(int j = 0; j < 3; j++) {
//             hand->Pp_dip[i] += hand->R[i][j] * Pp_dipS[j];
//         }
//     }
//     hand->Pp_dip[2] += hand->p;
// }


void init_dexterous_hand(DexterousHand* hand, 
                        float ax, float ay, float bx, float by, float p,
                        float l1, float l2, float l3, float l4,
                        float cy, float h,
                        float r1, float r2, float r3, float r4,
                        float u1, float u2, float u3, float u4,
                        float beta1, float beta2, float beta3,  float theta4) {
    hand->ax = ax; hand->ay = ay;
    hand->bx = bx; hand->by = by;
    hand->p = p;
    hand->l1 = l1; hand->l2 = l2;
    hand->l3 = l3; hand->l4 = l4;
    hand->cy = cy; hand->h = h;
    hand->r1 = r1; hand->r2 = r2;
    hand->r3 = r3; hand->r4 = r4;
    hand->u1 = u1; hand->u2 = u2;
    hand->u3 = u3; hand->u4 = u4;
    hand->beta1 = beta1;
    hand->beta2 = beta2;
    hand->beta3 = beta3;
    hand->theta4 = theta4;
}

void update_kinematics(DexterousHand* hand, float q1, float q2, float q3) {
    hand->q1 = q1;
    hand->q2 = q2;
    hand->theta3 = q3;
    
    create_rotation_matrix(hand);
    create_argument(hand);
    calculate_gamma2(hand);
    calculate_theta1(hand);
    calculate_d12(hand);
    calculate_d3(hand);
    // calculate_dip_position(hand);
}
