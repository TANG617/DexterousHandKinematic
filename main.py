import torch
import math
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# 赋值
ax, ay = 7, 12
bx, by = (18.15-2.5)/2, 14.14 # bx = bu; by = bv
p = 14.48
l1, l2 = 24.19, 24.19
l3, l4 = 10.2, 13.07
cy = 0
h = by #TODO
r1, r2, r3, r4 = 10.3, 26.10, 5.15, 30.59
u1, u2, u3, u4 = 26.1, 6.08, 6, 27
beta1 = 52.61 / 180 * math.pi
beta2 = 47.84 / 180 * math.pi
beta3 = 60.95 / 180 * math.pi

theta4 = 56.2 / 180 * math.pi

# ax=10
# ay=20
# bx=10
# by=20
# p=65
# q1=0
# q2=0
# l1=20
# l2=20
# l3=30
# l4=15
# cy = 0
# h = 30
# alpha = 0
#
# r1 = 13
# r2 = 26
# r3 = 7
# r4 = 28.5
# #
# theta1 = 135/360*math.pi*2
# theta2 = 25.5/360*math.pi*2
# theta3 = 121.8/360*math.pi*2
# theta4 = 68/360*math.pi*2
#
# u1 = 28
# u2 = 6.5
# u3 = 7
# u4 = 31
#
# gamma1 = 42/360*math.pi*2
# gamma2 = 275/360*math.pi*2
# gamma3 = 180/360*math.pi*2
# gamma4 = 23/360*math.pi*2
#
# beta1 = 45/360*math.pi*2
# beta2 = 43/360*math.pi*2
# beta3 = 50/360*math.pi*2

class DexterousHandKinematics:
    def __init__(self, ax, ay, bx, by, p, l1, l2, l3, l4, cy, h, r1, r2, r3, r4, u1, u2, u3, u4, beta1, beta2, beta3, theta4):
        self.beta1 = beta1
        self.beta2 = beta2
        self.beta3 = beta3
        self.ax = ax
        self.ay = ay
        self.bx = bx
        self.by = by
        self.p = p
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.cy = cy
        self.h = h
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
        self.r4 = r4
        self.u1 = u1
        self.u2 = u2
        self.u3 = u3
        self.u4 = u4
        self.theta4 = theta4

    def update_kinematics(self,q1,q2,q3):

        self.q1 = q1
        self.q2 = q2
        self.theta3 = q3
        
        self.create_rotation_matrix()
        self.create_argument()
        
        # self.calculate_gamma2()
        self.calculate_theta1() 
        
        self.calculate_d12()
        self.calculate_d3()
        # self.calculate_dip_position()

    def create_rotation_matrix(self):
        self.R = torch.tensor([
            [math.cos(self.q2),      math.sin(self.q2)*math.sin(self.q1),    math.sin(self.q2)*math.cos(self.q1)],
            [0,                 math.cos(self.q1),                  -math.sin(self.q1)],
            [-math.sin(self.q2),     math.cos(self.q2)*math.sin(self.q1),    math.cos(self.q2)*math.cos(self.q1)]
        ], dtype=torch.float32)
        print("R:", self.R)

    def create_argument(self):

        a = torch.tensor([[self.ax,self.ay,0],[-self.ax,self.ay,0]],dtype=torch.float32)
        b_ = torch.tensor([[self.bx,self.by,0],[-self.bx,self.by,0]],dtype=torch.float32)
        p = torch.tensor([[0,0,self.p]],dtype=torch.float32)
        self.k = torch.cat([p + self.R @ b_[0] - a[0], p + self.R @ b_[1] - a[1]], dim=0)

    def calculate_theta1(self):
        A2 = 2*self.r1*(self.r3*math.sin(self.theta3) - self.r4 * math.sin(self.theta4))
        B2 = 2*self.r1*(self.r3*math.cos(self.theta3) - self.r4 * math.cos(self.theta4))
        C2 = self.r2**2 - self.r1**2-self.r3**2 -self.r4**2 + 2*self.r3*self.r4*math.cos(self.theta3-self.theta4)
        # self.theta1 = math.pi /4 - (math.asin(C2/math.sqrt(A2**2 + B2**2)) - math.atan(B2/A2))
        self.theta1 = math.asin(C2 / math.sqrt(A2 ** 2 + B2 ** 2)) - math.atan(B2 / A2) + math.pi
        # self.theta1 = (180 - 73.75)/180*math.pi
        print("theta1:",self.theta1/math.pi*180)
        #unkown why there is a pi bias, but it works

    def calculate_d12(self):
        self.alpha = math.pi - self.beta1 - self.theta1
        print("alpha:",self.alpha/math.pi*180)
        k_norm_squared = torch.sqrt(torch.sum(self.k**2, dim=1))

        self.d = torch.zeros(3,3,dtype=torch.float32)
        self.d[0][2] = self.k[0][2] - torch.sqrt(self.l1**2 + self.k[0][2]**2 - k_norm_squared[0]**2)
        self.d[1][2] = self.k[1][2] - torch.sqrt(self.l1**2 + self.k[1][2]**2 - k_norm_squared[1]**2)

    def calculate_d3(self):
        P12_x = torch.tensor(self.h*math.sin(self.q2)*math.sin(self.q1) - self.l4*math.sin(self.q2)*math.sin(self.q1-self.alpha))
        P12_y = torch.tensor(self.h*math.cos(self.q1) - self.l4*math.cos(self.q1-self.alpha) - self.cy)
        P12_z = torch.tensor(self.h*math.cos(self.q2)*math.sin(self.q1) - self.l4*math.cos(self.q2)*math.sin(self.q1 - self.alpha) + self.p)
        A1 = P12_z


        B1 = P12_x**2 + P12_y**2 + P12_z**2 - self.l3**2

        print("A1:",A1)
        print("torch.sqrt(A1**2 - B1)",torch.sqrt(A1**2 - B1))
        self.d[2][2] = A1 - torch.sqrt(A1**2 - B1)



    # def calculate_gamma2(self):
    #     self.gamma1 = self.theta3 - self.beta2 + self.beta3 - math.pi/2
    #     A3 = 2*self.u1*self.u2*math.sin(self.gamma1)
    #     B3 = 2*self.u1*self.u2*math.cos(self.gamma1) + 2*self.u2*self.u3
    #     C3 = self.u4**2 - self.u1**2 - self.u2**2 - self.u3**2 - 2*self.u1*self.u3*math.cos(self.gamma1)
    #     self.gamma2 = math.asin(C3/math.sqrt(A3**2 + B3**2)) - math.atan(B3/A3)

    
    # def calculate_dip_position(self):
    #     DY = u1*math.cos(self.gamma1)
    #     DZ = u1*math.sin(self.gamma1)
    #     Ppip_dipSS = torch.tensor([0,DY,DZ],dtype=torch.float32)
    #     delta =  (self.gamma1 - self.beta3 + math.pi - self.theta3)
    #     self.r = torch.tensor([
    #         [1,0,0],
    #         [0,math.cos(delta),math.sin(delta)],
    #         [0,-math.sin(delta),math.cos(delta)]
    #     ], dtype=torch.float32)
    #     Ppip_dipS = self.r @ Ppip_dipSS
    #
    #     CV = r1*math.cos(self.theta1) + r2*math.cos(theta2) + r3*math.cos(self.theta3)
    #     CW = r1*math.sin(self.theta1) + r2*math.sin(theta2) + r3*math.sin(self.theta3)
    #     Pp_pipS = torch.tensor([0,CV,CW],dtype=torch.float32) + torch.tensor([0,h,0],dtype=torch.float32);
    #     Pp_dipS = Pp_pipS + Ppip_dipS
    #
    #     self.Pp_dip = self.R @ Pp_dipS + torch.tensor([[0,0,self.p]],dtype=torch.float32)


if __name__ == "__main__":
    start_time = time.time()

    dhk = DexterousHandKinematics(ax,ay,bx,by,p,l1,l2,l3,l4,cy,h,r1,r2,r3,r4,u1,u2,u3,u4,beta1,beta2,beta3,theta4)


    dhk.update_kinematics(13.61/180*math.pi, 0, (180-45.02)/180*math.pi)
    print(dhk.d)

    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")

        #Time consumption: 21.78ms(the first run)

    # 43.03/180*math.pi , 0 , 45.1/180*math.pi => 0,0,0
    # 13.61/180*math.pi, 0, 45.02/180*math.pi               => -7.16,-7.16, -5.21