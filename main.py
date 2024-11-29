import torch
import math
import time

ax=10
ay=20
bx=10
by=20
p=65
q1=0
q2=0
l1=20
l2=20
l3=30
l4=15
cy = 0
h = 30
alpha = 0

r1 = 15
r2 = 25
r3 = 7
r4 = 28
#
#theta1 = 135/360*math.pi*2
theta2 = 25.5/360*math.pi*2
theta3 = 121.8/360*math.pi*2
theta4 = 68/360*math.pi*2

u1 = 28
u2 = 7
u3 = 7
u4 = 31

gamma1 = 60/360*math.pi*2
# gamma2 = 275/360*math.pi*2
# gamma3 = 23/360*math.pi*2
# gamma3 = 180/360*math.pi*2
# gamma4 = 34/360*math.pi*2

beta1 = 45/360*math.pi*2
beta2 = 43/360*math.pi*2
beta3 = 41/360*math.pi*2

class DexterousHandKinematics:
    def __init__(self, ax, ay, bx, by, p, q1, q2, l1, l2, l3, l4, cy, h, alpha, r1, r2, r3, r4, theta2, theta3, theta4, u1, u2, u3, u4, gamma1, beta1, beta2, beta3):
        self.ax = ax
        self.ay = ay
        self.bx = bx
        self.by = by
        self.p = p
        self.q1 = q1
        self.q2 = q2
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.cy = cy
        self.h = h
        self.alpha = alpha
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
        self.r4 = r4
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4

        self.u1 = u1
        self.u2 = u2
        self.u3 = u3
        self.u4 = u4
        self.gamma1 = gamma1
        # self.gamma3 = gamma3
        # self.gamma4 = gamma4

        self.beta1 = beta1
        self.beta2 = beta2
        self.beta3 = beta3

        self.create_rotation_matrix()
        self.create_argument()
        self.calculate_gamma2()
        # self.gamma1 = self.theta3 - self.beta2 + self.beta3
        self.calculate_theta1() 
        # alpha = math.pi - self.beta1
        self.calculate_d12()
        self.calculate_d3()


    def create_rotation_matrix(self):
        self.R = torch.tensor([
            [math.cos(self.q2),      math.sin(self.q2)*math.sin(self.q1),    math.sin(self.q2)*math.cos(self.q1)],
            [0,                 math.cos(self.q1),                  -math.sin(self.q1)],
            [-math.sin(self.q2),     math.cos(self.q2)*math.sin(self.q1),    math.cos(self.q2)*math.cos(self.q1)]
        ], dtype=torch.float32)

    def create_argument(self):

        a = torch.tensor([[self.ax,self.ay,0],[-self.ax,self.ay,0]],dtype=torch.float32)
        b_ = torch.tensor([[self.bx,self.by,0],[-self.bx,self.by,0]],dtype=torch.float32)
        p = torch.tensor([[0,0,self.p]],dtype=torch.float32)
        self.k = torch.cat([p + self.R @ b_[0] - a[0], p + self.R @ b_[1] - a[1]], dim=0)


    def calculate_d12(self):
        self.d = torch.zeros(3,3,dtype=torch.float32)
        k_norm_squared = torch.sqrt(torch.sum(self.k**2, dim=1))
        self.d[0][2] = self.k[0][2] - torch.sqrt(self.l1**2 + self.k[0][2]**2 - k_norm_squared[0]**2)
        self.d[1][2] = self.k[1][2] - torch.sqrt(self.l1**2 + self.k[1][2]**2 - k_norm_squared[1]**2)

    def calculate_d3(self):
        P12_x = torch.tensor(self.h*math.sin(self.q2)*math.sin(self.q1) - self.l4*math.sin(self.q2)*math.sin(self.q1-self.alpha))
        P12_y = torch.tensor(self.h*math.cos(self.q1) - self.l4*math.cos(self.q1-self.alpha) - self.cy)
        P12_z = torch.tensor(self.h*math.cos(self.q2)*math.sin(self.q1) - self.l4*math.cos(self.q2)*math.sin(self.q1 - self.alpha) + self.p)
        A1 = P12_z
        B1 = P12_x**2 + P12_y**2 + P12_z**2 - self.l3**2
        self.d[2][2] = A1 - torch.sqrt(A1**2 - B1)

    def calculate_theta1(self):
        A2 = 2*self.r1*(self.r3*math.sin(self.theta3) - self.r4 * math.sin(self.theta4))
        B2 = 2*self.r1*(self.r3*math.cos(self.theta3) - self.r4 * math.cos(self.theta4))
        C2 = self.r2**2 - self.r1**2-self.r3**2 -self.r4**2 + 2*self.r3*self.r4*math.cos(self.theta3-self.theta4)
        self.theta1 = math.asin(C2/math.sqrt(A2**2 + B2**2)) - math.atan(B2/A2)
        print("theta1: ",self.theta1 + math.pi, "ref:", 135/360*math.pi*2)

    def calculate_gamma2(self):
        A3 = 2*self.u1*self.u2*math.sin(self.gamma1)
        B3 = 2*self.u1*self.u2*math.cos(self.gamma1) + 2*self.u2*self.u3
        C3 = self.u4**2 - self.u1**2 - self.u2**2 - self.u3**2 - 2*self.u1*self.u3*math.cos(self.gamma1)
        print("A3",A3,"B3",B3,"C3",C3)
        self.gamma2 = math.asin(C3/math.sqrt(A3**2 + B3**2)) - math.atan(B3/A3)
        print("gamma2: ",self.gamma2 + math.pi *2, "ref:", 275/360*math.pi*2)

def main():
    dhk = DexterousHandKinematics(ax,ay,bx,by,p,q1,q2,l1,l2,l3,l4,cy,h,alpha,r1,r2,r3,r4,theta2,theta3,theta4,u1,u2,u3,u4,gamma1,beta1,beta2,beta3)
    print(dhk.d)

if __name__ == "__main__":
    start_time = time.time()
    main()
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")


    