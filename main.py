import torch
import math

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
# alpha = float(math.pi/3)
alpha = 0

class DexterousHandKinematics:
    def __init__(self, ax, ay, bx, by, p, q1, q2, l1, l2, l3, l4, cy, h, alpha):
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

        self.create_rotation_matrix()
        self.create_argument()
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

def main():
    dhk = DexterousHandKinematics(ax,ay,bx,by,p,q1,q2,l1,l2,l3,l4,cy,h,alpha)
    print(dhk.d)
    
if __name__ == "__main__":
    main()


    