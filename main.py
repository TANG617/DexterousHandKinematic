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

def create_rotation_matrix(q1, q2):
    R = torch.tensor([
        [math.cos(q2),      math.sin(q2)*math.sin(q1),    math.sin(q2)*math.cos(q1)],
        [0,                 math.cos(q1),                  -math.sin(q1)],
        [-math.sin(q2),     math.cos(q2)*math.sin(q1),    math.cos(q2)*math.cos(q1)]
    ], dtype=torch.float32)
    
    return R

def create_argument(ax,ay,bx,by,p,R):

    a = torch.tensor([[ax,ay,0],[-ax,ay,0]],dtype=torch.float32)
    b_ = torch.tensor([[bx,by,0],[-bx,by,0]],dtype=torch.float32)
    p = torch.tensor([[0,0,p]],dtype=torch.float32)
    k = torch.cat([p + R @ b_[0] - a[0], p + R @ b_[1] - a[1]], dim=0)
    return k


def calculate_d12(d, k, l1):
    k_norm_squared = torch.sqrt(torch.sum(k**2, dim=1))
    d[0][2] = k[0][2] - torch.sqrt(l1**2 + k[0][2]**2 - k_norm_squared[0]**2)
    d[1][2] = k[1][2] - torch.sqrt(l1**2 + k[1][2]**2 - k_norm_squared[1]**2)
    return d

def calculate_d3(d, h,q1,q2,alpha,l3,l4, p,cy):
    P12_x = torch.tensor(h*math.sin(q2)*math.sin(q1) - l4*math.sin(q2)*math.sin(q1-alpha))
    P12_y = torch.tensor(h*math.cos(q1) - l4*math.cos(q1-alpha) - cy)
    P12_z = torch.tensor(h*math.cos(q2)*math.sin(q1) - l4*math.cos(q2)*math.sin(q1 - alpha) + p)
    A1 = P12_z
    B1 = P12_x**2 + P12_y**2 + P12_z**2 - l3**2
    d[2][2] = A1 - torch.sqrt(A1**2 - B1)
    return d

def main():
    R = create_rotation_matrix(q1=q1, q2=q2)
    k = create_argument(ax=ax,ay=ay,bx=bx,by=by,p=p, R=R)
    d = torch.zeros(3,3,dtype=torch.float32)
    d = calculate_d12(d, k, l1)
    d = calculate_d3(d, h,q1,q2,alpha,l3,l4, p,cy)
    print("d1:",d[0][2].item())
    print("d2:",d[1][2].item())
    print("d3:",d[2][2].item())
if __name__ == "__main__":
    main()


    