import torch
import math
d1=0.1
d2=0.1
d3=0.1
ax=0.1
ay=0.1
bx=0.1
by=0.1
p=0.1
q1=0
q2=0
l1=1
cy =2




def create_argument(ax,ay,bx,by,p,R):

    a = torch.tensor([[ax,ay,0],[-ax,ay,0]]).T
    b_ = torch.tensor([[bx,by,0],[-bx,by,0]]).T
    p = torch.tensor([[0,0,p]]).T
    k = p + R @ b_ - a
    return k.reshape(2,3,1)


def create_rotation_matrix(q1, q2):
    # 创建旋转矩阵 R
    R = torch.tensor([
        [math.cos(q2),      math.sin(q2)*math.sin(q1),    math.sin(q2)*math.cos(q1)],
        [0,                 math.cos(q1),                  -math.sin(q1)],
        [-math.sin(q2),     math.cos(q2)*math.sin(q1),    math.cos(q2)*math.cos(q1)]
    ], dtype=torch.float32)
    
    return R

def calculate_d12(d, k, l1):
    # d = torch.zeros(3,3)
    k_norm_squared = torch.sum(k**2, dim=0)
    d[0][2] = k[0][2] + torch.sqrt(l1**2 - k[0][2]**2 - k_norm_squared[0]**2)
    d[1][2] = k[1][2] + torch.sqrt(l1**2 - k[1][2]**2 - k_norm_squared[1]**2)
    print(d)

def calculate_d3(d, h,q1,q2,alpha,l3,l4, p,cy):
    P12_x = h*math.sin(q2)*math.sin(q1) - l4*math.sin(q2)*math.sin(q1-alpha)
    P12_y = h*math.cos(q1) - l4*math.cos(q1-alpha) - cy
    P12_z = h*math.cos(q2)*math.cos(q1) - l4*math.cos(q1 - alpha) + p
    A1 = P12_z
    B1 = P12_x**2 + P12_y**2 + P12_z**2 - l3**2
    d[2][2] = A1 + torch.sqrt(A1**2 - B1)
    print(d)
    
def main():
    R = create_rotation_matrix(q1=q1, q2=q2)
    k = create_argument(ax=ax,ay=ay,bx=bx,by=by,p=p, R=R)
    print(k)
    calculate_d12(k, l1)
if __name__ == "__main__":
    main()


    