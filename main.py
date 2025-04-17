#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dexterous Hand Kinematics Simulator

This module implements the forward and inverse kinematics calculations
for a dexterous robotic hand mechanism.
"""

import time
from typing import Tuple, Optional
import math

import torch
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class DexterousHandKinematics:
    """
    A class to perform kinematics calculations for a dexterous robotic hand.
    
    This class implements the mathematical models needed to translate joint angles
    to end-effector positions and forces.
    """
    
    def __init__(
        self,
        ax: float, ay: float,          
        bx: float, by: float,          
        p: float,                      
        l1: float, l2: float,          
        l3: float, l4: float,          
        cy: float, h: float,           
        r1: float, r2: float,          
        r3: float, r4: float,          
        u1: float, u2: float,          
        u3: float, u4: float,          
        beta1: float, beta2: float,    
        beta3: float, theta4: float    
    ):
        """
        Initialize the dexterous hand kinematics model with geometric parameters.
        
        Args:
            Reference: https://www.nature.com/articles/s41467-021-27261-0
            The parameters are from the paper.
        """
        # Store all parameters
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
        self.beta1 = beta1
        self.beta2 = beta2
        self.beta3 = beta3
        self.theta4 = theta4
        
        # Initialize state variables
        self.R = None
        self.k = None
        self.d = None
        self.alpha = None
        self.theta1 = None
        self.theta2 = None
        self.theta3 = None
        self.gamma1 = None
        self.gamma2 = None
        self.q1 = None
        self.q2 = None
        self.Pp_dip = None

    def calculate_rotation_matrix(self) -> None:
        """
        Calculate the rotation matrix based on joint angles q1 and q2.
        
        This creates a 3x3 rotation matrix using the current q1 and q2 values.
        """
        self.R = torch.tensor([
            [math.cos(self.q2),       math.sin(self.q2)*math.sin(self.q1),    math.sin(self.q2)*math.cos(self.q1)],
            [0,                       math.cos(self.q1),                     -math.sin(self.q1)],
            [-math.sin(self.q2),      math.cos(self.q2)*math.sin(self.q1),    math.cos(self.q2)*math.cos(self.q1)]
        ], dtype=torch.float32)

    def calculate_k(self) -> None:
        """
        Calculate the k vectors used in the kinematics equations.
        
        This computes the position vectors needed for subsequent calculations.
        """
        # Define base and platform points
        a = torch.tensor([[self.ax, self.ay, 0], [-self.ax, self.ay, 0]], dtype=torch.float32)
        b_prime = torch.tensor([[self.bx, self.by, 0], [-self.bx, self.by, 0]], dtype=torch.float32)
        p_vec = torch.tensor([[0, 0, self.p]], dtype=torch.float32)
        
        # Calculate k vectors by concatenating transformed positions
        self.k = torch.cat([
            p_vec + self.R @ b_prime[0] - a[0],
            p_vec + self.R @ b_prime[1] - a[1]
        ], dim=0)

    def calculate_theta1(self) -> None:
        """
        Calculate theta1 angle based on four-bar linkage geometry.
        """
        # Calculate intermediate parameters
        A2 = 2 * self.r1 * (self.r3 * math.sin(self.theta3) - self.r4 * math.sin(self.theta4))
        B2 = 2 * self.r1 * (self.r3 * math.cos(self.theta3) - self.r4 * math.cos(self.theta4))
        C2 = (self.r2**2 - self.r1**2 - self.r3**2 - self.r4**2 + 
              2 * self.r3 * self.r4 * math.cos(self.theta3 - self.theta4))
        
        # Calculate theta1 angle
        self.theta1 = math.asin(C2 / math.sqrt(A2**2 + B2**2)) - math.atan(B2 / A2) + math.pi
        # Note: Pi offset is empirically determined to match expected behavior

    def calculate_theta2(self) -> None:
        """
        Calculate theta2 angle from the four-bar linkage geometry.
        """
        cosine_term = ((self.r4 * math.cos(self.theta4) - 
                      self.r3 * math.cos(self.theta3) - 
                      self.r1 * math.cos(self.theta1)) / self.r2)
        
        # Ensure value is within valid range for arccos
        cosine_term = max(min(cosine_term, 1.0), -1.0)
        self.theta2 = math.acos(cosine_term)

    def calculate_gamma2(self) -> None:
        """
        Calculate gamma2 angle for the finger linkage.
        """
        # Calculate gamma1 first
        self.gamma1 = self.theta3 - self.beta2 + self.beta3 - math.pi/2
        
        # Calculate intermediate parameters
        A3 = 2 * self.u1 * self.u2 * math.sin(self.gamma1)
        B3 = 2 * self.u1 * self.u2 * math.cos(self.gamma1) + 2 * self.u2 * self.u3
        C3 = (self.u4**2 - self.u1**2 - self.u2**2 - self.u3**2 - 
              2 * self.u1 * self.u3 * math.cos(self.gamma1))
        
        # Calculate gamma2
        self.gamma2 = math.asin(C3 / math.sqrt(A3**2 + B3**2)) - math.atan(B3 / A3)

    def calculate_d12(self) -> None:
        """
        Calculate d1 and d2 values (joint displacements for first two joints).
        """
        # Calculate alpha angle
        self.alpha = math.pi - self.beta1 - self.theta1
        
        # Calculate the norms of position vectors
        k_norm = torch.sqrt(torch.sum(self.k**2, dim=1))
        
        # Initialize displacement matrix if not already created
        if self.d is None:
            self.d = torch.zeros(3, 3, dtype=torch.float32)
        
        # Calculate d1 and d2
        self.d[0][2] = self.k[0][2] - torch.sqrt(self.l1**2 + self.k[0][2]**2 - k_norm[0]**2)
        self.d[1][2] = self.k[1][2] - torch.sqrt(self.l1**2 + self.k[1][2]**2 - k_norm[1]**2)

    def calculate_d3(self) -> None:
        """
        Calculate d3 value (joint displacement for third joint).
        """
        # Calculate intermediate position parameters
        P12_x = torch.tensor(
            self.h * math.sin(self.q2) * math.sin(self.q1) - 
            self.l4 * math.sin(self.q2) * math.sin(self.q1 - self.alpha)
        )
        P12_y = torch.tensor(
            self.h * math.cos(self.q1) - 
            self.l4 * math.cos(self.q1 - self.alpha) - self.cy
        )
        P12_z = torch.tensor(
            self.h * math.cos(self.q2) * math.sin(self.q1) - 
            self.l4 * math.cos(self.q2) * math.sin(self.q1 - self.alpha) + self.p
        )
        
        # Calculate d3 using quadratic equation solution
        A1 = P12_z
        B1 = P12_x**2 + P12_y**2 + P12_z**2 - self.l3**2
        
        # Ensure we don't take square root of negative number
        discriminant = A1**2 - B1
        if discriminant < 0:
            raise ValueError(f"Invalid parameters: discriminant = {discriminant}")
            
        self.d[2][2] = A1 - torch.sqrt(discriminant)

    def calculate_dip_position(self) -> None:
        """
        Calculate the Distal Interphalangeal (DIP) joint position.
        """
        # Calculate finger segment parameters
        DY = self.u1 * math.cos(self.gamma1)
        DZ = self.u1 * math.sin(self.gamma1)
        Ppip_dipSS = torch.tensor([0, DY, DZ], dtype=torch.float32)
        
        # Calculate delta angle
        delta = (self.gamma1 - self.beta3 + math.pi - self.theta3)
        
        # Create rotation matrix
        self.r = torch.tensor([
            [1, 0, 0],
            [0, math.cos(delta), math.sin(delta)],
            [0, -math.sin(delta), math.cos(delta)]
        ], dtype=torch.float32)
        
        # Calculate transformed position
        Ppip_dipS = self.r @ Ppip_dipSS
        
        # Calculate intermediate positions
        CV = (self.r1 * math.cos(self.theta1) + 
              self.r2 * math.cos(self.theta2) + 
              self.r3 * math.cos(self.theta3))
        CW = (self.r1 * math.sin(self.theta1) + 
              self.r2 * math.sin(self.theta2) + 
              self.r3 * math.sin(self.theta3))
              
        Pp_pipS = torch.tensor([0, CV, CW], dtype=torch.float32) + torch.tensor([0, self.h, 0], dtype=torch.float32)
        Pp_dipS = Pp_pipS + Ppip_dipS
        
        # Calculate final DIP position
        self.Pp_dip = self.R @ Pp_dipS + torch.tensor([[0, 0, self.p]], dtype=torch.float32)

    def update(self, q1: float, q2: float, q3: float) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Update kinematics calculations with new joint angles.
        
        Args:
            q1: First joint angle (radians)
            q2: Second joint angle (radians)
            q3: Third joint angle (theta3, radians)
            
        Returns:
            Tuple containing joint displacements and DIP position
        """
        # Store joint angles
        self.q1 = q1
        self.q2 = q2
        self.theta3 = q3
        
        try:
            # Execute calculation pipeline
            self.calculate_rotation_matrix()
            self.calculate_k()
            self.calculate_theta1()
            
            # Initialize displacement tensor if not already created
            if self.d is None:
                self.d = torch.zeros(3, 3, dtype=torch.float32)
                
            # Calculate remaining parameters
            self.calculate_theta2()
            self.calculate_gamma2()
            self.calculate_d12()
            self.calculate_d3()
            self.calculate_dip_position()
            
            return self.d, self.Pp_dip
            
        except Exception as e:
            print(f"Error in kinematics update: {e}")
            return None, None


def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0


def main():
    """Main function to demonstrate the dexterous hand kinematics."""
    # Define hand parameters
    # Base frame parameters
    ax, ay = 7.0, 12.0
    
    # Platform frame parameters (bx = bu; by = bv)
    bx, by = (18.15 - 2.5) / 2, 14.14
    
    # Platform height
    p = 14.48
    
    # Link lengths
    l1, l2 = 24.19, 24.19
    l3, l4 = 13.75, 13.07
    
    # Geometric parameters
    cy = 0.0
    h = by 
    
    r1, r2 = 10.3, 26.10
    r3, r4 = 5.15, 30.59

    u1, u2 = 26.1, 6.08
    u3, u4 = 6.0, 27.0
    
    # Joint angle parameters (in radians)
    beta1 = degrees_to_radians(52.61)
    beta2 = degrees_to_radians(47.84)
    beta3 = degrees_to_radians(60.95)
    theta4 = degrees_to_radians(56.2)
    
    # Create kinematics instance
    dhk = DexterousHandKinematics(
        ax, ay, bx, by, p, l1, l2, l3, l4, cy, h,
        r1, r2, r3, r4, u1, u2, u3, u4,
        beta1, beta2, beta3, theta4
    )
    
    # Test case parameters
    test_q1 = degrees_to_radians(13.61)
    test_q2 = 0.0
    test_q3 = degrees_to_radians(180.0 - 45.02)
    
    # Measure performance
    start_time = time.time()
    
    # Update kinematics with test case
    joint_displacements, dip_position = dhk.update(
        q1=test_q1,
        q2=test_q2,
        q3=test_q3
    )
    
    # Print results
    print("Joint displacements:")
    print(joint_displacements)
    print("\nDIP position:")
    print(dip_position)
    
    # Calculate and print execution time
    execution_time = time.time() - start_time
    print(f"\nExecution time: {execution_time*1000:.2f} ms")
    
    # Example test cases and expected results:
    # q1=43.03°, q2=0°, q3=45.1° => d = [0, 0, 0]
    # q1=13.61°, q2=0°, q3=45.02° => d = [-7.16, -7.16, -5.21]


if __name__ == "__main__":
    main()