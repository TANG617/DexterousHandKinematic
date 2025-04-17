# ILDA Dexterous Hand Kinematics Implementation

[![Python](https://img.shields.io/badge/Python-3.10-blue.svg)](https://www.python.org/downloads/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.6.0-ee4c2c.svg)](https://pytorch.org/get-started/locally/)
[![C](https://img.shields.io/badge/C-C99-lightgrey.svg)](https://en.cppreference.com/w/c)

This project reproduces and extends the kinematics model of the [ILDA Dexterous Hand](https://www.nature.com/articles/s41467-021-27261-0), providing complete forward kinematics calculations and workspace analysis capabilities. The implementation is available in both Python (with PyTorch) and C languages.

## Table of Contents

- [Overview](#overview)
- [Branches](#branches)
- [Technical Implementation](#technical-implementation)
  - [Python Implementation](#python-implementation)
  - [C Implementation](#c-implementation)
- [Installation and Usage](#installation-and-usage)
  - [Python Setup](#python-setup)
  - [C Setup](#c-setup)
- [Core Features](#core-features)
- [Performance Comparison](#performance-comparison)
- [Known Issues](#known-issues)
- [License](#license)

## Overview

The ILDA Dexterous Hand is a novel robotic hand system with high flexibility and precise motion control capabilities. This project implements the hand's kinematic model using both Python (with PyTorch) and C, based on the design parameters and kinematic equations from the original paper, with additional extensions.

![Dexterous Hand Core Structure](images/core.png)

## Branches

- `main`: Implementation of the paper's kinematic model in both Python and C
- `simulation`: ILDA Dexterous Hand simulation using NVIDIA Isaac Sim

## Technical Implementation

The project offers two separate implementations of the same kinematic model:

### Python Implementation

The Python version uses the following technology stack:

- **Python 3.10** - Core programming language
- **PyTorch 2.6.0** - For matrix operations and numerical calculations
- **NumPy 2.2.3** - Scientific computing and data processing
- **Matplotlib 3.10.1** - Data visualization
- **SciPy 1.15.2** - Additional scientific computing capabilities

### C Implementation

The C implementation provides:

- **Optimized Performance** - Faster computation with lower resource usage
- **Embedded Compatibility** - Suitable for resource-constrained systems and embedded applications
- **Cross-Platform Support** - Works on various operating systems and hardware platforms

The C codebase consists of:

- **dexterous_hand.h** - Header file containing structure definitions and function declarations
- **dexterous_hand.c** - Implementation of the kinematics algorithms
- **main.c** - Example application demonstrating usage of the library
- **main.h** - Header for the main application

## Installation and Usage

### Python Setup

#### Requirements

- Python 3.10+
- PyTorch 2.6.0+
- NumPy 2.2+
- Matplotlib 3.10+
- SciPy 1.15+

#### Installation

**Option 1: Using pip**

```bash
git clone https://github.com/yourusername/DexterousHandKinematic.git
cd DexterousHandKinematic
pip install -r requirements.txt
```

**Option 2: Using Conda (Recommended)**

```bash
git clone https://github.com/yourusername/DexterousHandKinematic.git
cd DexterousHandKinematic

# Create and activate conda environment
conda create -n dexterous_hand python=3.10
conda activate dexterous_hand

# Install packages
conda install pytorch torchvision torchaudio -c pytorch
conda install numpy matplotlib scipy
```

#### Python Usage Example

```python
from main import DexterousHandKinematics, degrees_to_radians

# Create dexterous hand model instance (with default parameters)
dhk = DexterousHandKinematics(
    ax=7.0, ay=12.0,
    bx=(18.15-2.5)/2, by=14.14,
    p=14.48,
    l1=24.19, l2=24.19, l3=13.75, l4=13.07,
    cy=0.0, h=14.14,
    r1=10.3, r2=26.10, r3=5.15, r4=30.59,
    u1=26.1, u2=6.08, u3=6.0, u4=27.0,
    beta1=degrees_to_radians(52.61),
    beta2=degrees_to_radians(47.84),
    beta3=degrees_to_radians(60.95),
    theta4=degrees_to_radians(56.2)
)

# Update joint angles and calculate kinematics
joint_displacements, dip_position = dhk.update(
    q1=degrees_to_radians(13.61),
    q2=0.0,
    q3=degrees_to_radians(180.0-45.02)
)

# Print results
print("Joint displacements:", joint_displacements)
print("DIP position:", dip_position)
```

### C Setup

#### Requirements

- C compiler (GCC, Clang, MSVC, etc.)
- Make (optional but recommended)
- Standard math library (math.h)

#### Compilation

**Using GCC directly:**

```bash
gcc -o dexterous_hand_demo main.c dexterous_hand.c -lm
```

**Using Make:**

Create a Makefile with the following content:

```makefile
CC = gcc
CFLAGS = -Wall -Wextra -O2
LDFLAGS = -lm

all: dexterous_hand_demo

dexterous_hand_demo: main.o dexterous_hand.o
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o dexterous_hand_demo
```

Then compile with:

```bash
make
```

#### C Usage Example

```c
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "dexterous_hand.h"

int main() {
    clock_t start, end;
    start = clock();

    // Initialize parameters
    double ax = 7, ay = 12;
    double bx = (18.15-2.5)/2, by = 14.14;
    double p = 14.48;
    double l1 = 24.19, l2 = 24.19;
    double l3 = 13.75, l4 = 13.07;
    double cy = 0;
    double h = by;
    double r1 = 10.3, r2 = 26.10, r3 = 5.15, r4 = 30.59;
    double u1 = 26.1, u2 = 6.08, u3 = 6, u4 = 27;
    double beta1 = 52.61 / 180 * M_PI;
    double beta2 = 47.84 / 180 * M_PI;
    double beta3 = 60.95 / 180 * M_PI;
    double theta4 = 56.2 / 180 * M_PI;

    // Create hand instance
    DexterousHand hand;
    init_dexterous_hand(&hand, ax, ay, bx, by, p, l1, l2, l3, l4, cy, h,
                        r1, r2, r3, r4, u1, u2, u3, u4,
                        beta1, beta2, beta3, theta4);

    // Update kinematics with test case parameters
    update_kinematics(&hand, 13.61/180.0*M_PI, 0, (180-45.02)/180.0*M_PI);
    
    // Print results
    printf("Joint displacements: [%.2f, %.2f, %.2f]\n",
           hand.d[0][2], hand.d[1][2], hand.d[2][2]);
    
    printf("DIP position: [%.2f, %.2f, %.2f]\n",
           hand.Pp_dip[0], hand.Pp_dip[1], hand.Pp_dip[2]);

    // Calculate execution time
    end = clock();
    double time_taken = ((double)(end - start)) / CLOCKS_PER_SEC;
    printf("Time taken: %f seconds\n", time_taken);

    return 0;
}
```

## Core Features

Both implementations provide the following functionalities:

1. **Forward Kinematics Calculation**: Compute the end-effector position and orientation based on given joint angles (q1, q2, q3)
2. **DIP Position Calculation**: Calculate the position of the finger's distal interphalangeal (DIP) joint in the global coordinate system
3. **Workspace Analysis**: Determine the reachable workspace of the dexterous hand by scanning different joint angle combinations

## Performance Comparison

The performance of Python (with PyTorch) and C implementations are compared below:

| Implementation | Single Calculation | Workspace Analysis (1000 poses) |
|----------------|-------------------:|--------------------------------:|
| Python/PyTorch | ~10ms              | ~2-3s                           |
| C              | ~0.1-0.5ms         | ~0.2-0.5s                       |

The C implementation offers approximately 10-20x speedup over the Python version, making it suitable for real-time applications and embedded systems. However, the Python version provides better visualization capabilities and easier integration with machine learning frameworks.

## C Implementation Details

### Code Structure

- **DexterousHand Struct**: Contains both parameters and internal state for the hand model
- **Function Organization**: 
  - `init_dexterous_hand()`: Initialize the hand with geometric parameters
  - `update_kinematics()`: Main entry point for updating calculations with new joint angles
  - Helper functions: Calculate specific aspects of the kinematics model

### Key Implementation Features

- **Memory Efficiency**: Uses stack allocation and avoids dynamic memory allocation
- **Matrix Operations**: Implements matrix multiplication and other operations manually
- **Error Handling**: Includes protections against invalid mathematical operations (e.g., square root of negative numbers)
- **Platform Independence**: No external dependencies beyond standard C libraries

## Known Issues

During implementation, we discovered some unclear or incorrect aspects in the original paper. These issues have been resolved through engineering approaches, but we are still investigating the root causes:

### 1. Angle Relationships

According to the geometric relationships:

$\alpha + \theta_1 + \beta_1 = \pi$

If $\beta_1 = 2\pi/9$, then $\alpha + \theta_1 = \pi - 2\pi/9 = 7\pi/9$

![Issue 1](images/issue1.png)

### 2. Gamma1 Calculation

The equation for calculating gamma1 in the original paper appears to be incorrect. The correct equation should be:

$\gamma_1 = \theta_3 - \beta_2 + \beta_3 - \pi/2$

![Issue 2](images/issue2.png)

### 3. Vector Directions

According to the figure, vector $\vec{r}_4$ has the opposite direction compared to the other vectors. Referencing the figure in the main text, the correct equation should be:

$\vec{r}_1 + \vec{r}_2 + \vec{r}_3 + \vec{r}_4 = 0$

The same issue occurs in the equation concerning $\vec{u}$.

![Issue 3-1](images/issue3_1.png)
![Issue 3-2](images/issue3_2.png)

### 4. Theta1 Calculation

In the code, $\theta_1$ is calculated with an additional $\pi$ term:

```python
self.theta1 = math.asin(C2/math.sqrt(A2**2 + B2**2)) - math.atan(B2/A2) + math.pi 
```

This compensation value was determined based on experimental results. While its mathematical basis is not yet clear, it produces the correct results.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## References

[1] ILDA dexterous hand kinematics. *Nature Communications*. [DOI: 10.1038/s41467-021-27261-0](https://www.nature.com/articles/s41467-021-27261-0)

