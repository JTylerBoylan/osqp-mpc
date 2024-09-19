# `OSQP-MPC` C++ Single Header Library
*Made by Jonathan Tyler Boylan*

`OSQP-MPC` is a lightweight, single-header C++ library designed to simplify the use of the OSQP solver for solving Model Predictive Control (MPC) problems. This library converts MPC problems into a quadratic programming (QP) form that can be efficiently solved using OSQP, a popular and highly optimized QP solver. Additionally, the library leverages the powerful Eigen library for handling vectors and matrices, ensuring ease of use and high performance.

## Prerequisites

### [CMake](https://cmake.org/)
CMake is a cross-platform build system generator that simplifies the process of managing project dependencies and configuring the build environment. It automates the detection of necessary libraries, such as Eigen and OSQP, and ensures that your project is correctly compiled and linked. By using CMake, you can easily manage complex C++ imports and ensure a consistent build process across different platforms.

#### Linux Installation
```
sudo apt install cmake
```

### [Eigen](https://eigen.tuxfamily.org/)

Eigen is a highly optimized C++ library for linear algebra, providing tools for matrix and vector operations. It is essential for handling the mathematical computations involved in formulating and solving MPC problems. Eigen simplifies working with matrices and vectors in C++, offering a clean and efficient API that integrates seamlessly with the rest of your project.

#### Linux Installation
```
sudo apt install libeigen3-dev
```

### [OSQP](https://osqp.org/)
OSQP (Operator Splitting Quadratic Program) is a state-of-the-art solver for quadratic programming (QP) problems. It is particularly well-suited for solving the QP formulations that arise in Model Predictive Control (MPC). OSQP is efficient, robust, and designed to handle large-scale problems, making it a perfect fit for real-time optimization tasks in control applications. Integrating OSQP into your project allows for fast and reliable solutions to QP problems, essential for implementing MPC.

#### Linux Installation
```
export OSQP_PATH=${HOME}/.osqp
git clone https://github.com/osqp/osqp ${OSQP_PATH}
mkdir ${OSQP_PATH}/build && cd ${OSQP_PATH}/build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release ..
make && sudo make install
```

## Installation

### 1. Download and move the header file into your project

Download the `osqp_mpc.hpp` header file and place it in the `include/` directory of your project. This header contains all the necessary code to formulate and solve MPC problems using OSQP and Eigen.

### 2. Add Eigen and OSQP dependencies to your CMakeLists.txt file

In your `CMakeLists.txt`, ensure that Eigen and OSQP are correctly linked by adding the following configuration:

```
cmake_minimum_required(VERSION 3.10)
project(my_project)

# ... compiler flags

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_executable(my_executable src/main.cpp)
target_link_libraries(my_executable Eigen3::Eigen osqp::osqp)

# ... other executables

```

## Usage

This section will guide you through the steps to use the `OSQP-MPC` library to formulate an MPC problem, convert it to a QP problem, solve it using OSQP, and finally, use the results.

### Include the header file
To begin, include the `osqp_mpc.hpp` header file in your source file:
```
#include "osqp_mpc.hpp"
```

### MPC Problem Formulation

Model Predictive Control (MPC) is a powerful control strategy used in various applications, ranging from robotics to process control. MPC optimizes control inputs over a future time horizon to minimize a cost function while satisfying system dynamics and constraints.

In an MPC problem, you aim to minimize a cost function that typically penalizes deviations from desired reference states and the use of control inputs. The optimization is subject to the system's dynamics and constraints on both states and control inputs.

The general form of the MPC problem is as follows:

#### Cost Function
The objective is to minimize the following quadratic cost function:

```math
\min_{\mathbf{x}, \mathbf{u}} \sum_{i=1}^{N} \left( \mathbf{x}_i - \mathbf{x}_{\text{ref},i} \right)^\top \mathbf{Q}_i \left( \mathbf{x}_i - \mathbf{x}_{\text{ref},i} \right) + \sum_{i=1}^{N-1} \mathbf{u}_i^\top \mathbf{R}_i \mathbf{u}_i 
```

- $\mathbf{x}_i \in \mathbb{R}^{n_x}$: The state vector at time step $i$.
- $\mathbf{u}_i \in \mathbb{R}^{n_u}$: The control input vector at time step $i$.
- $\mathbf{x}_{\text{ref},i} \in \mathbb{R}^{n_x}$: The reference state vector at time step $i$.
- $\mathbf{Q}_i \in \mathbb{R}^{n_x \times n_x}$: The state cost matrix at time step $i$.
- $\mathbf{R}_i \in \mathbb{R}^{n_u \times n_u}$: The control input cost matrix at time step $i$.
- $N$: The prediction horizon or window size.
- $n_x$: Number of states *(Constant along prediction horizon)*
- $n_u$: Number of control inputs *(Can vary between nodes)*

#### System Dynamics
The evolution of the state vector over time is governed by the system dynamics:
```math
\mathbf{x}_{i+1} = \mathbf{A}_i\mathbf{x}_i + \mathbf{B}_i\mathbf{u}_i
```

- $\mathbf{A}_i \in \mathbb{R}^{n_x \times n_x}$: The state transition matrix at time step $i$.
- $\mathbf{B}_i \in \mathbb{R}^{n_x \times n_u}$: The control input matrix at time step $i$.

#### State and Control Constraints
The state vector is subject to constraints at each time step:

```math
\mathbf{lb}_{x,i} \leq \mathbf{C}_i\mathbf{x}_i \leq \mathbf{ub}_{x,i}
```

- $\mathbf{C}_{x,i} \in \mathbb{R}^{n_c \times n_x}$: The state constraint matrix at time step $i$.
- $\mathbf{lb}_{x,i} \in \mathbb{R}^{n_c}$: The lower bound on the state at time step $i$.
- $\mathbf{ub}_{x,i} \in \mathbb{R}^{n_c}$: The upper bound on the state at time step $i$.
- $n_c$: Number of state constraints *(Can vary between nodes)*

And the control input vector is also subject to constraints:

```math
\mathbf{lb}_{u,i} \leq \mathbf{D}_i\mathbf{u}_i \leq \mathbf{ub}_{u,i}
```

- $\mathbf{D}_i \in \mathbb{R}^{n_d \times n_u}$: The control input constraint matrix at time step $i$.
- $\mathbf{lb}_{u,i} \in \mathbb{R}^{n_d}$: The lower bound on the control input at time step $i$.
- $\mathbf{ub}_{u,i} \in \mathbb{R}^{n_d}$: The upper bound on the control input at time step $i$.
- $n_d$: Number of control input constraints *(Can vary between nodes)*

#### Example Usage
To formulate an MPC problem with a specific prediction horizon or window size, you can initialize the `MPCProblem` struct as follows:

```
std::size_t window_size = 21;
MPCProblem mpc(window_size);
```

This initializes the `MPCProblem` with the specified prediction horizon. 

Next, you can fill in the matrices at every time point in the horizon window based on your MPC problem formulation:

```
mpc.x0 = /* get initial state */;
for (std::size_t i = 0; i < window_size; i++)
{
    mpc.xref[i] = /* get xref @ node i */;
    mpc.Q[i] = /* get Q @ node i */;
    mpc.C[i] = /* get C @ node i */;
    mpc.lbx[i] = /* get lbx @ node i */;
    mpc.ubx[i] = /* get ubx @ node i */;
    if (i < window_size - 1)
    {
        mpc.R[i] = /* get R @ node i */;
        mpc.A[i] = /* get A @ node i */;
        mpc.B[i] = /* get B @ node i */;

        mpc.D[i] = /* get D @ node i */;
        mpc.lbu[i] = /* get lbu @ node i */;
        mpc.ubu[i] = /* get ubu @ node i */;
    }
}
```

This setup allows you to define the dynamics, costs, and constraints for each time step in your MPC problem, ensuring that the solver has all the necessary information to compute the optimal control actions.

#### Important Note about State and Control Dynamics

When working with Model Predictive Control (MPC), it's important to recognize that the state and control dynamic matrices used in the MPC formulation are different from the continuous-time state-space matrices commonly used in control theory.

In control theory, the continuous-time dynamics of a system are typically represented in state-space form as:

```math
\mathbf{\dot{x}}(t) = \mathbf{\hat{A}}(t)\mathbf{x}(t) + \mathbf{\hat{B}}(t)\mathbf{u}(t)
```

- $\mathbf{\dot{x}}$: The derivative of the state vector with respect to time.
- $\mathbf{\hat{A}}$: The continuous-time state transition matrix.
- $\mathbf{\hat{B}}$: The continuous-time control input matrix.
- $\mathbf{x}$: The state vector.
- $\mathbf{u}$: The control input vector.

However, in MPC, the problem is typically formulated in discrete time. The continuous-time dynamics need to be discretized to be used within the MPC framework. The discrete-time state and control matrices, $\mathbf{A}_i$ and $\mathbf{B}_i$, are related to the continuous-time matrices as follows:

```math
\mathbf{A}_i = \mathbf{\hat{A}}(t_i)\Delta T + \mathbf{I}^{n_x}
```

```math
\mathbf{B}_i = \mathbf{\hat{B}}(t_i)\Delta T
```

- $\Delta T$: The time step between nodes.
- $t_i$: The time at time step $i$ ($= \Delta T \times i$)
- $\mathbf{I}^{n_x}$: The identity matrix of size $n_x$, which accounts for the fact that in discrete time, the state at the next time step includes the current state plus the change due to the system dynamics over the time step $\Delta T$.

This discretization process transforms the continuous-time state-space model into a form that can be used in the discrete-time optimization problem of MPC. The matrices $\mathbf{A}_i$ and $\mathbf{B}_i$ are key components of the MPC formulation, defining how the system evolves from one time step to the next under the influence of the control inputs.

Understanding this distinction is crucial when setting up your MPC problem, as it ensures that the system's behavior is accurately represented in the discrete-time domain used by the MPC algorithm.

In control theory, the feedback equation of a system is also included in state-space form and is typically represented as:

```math
\mathbf{y}(t) = \mathbf{\hat{C}}(t)\mathbf{x}(t) + \mathbf{\hat{D}}(t)\mathbf{u}(t)
```

However, these matrices are in no way correlated to the $\mathbf{C}$ and $\mathbf{D}$ matrices in the MPC formulation, which instead represent constraints on the state and control input decision vectors.

### QP Problem Formulation

Once the Model Predictive Control (MPC) problem is defined, it needs to be converted into a Quadratic Programming (QP) problem that can be efficiently solved using a solver like OSQP. The QP problem is a specific type of optimization problem where the objective is a quadratic function, and the constraints are linear.

The general form of the QP problem derived from an MPC formulation is as follows:

#### Objective Function
The objective is to minimize a quadratic function with respect to the decision variable vector $\mathbf{z}$:

```math
\min_{\mathbf{z}} \mathbf{z}^\top \mathbf{H} \mathbf{z} + \mathbf{g}^\top\mathbf{z}
```

- $\mathbf{z} \in \mathbb{R}^{n}$: The decision variable vector, which typically includes both state and control variables over the prediction horizon.
- $\mathbf{H} \in \mathbb{R}^{n \times n}$: The Hessian matrix, representing the quadratic cost terms. It is a sparse, symmetric, positive semi-definite matrix.
- $\mathbf{g} \in \mathbb{R}^{n}$: The gradient vector, representing the linear cost terms.
- $n$: The total number of states and control inputs.

#### Constraints

The optimization is subject to a set of linear inequality constraints:

```math
\mathbf{l}_c \leq \mathbf{A}_c\mathbf{z} \leq \mathbf{u}_c
```

- $\mathbf{A}_c \in \mathbb{R}^{m \times n}$: The constraint matrix, representing the linear relationships between the decision variables.
- $\mathbf{l}_c \in \mathbb{R}^{m}$: The lower bounds on the constraints.
- $\mathbf{u}_c \in \mathbb{R}^{m}$: The upper bounds on the constraints.
- $m$: The total number of constraints.

#### MPC Conversion

To convert the previously defined MPC problem into a QP problem that can be solved using OSQP, use the following function:

```
QPProblem qp = getQPProblem(mpc);
```

The function `getQPProblem` takes the MPC problem as input and returns a `QPProblem` struct. The `QPProblem` struct contains the matrices and vectors ($\mathbf{H}$, $\mathbf{g}$, $\mathbf{A}_c$, $\mathbf{l}_c$, and $\mathbf{u}_c$) necessary to solve the QP problem with OSQP.

This conversion allows the MPC problem to be efficiently handled by QP solvers, enabling real-time optimization in control applications.

### Solve using OSQP

Once the MPC or QP problem has been formulated, you can solve it using the OSQP solver. The solver requires specific settings that can be customized based on the needs of your application, such as toggling verbosity or adjusting convergence criteria.

#### Set Up OSQP Solver Settings

First, create and configure the OSQP solver settings:

```
OSQPSettings settings;
osqp_set_default_settings(&settings);
// settings.verbose = false;
```

- `settings`: This object holds the configuration parameters for the OSQP solver, such as the maximum number of iterations, tolerance values, and verbosity. The default settings are often sufficient, but they can be customized as needed.
- `osqp_set_default_settings`: This function initializes the settings with default values. For instance, you can set `settings->verbose = false;` to disable solver output during the solving process.

#### Solve the MPC Problem Directly
If you're working directly with an `MPCProblem`, you can solve it using the `solveMPC` function:

```
MPCSolution mpc_sol = solveMPC(mpc, &settings)
```

- `mpc_sol`: This object contains the solution to the MPC problem, including optimal state and control trajectories over the prediction horizon.
- `solveMPC`: This function handles the entire process of converting the MPC problem into a QP problem and then solving it using OSQP. The result is returned as an `MPCSolution`.

#### Solve the QP Problem

Alternatively, if you've already converted your MPC problem into a QP problem, you can solve it directly using the `solveOSQP` function:

```
QPSolution qp_sol = solveOSQP(qp, &settings);
```

- `qp_sol`: This object contains the solution to the QP problem, including the optimal decision variables.
- `solveOSQP`: This function solves the QP problem using OSQP and returns the solution as a `QPSolution`.

### Interpreting the Results

After solving the MPC problem using OSQP, the solution provides the optimal state and control trajectories over the prediction horizon. To implement these results in your physical system, you typically extract the first control input from the solution and apply it to your system.

```
OSQPVector ctrl = mpc_sol.ustar[0];
// ... send ctrl to physical system
```

- `ctrl`: This is the optimal control input that should be applied to your system at the current time step.

#### Applying the Control Input

Once extracted, the control input can be sent to the physical system (e.g., a robot, a motor controller, or any other actuated system). After applying this control input, you would typically re-solve the MPC problem at the next time step with updated state information, following the receding horizon approach characteristic of MPC.

This process ensures that your system continually adapts to changing conditions and stays on track to meet its objectives.

## Examples

### [Inverted Pendulum on a Cart MPC Example](examples/inverted_pendulum.cpp)

This example demonstrates how to use the OSQP-MPC library to control an inverted pendulum on a cart using Model Predictive Control (MPC). The example is based on the system dynamics of an inverted pendulum as described in [this resource](https://ctms.engin.umich.edu/CTMS/?example=InvertedPendulum&section=SystemModeling).

### [Benchmarking](examples/benchmarking.cpp)

This benchmarking example demonstrates the performance of the `OSQP-MPC` library across varying prediction horizon sizes. The purpose of this benchmark is to measure the time required for generating the MPC problem, converting it into a QP problem, and solving it using the OSQP solver. The example uses randomly generated MPC problems with varying horizon window sizes to test the scalability and efficiency of the solver.

The benchmark outputs the timing results for each step and the estimated frequency at which the MPC problem can be solved. This information is crucial for understanding the real-time performance capabilities of the OSQP-MPC library when applied to systems with varying complexities.

#### *Note*
*This README was largely generated with the assistance of ChatGPT 4, an AI language model developed by OpenAI. It was used to streamline the documentation process and ensure clarity and consistency in explaining the library's usage and features.*