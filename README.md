**Trajectory Tracking using State Feedback Linearization Control for a 2 DoF RRBot**

<!-- TOC -->

- [About](#about)
- [How to Design State Feedback Controller](#how-to-design-state-feedback-controller)
    - [Find the Equilibrium Points of the system:](#find-the-equilibrium-points-of-the-system)
    - [Linearize the System Using Equilibrium Points](#linearize-the-system-using-equilibrium-points)
    - [Check for stability](#check-for-stability)
    - [Check for Controllability](#check-for-controllability)
    - [Design State Feedback Controller](#design-state-feedback-controller)
- [Results](#results)
    - [MATLAB](#matlab)
    - [Gazebo and ROS](#gazebo-and-ros)
- [Observations and Results](#observations-and-results)
- [Designer Details](#designer-details)
- [License](#license)

<!-- /TOC -->

# About

The assignment aims to design State Feedback Controller for a 2-DoF Revolute Revolute Arm for Position Control.

![RRBot](./Docs/Images/RRBot.png)

The project uses the Equation of Motion derived in this [project](https://github.com/parth-20-07/2-DoF-Revolute-Revolute-robot-arm-Equation-of-Motion) for the identical RRBot.

The assignment aims to design a state feedback controller for position tracking for the RRBot. The controller is of the form:

$$
u = - Kx
$$

# How to Design State Feedback Controller

When taking a look at the dynamics of the system, we notice that the dynamics are non-linear. This makes it impossible to implement the controller. We can linearise the dynamics as follows:

## Find the Equilibrium Points of the system:

  Form the state space matrix for the system using the Equation of motion where
  
$$
\begin{equation}\notag
z = 
\begin{bmatrix}
z_{1}\\
z_{2}\\
z_{3}\\
z_{4}\\
\end{bmatrix}
\end{equation}
$$

where;

$$
\begin{equation}\notag
z_{1} = \theta_{1}
\end{equation}
$$

$$
\begin{equation}\notag
z_{2} = \dot{\theta_{1}}
\end{equation}
$$

$$
\begin{equation}\notag
\dot{z_{1}} = \dot{\theta_{1}} = z_{2}
\end{equation}
$$

$$
\begin{equation}\notag
\dot{z_{2}} = \ddot{\theta_{1}}
\end{equation}
$$

$$
\begin{equation}\notag
z_{3} = \theta_{2}
\end{equation}
$$

$$
\begin{equation}\notag
z_{4} = \dot{\theta_{2}}
\end{equation}
$$

$$
\begin{equation}\notag
\dot{z_{3}} = \dot{\theta_{2}} = z_{4}
\end{equation}
$$

$$
\begin{equation}\notag
\dot{z_{4}} = \ddot{\theta_{2}}
\end{equation}
$$

Thus;

$$
\begin{equation}\notag
\dot{z} = 
\begin{bmatrix}
\dot{z_{1}}\\
\dot{z_{2}}\\
\dot{z_{3}}\\
\dot{z_{4}}\\
\end{bmatrix} = 
\begin{bmatrix}
z_{2}\\
\ddot{\theta_{1}}\\
z_{4}\\
\ddot{\theta_{2}}
\end{bmatrix}
\end{equation}
$$

Equate $\dot{z} = 0$ with $\dot{\theta_{1}} = 0,\dot{\theta_{2}} = 0,\ddot{\theta_{1}} = 0,\ddot{\theta_{2}} = 0$ which related to arm being stationary at eqilibirum. Thus, the equilibrium positions can be found in the solution. The set of solutions is represented by $x^{*}$.

## Linearize the System Using Equilibrium Points

Using the non-linearized state space equation

$$
\begin{equation}\notag
\dot{z} = f(x,u)
\end{equation}
$$

$$
\begin{equation}\notag
y = h(x,u)
\end{equation}
$$

Create a new set of linearized state space equations using the equilibrium point of the form:

$$
\begin{equation}\notag
\dot{\bar{x}} = \bar{A}\bar{x} + \bar{B}\bar{u}\\
\end{equation}
$$

$$
\begin{equation}\notag
\bar{y} = \bar{C}\bar{x} + \bar{D}\bar{u}\\
\end{equation}
$$

where;

$$
\begin{equation}\notag
\bar{A} = \frac{\partial{f_{z}}}{\partial{z}}|_{z=z^{*}}
\end{equation}
$$

$$
\begin{equation}\notag
\bar{B} = \frac{\partial{f_{u}}}{\partial{u}}|_{u=u^{*}}
\end{equation}
$$

$$
\begin{equation}\notag
\bar{C} = \frac{\partial{h_{z}}}{\partial{z}}|_{z=z^{*}}
\end{equation}
$$

$$
\begin{equation}\notag
\bar{D} = \frac{\partial{h_{u}}}{\partial{u}}|_{u=u^{*}}
\end{equation}
$$

## Check for stability

Check the system stability using eigenvalues. If the Real part of any eigenvalue is greater than zero, the system is unstable for the found equilibrium point.

## Check for Controllability

Check the controllability of the system by deriving the $M_{C}$ matrix, the controllability matrix using:

$$
\begin{equation}\notag
M_{C} = 
\begin{bmatrix}
\bar{B} && \bar{A}\bar{B} && \bar{A}^{2}\bar{B} && \dots && \bar{A}^{n}\bar{B}
\end{bmatrix}
\end{equation}
$$

if $rank(C) = rank(A)$, the system for the equilibrium point is controllable.

## Design State Feedback Controller

For a controllable set of equilibrium points;

- Choose a set of complex poles (The real part of poles are less than zero).
- Find the $K$ for the controller using `K = place(linearA, linearB, poles)`.
- Design the controller of form $u = -Kx$

# Results

## MATLAB
The complete calculation has been done [here (webpage form)](https://htmlpreview.github.io/?https://github.com/parth-20-07/Position-Tracking-using-State-Feedback-Controller-Design-for-a-2-DoF-RRBot/blob/main/Solution/MATLAB/main.html) in MATLAB. The systems with the equation of motion are simulated as follows

![MATLAB Simulation](./Docs/MATLAB%20Simulation.gif)

## Gazebo and ROS

The same system is simulated with the equations of motion present in Gazebo with real-world physics replication as shown [here (webpage form)](https://htmlpreview.github.io/?https://github.com/parth-20-07/Position-Tracking-using-State-Feedback-Controller-Design-for-a-2-DoF-RRBot/blob/main/Solution/Gazebo/rrbot_control.html).

# Observations and Results

| Graph Type                | MATLAB                                                    | Gazebo                                                    |
| ------------------------- | --------------------------------------------------------- | --------------------------------------------------------- |
| $\theta_{1}$ vs $t$       | ![matlab_theta1_vs_t](./Solution/MATLAB/theta1.jpg)       | ![gazebo_theta1_vs_t](./Solution/Gazebo/theta_1.jpg)      |
| $\dot{\theta_{1}}$ vs $t$ | ![matlab_dtheta1_vs_t](./Solution/MATLAB/theta_dot_1.jpg) | ![gazebo_dtheta1_vs_t](./Solution/Gazebo/theta_dot_1.jpg) |
| $\tau_{1}$ vs $t$         | ![matlab_tau1_vs_t](./Solution/MATLAB/tau_1.jpg)          | ![gazebo_tau1_vs_t](./Solution/Gazebo/Tau_1.jpg)          |
| $\theta_{2}$ vs $t$       | ![matlab_theta2_vs_t](./Solution/MATLAB/theta2.jpg)       | ![gazebo_theta2_vs_t](./Solution/Gazebo/theta_2.jpg)      |
| $\dot{\theta_{2}}$ vs $t$ | ![matlab_dtheta2_vs_t](./Solution/MATLAB/theta_dot_2.jpg) | ![gazebo_dtheta2_vs_t](./Solution/Gazebo/theta_dot_2.jpg) |
| $\tau_{2}$ vs $t$         | ![matlab_tau2_vs_t](./Solution/MATLAB/tau_2.jpg)          | ![gazebo_tau2_vs_t](./Solution/Gazebo/tau_2.jpg)          |

**Possible Reasons for difference:**
- The lack of friction in the MATLAB System.
- The estimated value of gravitational acceleration in MATLAB.

# Designer Details

- Designed for:
  - Worcester Polytechnic Institute
  - RBE502 - Robot Control
- Designed by:
  - [Parth Patel](mailto:parth.pmech@gmail.com)

# License

This project is licensed under [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html) (see [LICENSE.md](LICENSE.md)).

Copyright 2023 Parth Patel

Licensed under the GNU General Public License, Version 3.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at

_https://www.gnu.org/licenses/gpl-3.0.en.html_

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
