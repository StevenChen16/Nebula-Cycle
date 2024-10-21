# Kinematic Modeling Report (with Detailed Parameters)

## 1. Introduction

This report details the kinematic modeling of a bicycle, including the effects of damping, road slope, and control torque using Lagrangian mechanics. The goal is to simulate the dynamic behavior under varying conditions and understand how different parameters impact stability and maneuverability.

## 2. Model Parameters

### 2.1 Overall Bicycle Parameters

- **Total Mass:** $15.79 \, \text{kg}$
- **Center of Mass Position:** $[0.000, -0.103, -0.019] \, \text{m}$
- **Inertia Tensor (about the center of mass, in kg·m²):**

$$
I = \begin{bmatrix}
0.8161 & 0.3963 & 0.0416 \\
0.3963 & 2.3674 & 0.0033 \\
0.0416 & 0.0033 & 3.0907 \\
\end{bmatrix}
$$

This tensor describes the bicycle's resistance to angular acceleration about each principal axis, which affects the dynamics when the bicycle yaws or the front wheel steers.

## 3. Lagrangian Equations

The system uses generalized coordinates:
- $\theta$: Yaw angle representing the bicycle's lateral inclination.
- $\phi$: Steering angle of the front wheel.

### 3.1 Equations of Motion

The Lagrangian mechanics formulation results in the following simplified equations of motion:

1. **Equation for $\theta$:**

$$
\begin{aligned}
109951162777600000000 \cdot \theta(t) + 54975581388800000000 \cdot \frac{d\theta(t)}{dt} = & 139358486919085293568 \cdot \frac{d^2\theta(t)}{dt^2} + 362838837166080000 \cdot \frac{d^2\phi(t)}{dt^2} \\
& + 109951162777600000000 \cdot \frac{d\theta(t)}{dt} \cdot (d_\theta + 0.02) \\
& + 139358486919085293568 \cdot \frac{d^2\theta(t)}{dt^2} + 2957475761360216796875
\end{aligned}
$$

2. **Equation for $\phi$:**

$$
\begin{aligned}
309640019 \cdot \frac{d^2\phi(t)}{dt^2} + 330000 \cdot \frac{d^2\theta(t)}{dt^2} + 100000000 \cdot \frac{d\phi(t)}{dt} \cdot (d_\phi + 0.02) = 80000000 \cdot \phi(t) + 30000000 \cdot \frac{d\phi(t)}{dt}
\end{aligned}
$$

These equations incorporate:
- **Damping Effects:** Represented by terms involving $d_\theta$ and $d_\phi$, which account for resistance forces from the environment, such as air drag and tire friction.
- **Control Torque:** Modeled using a PID control approach to simulate the rider's steering actions:

$$
\text{Control Torque for } \theta = Kp_\theta \cdot \theta + Kd_\theta \cdot \frac{d\theta}{dt}
$$

$$
\text{Control Torque for } \phi = Kp_\phi \cdot \phi + Kd_\phi \cdot \frac{d\phi}{dt}
$$

- **Road Slope:** Simulated by incorporating a 10-degree slope, which affects the yaw dynamics.

## 4. Simulation Results Analysis

The simulation tested different damping values (0.05, 0.1, and 0.2), as illustrated in the provided plots:
- **Observation:** As damping increases, the magnitude of oscillations in both $\theta$ and $\phi$ decreases, indicating that higher damping contributes to stabilizing the system faster. Lower damping allows for larger deviations and prolonged oscillations.
- **Yaw Angle Behavior ($\theta$):** Shows a significant deviation over time, which can be controlled using appropriate PID settings.
- **Steering Angle Behavior ($\phi$):** Stabilizes faster with increased damping, demonstrating the role of steering resistance.

## 5. Conclusion

The provided parameters and derived equations demonstrate the impact of damping, road conditions, and control torques on the bicycle's kinematic behavior. This modeling approach helps predict stability under different conditions and aids in designing control strategies for improved maneuverability.

Further analysis can include nonlinear effects, varying road conditions, and optimizing PID settings to improve the control performance.