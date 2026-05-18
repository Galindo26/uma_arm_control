# uma_arm_control
This is the UMA arm control repo

## Launch 

```bash
ros2 launch uma_arm_description uma_arm_visualization.launch.py
ros2 launch uma_arm_control uma_arm_dynamics_launch.py
ros2 launch uma_arm_control gravity_compensation_launch.py
ros2 launch uma_arm_control dynamics_cancellation_launch.py
```

## Inverse Dynamics Control

### Gravity compensation
Method `gravity_compensation()` to calculate the desired torques:

```cpp
        // Method to calculate the desired joint torques
        Eigen::VectorXd gravity_compensation()
        {
            // Vector para los pares articulares
            Eigen::VectorXd torque(2);

            // Posiciones articulares actuales
            double q1 = joint_positions_(0);
            double q2 = joint_positions_(1);

            // Cálculo del vector de gravedad g(q)
            // Asume configuración planar estándar con ángulos desde la horizontal
            // y masas concentradas al final de l1 y l2.
            double g1 = (m1_ + m2_) * g_ * l1_ * std::cos(q1) + m2_ * g_ * l2_ * std::cos(q1 + q2);
            double g2 = m2_ * g_ * l2_ * std::cos(q1 + q2);

            // Asignación de los pares de control deseados
            torque << g1, g2;

            return torque;
        }
```

Resultado Manipulador:
![alt text](gravity_compensation.png)

Representacion con Rqt Graph:
![alt text](rqt_graph.png)

### Linearization by inverse dynamics control

When the desired manipulator motion requires large joint speeds and accelerations, the nonlinear coupling terms strongly influence the system performance. In these cases, a decentralized control strategy is not suitable. Instead, we use a centralized control approach known as **Inverse Dynamics Control** (or Feedback Linearization).

The dynamics of an $n$-joint manipulator can be described by the following equation:

$$M(q)\ddot{q} + n(q,\dot{q}) = \tau$$

where $n(q,\dot{q})$ groups all the nonlinear terms (Coriolis, centrifugal forces, viscous friction, and gravity):

$$n(q,\dot{q}) = C(q,\dot{q})\dot{q} + F_b\dot{q} + g(q)$$

The goal of the inverse dynamics control is to perform an exact linearization of the system dynamics by means of a nonlinear state feedback. We define our control input $\tau$ as a function of the manipulator state:

$$\tau = M(q)y + n(q,\dot{q})$$

By substituting this control law back into the dynamic model, the nonlinearities are perfectly cancelled, and the system is reduced to a simple, decoupled double integrator:

$$\ddot{q} = y$$

Once the system is linearized, $y$ represents a new input vector that dictates the desired dynamic behavior. We can now easily track a desired trajectory $q_d(t)$ by applying a stabilizing linear control law, such as a Proportional-Derivative (PD) controller:

$$y = \ddot{q}_d + K_D\dot{\tilde{q}} + K_P\tilde{q}$$

where $\tilde{q} = q_d - q$ is the tracking error, and $K_P$ and $K_D$ are positive definite diagonal matrices. This guarantees that the position error converges asymptotically to zero.

#### Euler-Lagrange Formulation for 2-DOF Planar Robot

To implement the exact dynamic cancellation, we derive the mathematical model using the **Euler-Lagrange** formulation. The Lagrangian $L$ is defined as the difference between the Kinetic Energy ($K$) and the Potential Energy ($P$):

$$L = K - P$$

The joint torques $\tau_i$ are obtained through the Euler-Lagrange equation:

$$\tau_i = \frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}_i} \right) - \frac{\partial L}{\partial q_i}$$

Assuming a standard 2-DOF planar manipulator with point masses $m_1$ and $m_2$ at the end of links of lengths $l_1$ and $l_2$:

**1. Potential Energy and Gravity Vector $g(q)$**
The potential energy depends only on the height (y-axis) of the masses:

$$P = m_1 g l_1 \sin(q_1) + m_2 g (l_1 \sin(q_1) + l_2 \sin(q_1 + q_2))$$

The gravity vector $g(q)$ is obtained by taking the partial derivative of $P$ with respect to each joint variable $q_i$:

$$g_1(q) = \frac{\partial P}{\partial q_1} = (m_1 + m_2) g l_1 \cos(q_1) + m_2 g l_2 \cos(q_1 + q_2)$$
$$g_2(q) = \frac{\partial P}{\partial q_2} = m_2 g l_2 \cos(q_1 + q_2)$$

**2. Kinetic Energy, Inertia Matrix $M(q)$, and Coriolis $C(q,\dot{q})$**
The total kinetic energy is the sum of the kinetic energies of the two point masses:

$$K = \frac{1}{2} m_1 v_1^2 + \frac{1}{2} m_2 v_2^2$$

By expressing the linear velocities $v_1$ and $v_2$ in terms of joint velocities $\dot{q}_1$ and $\dot{q}_2$, we get:

$$K = \frac{1}{2} \left[ (m_1+m_2)l_1^2 + m_2 l_2^2 + 2m_2 l_1 l_2 \cos(q_2) \right] \dot{q}_1^2 + \frac{1}{2} m_2 l_2^2 \dot{q}_2^2 + \left[ m_2 l_2^2 + m_2 l_1 l_2 \cos(q_2) \right] \dot{q}_1 \dot{q}_2$$

Applying the Euler-Lagrange operator to $K$ yields the Inertia Matrix $M(q)$:

$$M_{11} = (m_1+m_2)l_1^2 + m_2 l_2^2 + 2m_2 l_1 l_2 \cos(q_2)$$
$$M_{12} = M_{21} = m_2 l_2^2 + m_2 l_1 l_2 \cos(q_2)$$
$$M_{22} = m_2 l_2^2$$

And the Coriolis and centrifugal terms $C(q,\dot{q})\dot{q}$, defining $h = m_2 l_1 l_2 \sin(q_2)$:

$$C(q,\dot{q})\dot{q} = \begin{bmatrix} -h \dot{q}_2^2 - 2h \dot{q}_1 \dot{q}_2 \\ h \dot{q}_1^2 \end{bmatrix}$$

These derived terms are the ones directly used in the node implementation.

#### Implementation of `cancel_dynamics()`

```cpp
        // Method to calculate joint acceleration
        Eigen::VectorXd cancel_dynamics()
        {
            // Initialize M, C (C*q_dot), Fb, g_vec
            Eigen::MatrixXd M(2, 2);
            Eigen::VectorXd C_vec(2); 
            Eigen::VectorXd Fb_vec(2);
            Eigen::VectorXd g_vec(2);

            // Initialize q1, q2, q_dot1, and q_dot2
            double q1 = joint_positions_(0);
            double q2 = joint_positions_(1);
            double q_dot1 = joint_velocities_(0);
            double q_dot2 = joint_velocities_(1);
            
            // Extract desired joint accelerations (y in the control scheme)
            Eigen::VectorXd q_ddot_d = desired_joint_accelerations_;

            // Calculate matrix M (Inertia Matrix)
            M(0, 0) = (m1_ + m2_) * l1_ * l1_ + m2_ * l2_ * l2_ + 2.0 * m2_ * l1_ * l2_ * std::cos(q2);
            M(0, 1) = m2_ * l2_ * l2_ + m2_ * l1_ * l2_ * std::cos(q2);
            M(1, 0) = M(0, 1);
            M(1, 1) = m2_ * l2_ * l2_;

            // Calculate vector C (C is 2x1 because it already includes q_dot)
            // It represents Coriolis and centrifugal forces
            double h = m2_ * l1_ * l2_ * std::sin(q2);
            C_vec(0) = -h * q_dot2 * q_dot2 - 2.0 * h * q_dot1 * q_dot2;
            C_vec(1) = h * q_dot1 * q_dot1;

            // Calculate Fb vector (Viscous friction: Fb * q_dot)
            Fb_vec(0) = b1_ * q_dot1;
            Fb_vec(1) = b2_ * q_dot2;

            // Calculate g_vect (Gravity vector)
            g_vec(0) = (m1_ + m2_) * g_ * l1_ * std::cos(q1) + m2_ * g_ * l2_ * std::cos(q1 + q2);
            g_vec(1) = m2_ * g_ * l2_ * std::cos(q1 + q2);

            // Calculate control torque using the dynamic model: torque = M * q_ddot_d + C * q_dot + Fb * q_dot + g
            Eigen::VectorXd torque(2);
            torque = M * q_ddot_d + C_vec + Fb_vec + g_vec;

            return torque;
        }
```

### Experiments