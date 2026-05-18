# uma_arm_control
This is the UMA arm control repo


## Launch 

ros2 launch uma_arm_description uma_arm_visualization.launch.py
ros2 launch uma_arm_control uma_arm_dynamics_launch.py
ros2 launch uma_arm_control gravity_compensation_launch.py
ros2 launch uma_arm_control dynamics_cancellation_launch.py


## Inverse Dynamics Control


### Gravity compensation
Method gravity_compensation() to calculate the desired torques:

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







### Linearization by inverse dynamics control



### Experiments