# uma_arm_control
This is the UMA arm control repo


## Launch the Dynamic model
ros2 launch uma_arm_control uma_arm_dynamics_launch.py

## Implementación del modelo dinámico

### Calcular la aceleración

La dinámica de un manipulador robótico de cadena cinemática abierta está dada por:


$$ \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{F}_b\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{ext} $$



donde:


* $\mathbf{P} \in \mathbb{R}^{n \times 1}$ es el vector de las posiciones articulares (). `joint_positions_`
* $\dot{\mathbf{q}} \in \mathbb{R}^{n \times 1}$ es el vector de velocidades conjuntas (). `joint_velocities_`
* $\ddot{\mathbf{q}} \in \mathbb{R}^{n \times 1}$ es el vector de aceleraciones de la articulación (). `joint_accelerations_`
* $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{n \times n}$ es la matriz de inercia.
* $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{n \times n}$ es la matriz de Coriolis y fuerzas centrífugas.
* $\mathbf{F}_b \in \mathbb{R}^{n \times n}$ es la matriz de fricción viscosa.
* $\mathbf{g} \in \mathbb{R}^{n \times 1}$ es el vector de gravedad.
* $\boldsymbol{\tau} \in \mathbb{R}^{n \times 1}$ es el vector de los torques articulares comandados (). `joint_torques_`
* $\boldsymbol{\tau}_{ext} \in \mathbb{R}^{n \times 1}$ es el vector de los torques de la articulación debidos a fuerzas externas.


![alt text](image-3.png)

En nuestro caso, la aceleración debida a los torques aplicados está dada por:

$$ \ddot{\mathbf{q}} = \mathbf{M}^{-1}(\mathbf{q})[ \boldsymbol{\tau} + \boldsymbol{\tau}_{ext} - \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} - \mathbf{F}_b\dot{\mathbf{q}} - \mathbf{g}(\mathbf{q}) ] $$

![alt text](image-4.png)

Para calcular las aceleraciones de las articulaciones, primero necesitamos calcular las matrices. Pueden calcularse aplicando las formulaciones de Lagrange o Newton-Euler. En nuestro caso, las matrices se definen por:

![alt text](image-5.png)

También necesitaremos calcular el jacobiano para incluir las llaves externas aplicadas en el EE en nuestro modelo:

$$ \mathbf{J}(\mathbf{q}) = \begin{bmatrix} -l_1 \cdot \sin(q_1) - l_2 \cdot \sin(q_1 + q_2) & -l_2 \cdot \sin(q_1 + q_2) \\ l_1 \cdot \cos(q_1) + l_2 \cdot \cos(q_1 + q_2) & l_2 \cdot \cos(q_1 + q_2) \end{bmatrix} $$





$$ 
\mathbf{J}(\mathbf{q}) = 
\begin{bmatrix} 
-l_1 \cdot \sin(q_1) - l_2 \cdot \sin(q_1 + q_2) & -l_2 \cdot \sin(q_1 + q_2) \\ 
l_1 \cdot \cos(q_1) + l_2 \cdot \cos(q_1 + q_2) & l_2 \cdot \cos(q_1 + q_2) 
\end{bmatrix} 
$$


![alt text](image-6.png)


Entonces, podemos calcular τ_ext como:

$$ \boldsymbol{\tau}_{ext} = \mathbf{J}(\mathbf{q})^T \cdot \mathbf{F}_{ext} $$

![alt text](image-7.png)




### Integrar posición y velocidad

Como estamos implementando un sistema discreto:

$$ \ddot{\mathbf{q}}_{k+1} = \mathbf{M}^{-1}(\mathbf{q}_k)[ \boldsymbol{\tau}_k + \boldsymbol{\tau}_{ext_k} - \mathbf{C}(\mathbf{q}_k, \dot{\mathbf{q}}_k)\dot{\mathbf{q}}_k - \mathbf{F}_b\dot{\mathbf{q}}_k - \mathbf{g}(\mathbf{q}_k) ] $$

![alt text](image-8.png)

podemos obtener las velocidades y posición conjuntas mediante integración discreta a lo largo del tiempo como:


$$\dot{\mathbf{q}} = \int \ddot{\mathbf{q}} \, dt \implies \dot{\mathbf{q}}_{k+1} = \dot{\mathbf{q}}_k + \ddot{\mathbf{q}}_{k+1} \cdot \Delta t$$

$$ \mathbf{q} = \int \dot{\mathbf{q}} Dt \implies \mathbf{q}_{k+1} = \mathbf{q}_k + \dot{\mathbf{q}}_{k+1} \cdot \Delta t $$


![alt text](image-9.png)

## Lanzar el nodo del simulador dinámico



![alt text](image.png)

![alt text](image-1.png)



## Representación gráfica