# uma_arm_control
This is the UMA arm control repo


## Launch the Dynamic model
ros2 launch uma_arm_control uma_arm_dynamics_launch.py

## Implementación del modelo dinámico

### Calcular la aceleración

La dinámica de un manipulador robótico de cadena cinemática abierta está dada por:


$$ \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{F}_b\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{ext} $$

![alt text](image-2.png)

donde:

![alt text](image-3.png)

En nuestro caso, la aceleración debida a los torques aplicados está dada por:

![alt text](image-4.png)

Para calcular las aceleraciones de las articulaciones, primero necesitamos calcular las matrices. Pueden calcularse aplicando las formulaciones de Lagrange o Newton-Euler. En nuestro caso, las matrices se definen por:

![alt text](image-5.png)

También necesitaremos calcular el jacobiano para incluir las llaves externas aplicadas en el EE en nuestro modelo:

![alt text](image-6.png)


Entonces, podemos calcular τ_ext como:

![alt text](image-7.png)




### Integrar posición y velocidad

Como estamos implementando un sistema discreto:

![alt text](image-8.png)

podemos obtener las velocidades y posición conjuntas mediante integración discreta a lo largo del tiempo como:

![alt text](image-9.png)

## Lanzar el nodo del simulador dinámico



![alt text](image.png)

![alt text](image-1.png)



## Representación gráfica