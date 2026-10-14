ROBOT MOVIL DIFERENCIAL CON LIDAR Y CAMARA EN ROS 2 Y GAZEBO
================================================================

Autor
-----
Felipe Vasquez Martinez

Descripcion
-----------
Este repositorio contiene el modelado y la simulacion de un robot movil de
traccion diferencial desarrollado con ROS 2 Jazzy y Gazebo Harmonic.

El proyecto incluye:

- Modelo parametrizado en URDF/Xacro.
- Chasis, dos ruedas motrices y apoyo frontal tipo caster simplificado.
- Propiedades visuales, de colision e inerciales.
- Control diferencial mediante el plugin DiffDrive de Gazebo.
- Publicacion de odometria y transformaciones TF.
- Publicacion de estados articulares de las ruedas.
- LIDAR 2D de 360 grados.
- Camara RGB frontal.
- Bridge entre Gazebo Transport y ROS 2.
- Visualizacion en RViz2.
- Visualizacion de imagen con rqt_image_view.
- Teleoperacion por teclado.
- Nodo propio de cinematica diferencial.
- Integracion de pose a partir de /joint_states.
- Comparacion entre la odometria calculada y /odom del plugin DiffDrive.
- Pista propia con curvas, estrechamiento, obstaculos, muros con colision y
  punto de spawn definido.

Entorno utilizado
-----------------

- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- Python 3
- RViz2
- Xacro
- ros_gz_sim
- ros_gz_bridge

Estructura del paquete
----------------------

robot_diferencial/
|-- config/
|-- launch/
|   |-- display.launch.py
|   `-- simulation.launch.py
|-- resource/
|   `-- robot_diferencial
|-- robot_diferencial/
|   |-- __init__.py
|   `-- cinematica_diferencial.py
|-- rviz/
|   `-- robot.rviz
|-- urdf/
|   `-- robot.urdf.xacro
|-- worlds/
|   `-- pista.sdf
|-- package.xml
|-- setup.cfg
`-- setup.py

Parametros principales del robot
--------------------------------

- Longitud del chasis: 0.45 m
- Ancho del chasis: 0.32 m
- Altura del chasis: 0.12 m
- Masa del chasis: 5.0 kg
- Radio de las ruedas: 0.08 m
- Ancho de las ruedas: 0.04 m
- Separacion entre ruedas: 0.36 m
- Masa de cada rueda: 0.5 kg
- Radio del apoyo frontal: 0.035 m
- Masa nominal total aproximada: 6.3 kg

Sensores
--------

LIDAR 2D:

- Tipo: gpu_lidar
- Cobertura: 360 grados
- Muestras: 360
- Frecuencia: 10 Hz
- Rango minimo: 0.10 m
- Rango maximo: 8.0 m
- Resolucion de rango: 0.01 m
- Topic: /scan
- Frame: lidar_link

Camara RGB:

- Resolucion: 640 x 480 pixeles
- Frecuencia: 20 Hz
- Campo visual horizontal: aproximadamente 80 grados
- Formato: R8G8B8
- Plano cercano: 0.05 m
- Plano lejano: 20.0 m
- Topic de imagen: /camera/image_raw
- Topic de informacion: /camera/camera_info
- Frame: camera_optical_link

Pista
-----

La pista es un circuito ovalado cerrado. El ovalo interior esta desplazado
0.60 m hacia la derecha respecto al ovalo exterior, lo cual forma una curva
mas estrecha y otra mas amplia.

- Dimension exterior aproximada en X: 14.8 m
- Dimension exterior aproximada en Y: 6.8 m
- Radio exterior: 3.40 m
- Radio interior: 1.65 m
- Altura de los muros: 0.60 m
- Espesor de los muros: 0.18 m
- Ancho libre aproximado de las rectas: 1.57 m
- Ancho libre minimo de la curva derecha: 0.97 m
- Punto de spawn: x=-2.60, y=-2.50, z=0.20, yaw=0 rad

La pista contiene obstaculos cilindricos en la recta inferior y obstaculos
rectangulares en la recta superior. Todos los muros y obstaculos fisicos
tienen geometria visual y geometria de colision.

Dependencias
------------

El computador debe tener instalado ROS 2 Jazzy sobre Ubuntu 24.04.

Instalar las herramientas principales:

    sudo apt update

    sudo apt install -y \
      ros-jazzy-ros-gz \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-rviz2 \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-rqt-image-view \
      ros-jazzy-image-transport \
      ros-jazzy-tf2-tools \
      python3-colcon-common-extensions \
      python3-rosdep

Si rosdep no ha sido inicializado en el computador:

    sudo rosdep init
    rosdep update

Descarga del repositorio
------------------------

Clonar el repositorio dentro de un workspace:

    mkdir -p ~/robot_diferencial_ws/src
    cd ~/robot_diferencial_ws/src
    git clone URL_DEL_REPOSITORIO robot_diferencial

Si el repositorio ya contiene la carpeta src en su raiz, puede clonarse de esta
manera:

    git clone URL_DEL_REPOSITORIO ~/robot_diferencial_ws

Antes de compilar, verificar que la ruta final sea:

    ~/robot_diferencial_ws/src/robot_diferencial

Instalacion de dependencias
---------------------------

    cd ~/robot_diferencial_ws
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths src --ignore-src -r -y

Compilacion
-----------

    cd ~/robot_diferencial_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash

Ejecucion completa
------------------

Iniciar Gazebo, el robot, el bridge, RViz y el nodo de cinematica:

    cd ~/robot_diferencial_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch robot_diferencial simulation.launch.py

Teleoperacion
-------------

En una terminal nueva:

    source /opt/ros/jazzy/setup.bash
    source ~/robot_diferencial_ws/install/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Teclas principales:

- i: avanzar
- ,: retroceder
- j: girar a la izquierda
- l: girar a la derecha
- k: detener
- x: reducir velocidad lineal
- c: reducir velocidad angular

Visualizacion de la camara
--------------------------

En otra terminal:

    source /opt/ros/jazzy/setup.bash
    source ~/robot_diferencial_ws/install/setup.bash
    ros2 run rqt_image_view rqt_image_view

Seleccionar el topic:

    /camera/image_raw

Topics principales
------------------

- /cmd_vel
  Comandos de velocidad lineal y angular.

- /odom
  Odometria publicada por el plugin DiffDrive.

- /joint_states
  Posicion y velocidad de las articulaciones de las ruedas.

- /odom_ruedas
  Odometria integrada por el nodo propio desde /joint_states.

- /error_odometria
  Vector [error_x, error_y, error_theta, error_posicion].

- /scan
  Mediciones del LIDAR.

- /camera/image_raw
  Imagen RGB.

- /camera/camera_info
  Informacion intrinseca de la camara.

- /tf y /tf_static
  Transformaciones entre frames.

- /clock
  Reloj de la simulacion.

Cinematica diferencial
----------------------

Sea:

    w = [w_D, w_I]^T

con w_D y w_I como velocidades angulares de las ruedas derecha e izquierda.
La velocidad del cuerpo se obtiene mediante:

    [v]       [ r/2    r/2 ] [w_D]
    [w]   =   [ r/L   -r/L ] [w_I]

Para r=0.08 m y L=0.36 m:

    [v]       [0.04     0.04   ] [w_D]
    [w]   =   [0.22222 -0.22222] [w_I]

La derivada de la pose se obtiene con:

    x_dot     v cos(theta)
    y_dot  =  v sin(theta)
    theta_dot w

El nodo cinematica_diferencial.py lee /joint_states, integra la pose y publica
/odom_ruedas. Luego compara esta estimacion con /odom del plugin DiffDrive.

Los errores publicados son:

    error_x = x_ruedas - x_odom
    error_y = y_ruedas - y_odom
    error_theta = normalizar(theta_ruedas - theta_odom)
    error_posicion = sqrt(error_x^2 + error_y^2)

Bridge ROS 2 - Gazebo
---------------------

ros_gz_bridge convierte mensajes entre Gazebo Transport y ROS 2.

Direcciones principales:

- /cmd_vel: ROS 2 hacia Gazebo.
- /odom: Gazebo hacia ROS 2.
- /joint_states: Gazebo hacia ROS 2.
- /scan: Gazebo hacia ROS 2.
- /camera/image_raw: Gazebo hacia ROS 2.
- /camera/camera_info: Gazebo hacia ROS 2.
- /clock: Gazebo hacia ROS 2.

Notas de portabilidad
---------------------

No es necesario compartir ni versionar las carpetas generadas:

- build/
- install/
- log/
- __pycache__/
- *.egg-info/

En un computador nuevo se deben reconstruir con rosdep y colcon.

Solucion de problemas comunes
-----------------------------

1. ROS 2 no encuentra el paquete:

    source /opt/ros/jazzy/setup.bash
    source ~/robot_diferencial_ws/install/setup.bash

2. El launch no encuentra pista.sdf, el Xacro o robot.rviz:

Revisar que setup.py instale launch, urdf, worlds, rviz y config.

3. Gazebo abre, pero el robot no aparece:

Revisar /robot_description, el nombre del mundo y el nodo ros_gz_sim create.

4. El robot no se mueve:

Revisar los nombres de left_wheel_joint y right_wheel_joint, el radio, la
separacion y el bridge de /cmd_vel.

5. Gazebo publica el sensor pero ROS 2 no lo muestra:

Comparar gz topic -l con ros2 topic list y revisar el bridge.

6. RViz muestra No transform:

Revisar robot_state_publisher, /joint_states, /tf y los frame_id de los
sensores.

Licencia
--------

Apache-2.0
