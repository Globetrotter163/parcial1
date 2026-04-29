Estas carpetas deberian estar dentro del ws, dentro de la capera /src

Para solamente visualizar el robot y controlarlo manualmente, correr

ros2 launch robot_description display.launch.py


Para poder controlarlo a travez de sus ecuaciones de cinematica inversa:

En una terminal correr:

ros2 launch robot_description view_robot.launch.py

En otra terminal enviar los comandos:

ros2 topic pub /target_position geometry_msgs/msg/Point \
  "{x: 1.5, y: 1.0, z: 0.8}" --rate 1

donde x, y & z son las cordenadas donde queremos mandar al robot 
