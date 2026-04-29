## PARCIAL 1

Las carpetas deben estar dentro del workspace de ROS 2 en la capeta:

```
/<tu_ws>/src
```

## Visualización

Para solamente visualizar el robot y moverlo manualmente:

```bash
ros2 launch robot_description display.launch.py
```

## Cinemática Inversa

### 1. En una primera terminal:

```bash
ros2 launch robot_description view_robot.launch.py
```

### 2. En una segunda terminal:

```bash
ros2 topic pub /target_position geometry_msgs/msg/Point \
"{x: 1.5, y: 1.0, z: 0.8}" --once
```

* `x`, `y`, `z`: coordenadas objetivo del efector final en el espacio cartesiano.

---

## Observaciones
Por alguna razón que no comprendo, no puedo visualiar el modelo del robot. Intenté varias veces, pero solo poder visualziarlo a travez del modificador TF
