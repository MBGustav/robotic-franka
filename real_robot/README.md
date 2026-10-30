# real_robot: Panda real con frankapy (fase 1)

El Panda del laboratorio es un **FER (Franka Emika Panda), no un FR3**, y su versión de
libfranka no es compatible con `franka_ros2`. Por eso esta parte no usa el workspace ROS 2
(`aided_building_ws/`): corre con **frankapy (ROS 1 Noetic)** en el Docker del laboratorio
`jgarcia10/u2is-franka-ros1`, y se comunica por UDP con un supervisor en el host.

```
[host]  supervisor/supervisor_teclado.py  --UDP 5005 (comando)-->  [Docker frankapy]  robot/controlador_ciclo.py
                                           <--UDP 5006 (estado)--
```

Ya funciona en el robot real: ciclo entre poses grabadas y rutina de puntos sobre la placa,
con los estados NORMAL / CAUTIOUS / PAUSED / RETRACT. Más adelante el supervisor de teclado
se sustituye por la máquina de estados alimentada por la confianza `T(t)`, y en la fase 2 por
nodos ROS 2 de este workspace.

## Contenido

```
robot/                    se ejecuta en el contenedor con frankapy
  controlador_ciclo.py      ejecuta el ciclo (--ciclos N) o la rutina sobre la placa (--placa)
  plan_placa.py             FK/IK del Panda en Python puro y plan de la rejilla (no mueve nada)
  grabar_poses.py           graba poses del ciclo o esquinas de la placa guiando el brazo a mano
  prueba_comunicacion.py    diagnóstico de communication_constraints_violation
  ejemplos/move_robot.py    ejemplo de frankapy
supervisor/               se ejecuta en el host
  supervisor_teclado.py     teclas -> estado; muestra el estado del robot y guarda un CSV
datos/                    poses.json (rad), poses_info.json (posición de la punta y esquinas
                          de la placa), mesa.json (ArUco de la mesa, marco de la base)
logs/                     CSV de cada sesión (fuera de git)
```

## Uso

**Primera vez con el robot del laboratorio (red, Desk, Docker, stack): ver
[PUESTA_EN_MARCHA.md](PUESTA_EN_MARCHA.md).**

Requisitos: stack de frankapy arrancado en el contenedor (`roscore`, `franka_interface`,
`franka_ros_interface`, `franka_gripper`), FCI activa en Desk y el contenedor lanzado con
`--net=host` (el puente usa `127.0.0.1`).

```bash
# host
cd real_robot/supervisor && python3 supervisor_teclado.py --v-normal 0.6 --v-cautious 0.3

# contenedor
cd real_robot/robot && python3 controlador_ciclo.py --placa --escala-max 0.6
#   o --ciclos 1 para el ciclo entre poses grabadas
```

Pulsa Enter en el controlador y `n` en el supervisor:

| Tecla | Estado | Qué hace el robot |
|---|---|---|
| `n` | NORMAL | velocidad `--v-normal` |
| `c` | CAUTIOUS | velocidad `--v-cautious` (desde el siguiente segmento) |
| `p` | PAUSED | `stop_skill()` y se queda quieto; `n`/`c` reanuda el segmento |
| `r` | RETRACT | para, sube la herramienta 5 cm en vertical, despacio, y espera |
| `q` | (salir) | envía PAUSED y cierra |

Seguridad: si el controlador no recibe un comando válido en 0.5 s, se para (nunca sigue).
`--escala-max` del controlador recorta la velocidad que pida el supervisor (por defecto 0.3,
para las primeras pruebas); tiene que ser al menos `--v-normal` o los modos se igualan.

Revisar el plan de la placa sin mover nada: `python3 robot/plan_placa.py`.
Probar sin robot: `python3 robot/controlador_ciclo.py --simulado --placa` (brazo falso).

Las poses y las esquinas de `datos/` son de nuestra mesa: si se mueve la placa o la mesa,
hay que volver a grabarlas con `robot/grabar_poses.py` (`--esquinas` para la placa).

## Problemas conocidos

- `communication_constraints_violation` / el robot no se mueve / modo guía rígido: casi
  siempre es la red. El enlace con la caja de control tiene que ser de 1000 Mb/s y con ping
  estable por debajo de 1 ms; con la tarjeta Realtek interna de nuestra laptop (100 Mb/s,
  picos >1 ms) fallaba todo, con un adaptador USB-Ethernet gigabit funciona.
  Diagnóstico: `python3 robot/prueba_comunicacion.py 1`.
- `stop_skill()` frena en seco (se nota una vibración); un frenado suave requiere el modo
  dinámico de frankapy (`dynamic=True`), aún no implementado.
