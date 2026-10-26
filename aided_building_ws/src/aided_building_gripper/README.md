# aided_building_gripper

Controle da Franka Hand com **a mesma interface no Gazebo e no robô real**.

| Modo | Quem fornece a interface |
| --- | --- |
| Gazebo (`use_sim:=true`) | `sim_gripper_server` (este pacote) |
| Real (`use_sim:=false`) | `franka_gripper_node` (oficial, `franka_ros2`) |

O código do time chama sempre as mesmas actions. Só o launch muda.

## Interface (namespace `/franka_gripper`)

| Nome | Tipo | Uso |
| --- | --- | --- |
| `homing` | `franka_msgs/action/Homing` | Abre totalmente (no real, também calibra) |
| `move` | `franka_msgs/action/Move` | `width` [m], `speed` [m/s] |
| `grasp` | `franka_msgs/action/Grasp` | `width` [m], `force` [N], `speed`, `epsilon.inner/outer` [m] |
| `gripper_action` | `control_msgs/action/GripperCommand` | Para MoveIt: `position` = largura/2 |
| `stop` | `std_srvs/srv/Trigger` | Para o movimento atual |
| `joint_states` | `sensor_msgs/msg/JointState` | Estado dos dedos |

Regra do `grasp` (igual ao `libfranka`): a garra fecha até encontrar o objeto.
O grasp tem sucesso se a largura medida `d` satisfaz
`width - epsilon.inner <= d <= width + epsilon.outer`.
Para objetos de tamanho variável, use um `epsilon` maior.

Exemplo de cabo de ferro de solda com ~20 mm de diâmetro:

```bash
ros2 run aided_building_gripper gripper_cli grasp 0.020 30 0.05 0.005
```

## Execução

```bash
# Simulação completa (braço + garra)
ros2 launch aided_building_bringup sim.launch.py

# Só a garra, no robô real
ros2 launch aided_building_gripper gripper.launch.py use_sim:=false robot_ip:=<IP>
```

Se o braço real for lançado com o `franka_bringup` e `load_gripper:=true`, ele já sobe
o `franka_gripper`. Nesse caso, **não** rode o launch acima (o nó ficaria duplicado).

## Teste rápido

```bash
ros2 run aided_building_gripper gripper_cli homing
ros2 run aided_building_gripper gripper_cli move 0.03
ros2 run aided_building_gripper gripper_cli grasp 0.02 20
ros2 run aided_building_gripper gripper_cli open
ros2 run aided_building_gripper gripper_cli stop
```

### Com um objeto

`object:=true` coloca um cilindro de 20 mm (50 g) sobre um pedestal fino, entre os dedos
abertos na pose inicial do braço (SAFE_ZONE). O padrão é `false`, porque o pedestal
bloquearia o braço.

```bash
ros2 launch aided_building_bringup sim.launch.py object:=true
ros2 run aided_building_gripper gripper_cli grasp 0.02 20   # success, dedos a ~0,010 m
ros2 run aided_building_gripper gripper_cli grasp 0.04 20   # failed: 20 mm fora de 40 ± 5 mm
ros2 run aided_building_gripper gripper_cli move 0.0        # failed: bloqueado pelo objeto
# Levantar 10 cm com o objeto preso:
ros2 topic pub --once /franka/joint_targets std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.6861, 0.0, -2.0279, 0.0, 1.3418, 0.7854]}"
```

Sem interface gráfica (servidor, CI): `gz_args:="-r -s empty.sdf" use_rviz:=false`.
Em Docker com `--net=host`, exporte `GZ_IP=127.0.0.1` antes do launch; sem isso o
`ros_gz_sim create` pode não encontrar o mundo (fica em "Requesting list of world names").

## Build

`franka_msgs` é necessário nos dois modos. `franka_gripper` (com `libfranka`) só é
necessário no modo real.

```bash
vcs import src < dependencies.repos
# Apenas simulação (não compila libfranka):
colcon build --symlink-install --packages-up-to aided_building_bringup
```

## Limitações da simulação

- **A força não é aplicada.** O controle de posição do Gazebo não regula a força. O campo
  `force` é validado (0 a 70 N), mas ignorado. Após o contato, os dedos são mantidos
  `grasp_squeeze` (2 mm) dentro do objeto, e a força resultante depende da física do Gazebo.
- A velocidade usa o relógio de parede. Se o Gazebo rodar mais lento que o tempo real,
  o movimento parecerá mais rápido no tempo simulado.
- Para segurar objetos no Gazebo, eles precisam ter colisão e massa coerentes. Os
  dedos já têm o atrito dos pads (`mu = 1.13`, valor do `franka_gazebo` oficial).

## Implementação (sim)

- O URDF ativa a Franka Hand (`hand=true`). Só `fr3_finger_joint1` recebe comando de
  posição. `fr3_finger_joint2` é mimic e fica no bloco `ros2_control` apenas com state
  interfaces; sem isso, o `gz_ros2_control` não o movimenta. O aviso
  `Attempting to create a mimic constraint ... does not support mimic constraints` do
  Gazebo é inofensivo: quem faz o dedo 2 seguir o 1 é o próprio `gz_ros2_control`.
- O Gazebo (DART) prende uma junta prismática parada exatamente no batente. Por isso os
  dedos começam em 0,0395 (e não 0,04) e o setpoint de abertura total fica
  `limit_margin` (1 mm de largura) abaixo de 0,08 m. `homing`/`open` continuam com
  sucesso (tolerância de 2 mm).
- O controlador é `fr3_gripper_controller` (`JointGroupPositionController`). O servidor
  gera uma rampa de setpoints na velocidade pedida e detecta travamento (contato).
- Parâmetros: `config/sim_gripper.yaml`.
