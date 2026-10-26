# Aided Building - base ROS 2 + Gazebo

Base inicial do projeto **Human-Robot Collaborative Manual Soldering & Dynamic Trust Assessment** para um robô Franka. Nesta etapa o ambiente é totalmente simulado no Gazebo, a percepção é sintética e o pipeline de confiança e segurança já roda de ponta a ponta.

## Divisão do projeto

O projeto será dividido em três frentes de trabalho:

| Frente | Responsável | Escopo |
| --- | --- | --- |
| 1 | Gustavo | Planejamento e movimentação do robô |
| 2 | Lucas | Captura de dados da mão do trabalhador e da posição da placa de trabalho |
| 3 | Edilberto | Leitura da face com câmera adicional para capturar as expressões do operador |

## Escopo desta base

- Modelo Franka (FR3 por padrão) com ferramenta de solda acoplada.
- Célula de trabalho no Gazebo com bancada, tapete ESD e PCB.
- Percepção sintética publicando mãos, medo facial, estresse vocal e evento de retração.
- Fusão multimodal do índice de confiança `T(t)`.
- Máquina de estados de segurança `NORMAL -> CAUTIOUS -> PAUSED`.
- Gate de movimento com `enabled`, `speed_scale` e `retract_tool` para o futuro planejador.
- Visualização do robô e das mãos no RViz.

Ainda não faz parte desta base: MoveIt 2, planejamento adaptativo real, percepção por câmera/áudio, aquisição de dados experimentais e controle em hardware.

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `franka_ros2` (branch `jazzy`) e `franka_description`
- `gz_ros2_control`, `ros_gz_sim` e `ros_gz_bridge`

O ambiente recomendado é o Docker oficial do `franka_ros2`. Consulte a documentação oficial da Franka para a instalação completa, pois a compilação do `franka_ros2` depende de `libfranka` e de versões compatíveis do `franka_description`.

## Instalação

```bash
cd aided_building_ws

vcs import src < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Caso os pacotes da Franka já estejam instalados como binários no seu ambiente, remova as entradas correspondentes de `dependencies.repos` ou ajuste o build para evitar duplicidade.

## Execução

```bash
source install/setup.bash
ros2 launch aided_building_bringup sim.launch.py scenario:=1
```

Opções úteis:

```bash
ros2 launch aided_building_bringup sim.launch.py scenario:=2
ros2 launch aided_building_bringup sim.launch.py scenario:=3
ros2 launch aided_building_bringup sim.launch.py use_rviz:=false
```

A configuração de controladores desta base usa os nomes de juntas do FR3. Para usar FP3 ou FER, é necessário criar um YAML de controladores correspondente antes de alterar `robot_type`.

O cenário controla as posições sintéticas das mãos:

| Cenário | Descrição |
| --- | --- |
| 1 | Duas mãos livres segurando a PCB |
| 2 | Uma mão na borda; outra mão apoiada |
| 3 | PCB em suporte com as mãos estabilizando |

## Teste do pipeline

Visualizar confiança e segurança:

```bash
ros2 topic echo /trust_state
ros2 topic echo /safety_state
ros2 topic echo /motion/enabled
ros2 topic hz /human_state
```

Disparar uma retração sintética:

```bash
ros2 service call /synthetic_perception/trigger_retraction std_srvs/srv/Trigger "{}"
```

Após o evento, o índice de confiança deve cair, a segurança deve entrar em `CAUTIOUS` e o gate deve reduzir a velocidade para `0.2`. Em proximidade crítica ou confiança abaixo do limite de pausa, o estado vai para `PAUSED` e o movimento é desabilitado.

## Estrutura

```text
aided_building_ws/
├── dependencies.repos
├── docs/
│   └── architecture.md
└── src/
    ├── aided_building_interfaces/   contratos de mensagens
    ├── aided_building_description/  Franka + ferramenta de solda
    ├── aided_building_perception/   percepção sintética
    ├── aided_building_trust/        fusão multimodal da confiança
    ├── aided_building_safety/       FSM de segurança
    ├── aided_building_motion/       interface de arbitragem de movimento
    └── aided_building_bringup/      launch, mundo Gazebo e RViz
```

## Tópicos principais

| Tópico | Tipo | Publicador |
| --- | --- | --- |
| `/human_state` | `aided_building_interfaces/msg/HumanState` | percepção |
| `/trust_state` | `aided_building_interfaces/msg/TrustState` | fusão de confiança |
| `/safety_state` | `aided_building_interfaces/msg/SafetyState` | supervisor de segurança |
| `/motion/enabled` | `std_msgs/msg/Bool` | gate de movimento |
| `/motion/speed_scale` | `std_msgs/msg/Float32` | gate de movimento |
| `/motion/retract_tool` | `std_msgs/msg/Bool` | gate de movimento |
| `/human_state_markers` | `visualization_msgs/msg/MarkerArray` | percepção |

## Próximos passos

1. Integrar MoveIt 2 com objetos de colisão dinâmicos para as mãos e a distância `D_safe(t)`.
2. Substituir a percepção sintética por nós reais mantendo o mesmo contrato `HumanState`.
3. Implementar planejamento adaptativo e retração vertical de 50 mm em `CAUTIOUS`.
4. Adicionar logging sincronizado, métricas experimentais e gerenciamento dos três cenários.
5. Integrar E-stop físico, limitação de força e validação conforme ISO/TS 15066 antes de qualquer teste com hardware.

## Garra (Franka Hand)

O pacote `aided_building_gripper` expõe `/franka_gripper/{homing,move,grasp,gripper_action,stop}`
com a mesma interface no Gazebo (`sim_gripper_server`) e no robô real (`franka_gripper` oficial).
O `sim.launch.py` já inicia a garra simulada (`start_gripper:=false` para desativar).
Detalhes e limitações em `aided_building_ws/src/aided_building_gripper/README.md`.

## Segurança

Este repositório é uma base de simulação. Ele não substitui análise de risco, E-stop físico, limitação de força ou validação normativa. Nenhum teste com o robô real deve ser executado sem os dispositivos de segurança e a autorização correspondentes.
