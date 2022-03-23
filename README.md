# RMA - Projeto de Implementação 1
Disciplina Robôs Móveis Autônomos 2021/02

## Planejamento de Trajetória com A*

Grupo:
  - Daniel Amaral Brigante - 769867
  - Otavio de Paiva Pinheiro Neto - 769664

## Pacotes ROS neste repositório

Este repositório é composto por três conjuntos de pacotes:
  - smb_common -> Configurações, descrições e controlador de velocidade do robô SMB utilizado
  - smb_highlevel_controll -> Planejador A* e controlador desenvolvidos durante o projeto. Além de arquivos auxiliares, como modelos para o ambiente de simulação no Gazebo, configurações do RViz etc. 
  - tf2 -> Adicionado a este repositório para torná-lo independente do mrs_workspace usado durante a disciplina (sem ele, ocorriam problemas com o timestamp das mensagens do robot_state_publisher)

## Guia de Instalação

Este guia presume que o usuário possua uma distribuição linux Ubuntu 20.04 com os pacotes básicos do ROS já instalados no sistema, além do Gazebo, Catkin, Python etc.

Após os passos deste guia, o workspace criado deve ser capaz de funcionar de maneira independente dos workspaces usados durante a disciplina.

### Criação do Workspace 

Um workspace próprio, diferente do utilizado ao longo da disciplina, é criado para compilar e executar o projeto de forma independente. Nos comandos, é usado o nome "novo_workspace" para indicar o workspace que será criado.

```shell
source /opt/ros/noetic/setup.bash
mkdir -p ~/novo_workspace/src
cd ~/novo_workspace/
catkin_make
source devel/setup.bash
catkin init
catkin clean

```

### Dependências

É recomendado verificar se os seguintes pacotes do ROS e módulos do Python estão instalados no sistema, pois eles são dependências dos pacotes que serão compilados nos próximos passos:

```shell
sudo apt install ros-noetic-rospy ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-geometry-msgs ros-noetic-visualization-msgs ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins python3-wstool python3-catkin-tools ros-noetic-ompl ros-noetic-move-base ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-costmap-2d ros-noetic-teb-local-planner ros-noetic-robot-self-filter ros-noetic-pointcloud-to-laserscan ros-noetic-ros-numpy ros-noetic-nav-core ros-noetic-openslam-gmapping ros-noetic-tf2-sensor-msgs  ros-noetic-velodyne-description ros-noetic-joint-state-publisher-gui ros-noetic-joint-state-publisher ros-noetic-hector_gazebo_plugins ros-noetic-xacro ros-noetic-urdf ros-noetic-roslib

pip3 install numpy matplotlib scipy
```
### Build

Os pacotes que serão montados no workspace criado são provenientes de três repositórios:
  - Slam Gmapping -> Usado no mapeamento do ambiente
  - Navigation -> Usado pelo controlador para localizar o robô através do sistema AMCL
  - RMA -> Pacotes deste repositório

A montagem dos pacotes deverá ser feita seguindo os seguintes passos:

```shell
cd ~/novo_workspace/src
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/ros-planning/navigation.git
git clone https://github.com/danielb-28/RMA.git
catkin build
source ../devel/setup.bash
```

## Rodando a Simulação


### Verificar o Workspace

Ao realizar os passos da semana 1 da disciplina, o arquivo .bashrc foi configurado para definir automaticamente as variáveis de ambiente para escolher os workspaces "\~/workspace" e "\~/mrs_workspace" como atuais. Isso será feito toda vez que um novo terminal for iniciado. Sendo assim, devemos alterar o workspace para o "\~/novo_workspace" criado durante este guia antes de rodarmos a simulação sempre que um novo terminal for inicializado.

Para verificar o path onde o ROS busca pelos pacotes, podemos usar o seguinte comando. Ele deve indicar os paths dentro da pasta "~/novo_workspace".

```shell
echo $ROS_PACKAGE_PATH 
```

Caso a variável de ambiente $ROS_PACKAGE_PATH indique um workspace diferente de "\~/novo_workspace" ou um novo terminal tenha sido inicializado, podemos selecionar o workspace criado para este projeto repetindo o seguinte passo: 

```shell
cd ~/novo_workspace/
source devel/setup.bash
```
### Launch

Com o workspace corretamente selecionado, a simulação pode ser iniciada a partir do .launch presente no pacote smb_highlevel_controller:

```shell
cd ~/novo_workspace/
roslaunch src/RMA/smb_highlevel_controller/launch/smb_highlevel_controller.launch 
```

## Solução de Problemas

### Problemas com o tópico /rslidar_points

Caso ocorra algum problema com o tópico /rslidar_points, há um script para substituir um arquivo original .xacro do Velodyne por outra versão disponibilizada durante a disciplina: 

```shell
cd ~/novo_workspace/
bash src/RMA/smb_highlevel_controller/scripts/fix_rslidar.sh
```
