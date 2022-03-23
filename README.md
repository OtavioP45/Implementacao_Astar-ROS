# RMA - Projeto de Implementação 1
Disciplina Robôs Móveis Autônomos 2021/02

## Planejamento de Trajetória com A*

Grupo:
  - Daniel Amaral Brigante - 769867
  - Otavio de Paiva Pinheiro Neto - 769664

## Guia de Instalação

Este guia presume que o usuário possua uma distribuição linux Ubuntu 20.04 com os pacotes básicos do ROS já instalados no sistema, além do Gazebo, Catkin, Python etc.

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

### Pacotes ROS

É recomendado verificar se os seguintes pacotes do ROS e módulos do Python estão instalados no sistema, pois eles são dependências dos pacotes que serão compilados nos próximos passos:

```shell
sudo apt install ros-noetic-rospy ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-geometry-msgs ros-noetic-visualization-msgs ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins python3-wstool python3-catkin-tools ros-noetic-ompl ros-noetic-move-base ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-costmap-2d ros-noetic-teb-local-planner ros-noetic-robot-self-filter ros-noetic-pointcloud-to-laserscan ros-noetic-ros-numpy ros-noetic-nav-core ros-noetic-openslam-gmapping ros-noetic-tf2-sensor-msgs  ros-noetic-velodyne-description ros-noetic-joint-state-publisher-gui ros-noetic-joint-state-publisher ros-noetic-hector_gazebo_plugins ros-noetic-xacro ros-noetic-urdf ros-noetic-roslib

pip3 install numpy matplotlib scipy
```


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

```shell
echo $ROS_PACKAGE_PATH 
```

```shell
cd ~/novo_workspace/
source devel/setup.bash
```

### Launch
```shell
cd ~/novo_workspace/
roslaunch src/RMA/smb_highlevel_controller/launch/smb_highlevel_controller.launch 
```

## Solução de Problemas

### Problemas com o tópico /rslidar_points

```shell
cd ~/novo_workspace/
bash src/RMA/smb_highlevel_controller/scripts/fix_rslidar.sh
```
