# RMA - Projeto de Implementação 1
Disciplina Robôs Móveis Autônomos 2022/01
Grupo:
  - Daniel Amaral Brigante - 769867
  - Otavio de Paiva Pinheiro Neto - 769664

## Criação do Workspace 

```shell
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/novo_workspace/src
$ cd ~/novo_workspace/
$ catkin_make

$ catkin init
$catkin clean

```

## Pacotes ROS

```console
$ cd ~/novo_workspace/src
$ git clone https://github.com/ros-perception/slam_gmapping.git
$ git clone https://github.com/ros-planning/navigation.git
$ sudo apt install ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins python3-wstool python3-catkin-tools ros-noetic-ompl ros-noetic-move-base ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-costmap-2d ros-noetic-teb-local-planner ros-noetic-robot-self-filter ros-noetic-pointcloud-to-laserscan ros-noetic-ros-numpy ros-noetic-nav-core ros-noetic-openslam-gmapping
```
