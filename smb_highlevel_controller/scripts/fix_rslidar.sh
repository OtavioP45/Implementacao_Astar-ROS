#!/bin/bash
echo "Removendo o arquivo original: /opt/ros/noetic/share/velodyne_description/urdf/VLP-16.urdf.xacro"
sudo rm -r /opt/ros/noetic/share/velodyne_description/urdf/VLP-16.urdf.xacro
echo "Copiando o novo arquivo: ./VLP-16.urdf.xacro"
sudo cp realpath "$0"/VLP-16.urdf.xacro /opt/ros/noetic/share/velodyne_description/urdf/
echo "Finalizado!"
