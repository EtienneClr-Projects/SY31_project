# SY31 Projet: Labyrinthe

## Première fois

```bash
cd /ros_ws/src/
catkin_make
```

## Pour tester le code
Et dans un nouveau terminal, 
```bash
roscore
roslaunch SY31_project robot.launch 
```

### Lancer rviz pour la visualisation avec la config
``` bash
cd ros_ws/src/SY31_project/
rviz -d configs/rviz/config.rviz
```

### Accès aux bags :
les télécharger à l'adresse https://drive.google.com/drive/folders/1LZguFK64TMTMT4ejFLmNZ_UkRLP3dJZa?usp=sharing

pour lancer un bag en boucle (loop):
``` bash
rosbag play <nom_du_bag.bag> -l
```




# Config ROS quand on est connecté au robot:
dans le .bashrc à la racine ajouter (changer ROS_IP en fonction de ce que dit "ip a")
``` bash
export TURTLEBOT3_MODEL=burger
export ROS_IP=192.168.1.155
export ROS_MASTER_URI=http://192.168.1.1:11311

source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash 
```

Si on teste sans être connecté, il ne faut pas inclure les 3 exports