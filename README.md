# Trajectory of a SCARA robot from a binary image

This is an ingeneer project made in the engeenering school Télécom Physique Strasbourg.
The objective is to use a scara robot to write somethings on unplane surfaces. The thinks which need to be writed as based on an binary image. Here the image than we use is black letters on a white screen, and for the moment we only use letters with straight lines (no courbs like in a O).
We extract trajectory from the image to communicate whith the robot

## How use the image processing code

### *Class_graph*

They are two part which you need to know how they work. The first one is the code in the file *class_graph*. In this file, there is the class *Graph*. This class allow to create trajectory from image which contain only shapes whith angles (like a black square for example).

First to use the class, you need to init it whith this form : OBJ = graph("*Path*")
Next you can use the fonction named *image2coord* to extract the motion points from the image. These points are avaible in the liste named *dim_reel_y*. You can use it whith OBJ.dim_reel_y for example.

### *Class_graph2*

This class is the improvement of the bellow funtion. It allow to create trajectory from coplexes images, and not only the images whith straight line only, like a sphere for example. The main of this function can be used like the last althought it works very differently.
The init form : OBJ = graph("*Path*")
The image processing : OBJ.ProcessingGene(pas=1 , fact_echelle = 0.1 , cadre=1, affichage=0)
 - pas : le pas d'échantillonnage de l'image en pixel (int)
 - fact_echelle : dimention réel de la hauteur selon X de notre image en m
 - cadre : Ajout d'un cadre ou non  (0 -> no // 1 -> yes)
 - affiche : plot the figure or not (0 -> no // 1 -> yes)

These points are avaible in the liste named *lst_tot*. You can use it whith OBJ.lst_tot for example.

## How move the scara robot. 

### Installation and launch

The configuration of the robot comme from the github directory of ICube Robotics in the section scara_tutorial_ros2 : https://github.com/ICube-Robotics/scara_tutorial_ros2 (1). This scara tutorial allow tho install the pacakage to simulate on Rviz and Gazebo avec use a real scara robot. 
It is necessary to install all the packages need in this tutorial (1) from branch named **scara_dxl**. Everythings is describe. 

After installing Ros2 Humble and the packages for the scara robot tutoral you can clone this github. Then go to the directory where the git is clone use this command to launch the robot on Rviz. Be sure to use the scara_dxl branch.

```bash
source ros2_ws/install/setup.bash 
ros2 launch scara_bringup scara_dxl.launch.py
```
An rviz windows with the scara robot and a speed contoller windows should be open. Indeed the robot is design to be controlled in speed and fot us it's necessary to controle the robot in position. Thus we will active the trajectory controller in the robot. 

In an other terminal and again in the folder cloned (/Projet-tutore__Create-robot-trajectory-from-a-binary-image)
```bash
source ros2_ws/install/setup.bash 
bash cmd_scara_dxl_rviz.bash
```
Now the speed controler is desactivate. 

Finaly to to make the robot draw the picture launch the main code in python. 
```bash
python3 main.py
```
Thus the image is processed and the trajectories for the plot are sent to the ros topic called ----. Then the robot listen this topic and can execute the movement. It is possible to see the topic with the command : 
```bash
source ros2_ws/install/setup.bash 
ros2 topic echo /scara_trajectory_controller/joint_trajectory
```

### Description of the process

This is how it works : 
 - This size of the image wanted is supplied and then the image processing supplied a list with the coordinate x, y and z matching with the shape present on the image. This list is named *trajectory_pts_reel* and can be called with OBJ.trajectory_pts_reel as say above. 
 - A verification step say if the size of the image can enter in the robot's workspace. 
 - The ros node is created and the trajectory messages (joint positions) are sent with a prior conversion of the Cartesian coordinates into joint coordinates.
 - When the list is finished then the ros node is deleted. 

### Description step by step

The function main is the function carried out at the start of the programme.
Fist, the features of the robot are describe with this lines : 

```python
a1 = 0.8  
a2 = 0.8      
coude = 1  
alpha_max = (85/180)*np.pi  #radian 
alpha_min = -(85/180)*np.pi #radian 
beta_max = (115/180)*np.pi   #radian 
beta_min = -(115/180)*np.pi  #radian 
```
a1 and a2 correspond to the size of each arm. In addition coude mean if we want to use the robot with elbow to the right (coude=1) or elbow to the left (coude =-1). Finaly the robot is dimentioned with maximum and minimum angles. 

The drawing process is initialized with the class graph for drawings with corner. (example letter)

```python
graph1 = graph("./images/TTT.png")
```
Or, the drawing process is initialized with the class Graph2 for drawings with rounded shapes. (example Telecom Physique Strasbourg icon)

```python
graph1 = Graph2("./images/TPS.png")
```
After, it is possible to choose the size of the image that we want to draw with the robot : the variable l mean the length and then the the height is proportional (return with *GRAPH.dim_reel_y*.
For drawings with corner : 
```python
l = 1.2
graph1.image2coord(pas=1 , fact_echelle= l)
```
For drawings with rounded shapes : 
```python
l = 0.6
graph1.ProcessingGene(pas=1 , fact_echelle= l,cadre=1, affichage=0)
```
This process is needed to have the two dimensions of the image. 

After do that we use the function *is_in_workspace* which check is all the corner of the image can be drawn by the robot. Indeed we can draw the drawing anywhere possible in the robot workspace thanks to *origin* which is the bottom left corner of the image drawn.

If the size of the image correspond to the robot correpond to the robot workspace then we can get the list of point like that : 

```python
lst = graph1.trajectory_pts_reel
```
