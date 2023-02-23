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

#### Detail of the *Class_graph2* process

1 - The Init :

The image is saved, the canny image is created and the blurred image too. All this elements are in an object defined, for the rest, it will be called OBJ.

2 - find_ensembles : 

This function used the label function. It allow to enumerate separate parts in the image. All this parts are stocked in the variable *self.props*.
With this list, it is possible make a processing only on one part of the image.

3 - createAllEnsembles :

For each region of the image, the trajectory is processessed. It return X,Y,Z coord.
To process this trajectory with *ProcessingGene*, this is what is done in a nutshell :

mapping_process :
 - The coords of all the white pixels are memorized in a list (rech_pt_cadrillage)
 - The detected points needs to to be replace in the right place. Then a research of the closesest point is made. And point after point, we found the next one whith the minimum. When a point is replaced, it cannot be chosen to be the next one. 

createAllEnsembles :
The previous function is used on each Ensemble detected in the image.

connectAllEnsemble : 
This function connect all the trajectory points maded by the previous function. All the trajetory are syntetised in an only one and intermediate way were created to link the differents Ensembles.

RotationRep :
Rotate the coords and place the origin of the axis in the down left corner.

Other :
 - It is possible to add one frame on the image (cadre = 0/1)
 - Show the result (affichage = 0/1)

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
As say above, this list come from the image processing and contains Cartesian coordinate information. And now it is interresting to send them to the robot. A Ros node have to be create and a the message have to be send in a topic. 

This 3 lines allow to init the node, then create it and after execute the timer callback link to the node. 
```python
rclpy.init(args=args)
point_publisher = TrajectoryPublisher(lst,origin=origin,a1=a1,a2=a2, coude=coude)       
rclpy.spin(point_publisher)
```
Now it is interresting to see what is going on in the class TrajectoryPublisher.
In the __init__ function, the publisher is created and correspond to a message such as **JointTrajectory** and published to the topic *'/scara_trajectory_controller/joint_trajectory'*. This type of message is decribed here : [JointTrajectory.](http://docs.ros.org/en/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html)
A few variables are defined as the frequency of sending the message : 
```python 
self.period = 0.05
```
This variable is important and allow to increase/decrease the speed of the robot as described after. 
Then according to this frequency the function *timer_callback* will be executed : 
```python
self.timer = self.create_timer(self.timer_period, self.timer_callback)
```
In this function the message is initialised : 
```python
msg = JointTrajectory()
```
For us 3 joint are defined :
```python
msg.joint_names = ['joint1','joint2','joint3']
```
And the list for the point is also initalised
```python
msg.points = []
point = JointTrajectoryPoint()
```
Thus at this stage we do not initialise any more than *msg* because we only want to transmit in the message the joint coordinates as well as the time between the initialized position before the message and the desired position and not the forces or accelerations to be applied. The robot controller takes care of this.

The end of this function is divised in 3 similar parts. The first part concerns the first item in the list, the second part concerns all the other items except the last one and the last part concerns the last item. 

For the first point the robot lift the pencil to the first point (be carefull because the top poisition for the Rviz siluation and for the real robot is not the same). And for the end the robot lift the pencil when the drawing is finished. 

Between this two parts (*self.i<L) the message is send accordig to the list of point. When the z is equal to 0 then the pencil should be on the paper. An other part important is he transition between the top position and the bottom position or the reverse. Indeed in this situation the time (**point.time_from_start**) between this two point is increased (equal to *self.time_z_move*) unlike the plotting moments where this time is equal to the period of sending the message. Thus to respect *self.time_z_move* a wait period should be done. (define by *self.z_move*) 

For each point : 
 - The cartesian coordinate is collected: 
 ```python
x = float(self.lst_point[0][self.i])
y = float(self.lst_point[1][self.i])
z = float(self.lst_point[2][self.i])
 ```
 - the points change reference points from the image reference point to the robot reference point according to the *origin*: 
 ```python 
 x,y = repere_change(x,y,self.origin)
 ```
 - And then the Cartesian coordinates are converted into joint coordinates using the inverse geometric model of the robot: 
 ```python
val = coord_articulaire(x,y,a1=self.a1,a2=self.a2,coude=self.coude)             
alpha, beta = float(val[0]), float(val[1])
 ```
 - Finaly the z is adjusted and the message is published: 
 ```python
 point.positions = [self.new_z,alpha,beta]
 msg.points.append(point)
 self.publisher_.publish(msg)
 ```
To conclude this message hold a set of joint coordinates corresponding to a unique point. A new message is therefore published on the topic for each point created by the image processing. We could have published all the trajectories at once with a message including all sets of joint coordinates but this did not work. 
