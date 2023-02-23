from class_graph import *
from class_graph2 import *
import matplotlib.pyplot as plt

import time
import rclpy
from rclpy.node import Node

#messages for the topic
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def coord_articulaire(x,y,a1,a2,coude=1,):
    # takes as argument the x and y coordinates in the robot base frame (see diagram)
    # sets the angles to be assigned to the joints so that the end effector reaches the point (x,y) 
    # depending also on the orientation of the elbow (1 or -1)
    
    D=(x**2. + y**2. - a1**2. - a2**2.)/(2. * a1 *a2)
    
    if abs(D)>1:
        print("Erreur - position inateignable")
        return False
    #inversion of the geometric model 
    beta = coude * acos(D)
    
    k1 = a1 + a2 * cos(beta)
    k2 = a2 * sin(beta)

    alpha = atan2(k1*y - k2*x, k2*y + k1*x)

    return [alpha, beta]

def repere_change_dxl(x,y):
    #point mark change only for scara dynamixel robot 
    pt_decallage=[0,0]
    #definition of the mark point change matrix
    M01 = np.array([[0,1,0,pt_decallage[0]],[1,0,0,pt_decallage[1]], [0,0,1,0], [0,0,0,1]])
    pt_homo = np.array([[x],[y],[0],[1]])
    pt_new = np.dot(M01,pt_homo)
    #adjustment to remain in homogeneous coordinates 
    pt_new = pt_new / pt_new[3][0]
    return pt_new[0][0], pt_new[1][0]

def repere_change(x,y,pt_decallage):
    #definition of the mark point change matrix
    M01 = np.array([[1,0,0,pt_decallage[0]],[0,1,0,pt_decallage[1]], [0,0,1,0], [0,0,0,1]])
    pt_homo = np.array([[x],[y],[0],[1]])
    pt_new = np.dot(M01,pt_homo)
    #adjustment to remain in homogeneous coordinates 
    pt_new = pt_new / pt_new[3][0]
    return pt_new[0][0], pt_new[1][0]

def is_in_workspace(x,y,a1,a2,alpha_min, alpha_max, beta_min, beta_max, coude):
    marge = 0.05
    #we check if the point is too close or too far from the origin
    v_z = ((sqrt(sin(np.pi - beta_max))**2)*(a2**2) + (a1 - a2)**2)
    if (sqrt(x**2  + y**2) > ( a1 + a2 ) - marge):
        #the point is too far from the workspace, untouchable position
        return False
    elif((x**2  + y**2) < v_z + marge):
        #the point is too close from the origin, untouchable position
        return False
    else:
        # the joint angles are calculated and checked to see if they do not exceed the max and min values 
        val = coord_articulaire(x,y,a1,a2,coude)
        alpha, beta = float(val[0]), float(val[1])
        if not(alpha_min<=alpha<=alpha_max):
            #alpha not correspond 
            print(alpha*180/np.pi)
            return False
        
        elif not(beta_min<=beta<=beta_max):
            #beta not correspond 
            print(beta*180/np.pi)
            return False
        
        return True

class TrajectoryPublisher(Node):

    def __init__(self,lst_point,origin,a1,a2, coude):
        super().__init__('trajectory_publisher')
        #creation of the message for the topic '/scara_trajectory_controller/joint_trajectory'
        self.publisher_ = self.create_publisher(JointTrajectory, '/scara_trajectory_controller/joint_trajectory', 10)
        #period between to points published
        self.period = 0.05
        self.timer_period = self.period # seconds
        #creation of the timer_callback for each period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        # ---------------------------------------------------
        # definition of variable for the rest of the programm
        self.lst_point = lst_point
        self.origin = origin
        #to keep the point just before  
        self.x_before = float()
        self.y_before = float()
        self.z_before = float()
        #when z move 
        self.z_move = bool() #if z move then true else false
        self.new_z = float()
        #time to lift the pencil
        self.time_z_move = 6 #en seconde 
        self.coude = coude
        self.a1 = a1
        self.a2 = a2
    
    def timer_callback(self):
        L = len(self.lst_point[0])
        # poisitions of the pencil. Can be change
        top_position_z = 0.02
        bottom_position_z = 0.0
        
        #Initialisation of the message
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint1','joint2','joint3']
        msg.points = []
        point = JointTrajectoryPoint()
        
        #First point in the list 
        if (self.i == 0):
            #The goal is to put the pencil above the firt point. 
            
            #Collect the informations (cartesian point) from the list
            x = float(self.lst_point[0][self.i])
            y = float(self.lst_point[1][self.i])
            z = float(self.lst_point[2][self.i])
            
            #change the point mark
            x,y = repere_change(x,y,self.origin)
            x,y = repere_change_dxl(x,y)

            print(f"x = {x} ; y = {y} ; z = {z}") 

            x = round(x,3)
            y = round(y,3)
            z = round(z,3)
            
            #get the angular values according to the point x,y
            val = coord_articulaire(x,y,a1=self.a1,a2=self.a2,coude=self.coude)

            if (val == False):
                    #unaligned position, not in the workspace 
                    return

            alpha, beta = float(val[0]), float(val[1])
    
            self.z_move = True
        
            #time to got to the position alpha, beta
            point.time_from_start.sec = self.time_z_move
            
            #the first point stay in the top pencil position 
            self.new_z = top_position_z
            
        elif (self.i < L): 
            #part for the drawing
            
            #Collect the informations (cartesian point) from the list
            x = float(self.lst_point[0][self.i])
            y = float(self.lst_point[1][self.i])
            z = float(self.lst_point[2][self.i])
            
            #change the point mark
            x,y = repere_change(x,y,self.origin)
            x,y = repere_change_dxl(x,y)
                
            print(f"x = {x} ; y = {y} ; z = {z}") 
                
            x = round(x,3)
            y = round(y,3)
            z = round(z,3)           
            
            #get the angular values according to the point x,y
            val = coord_articulaire(x,y,a1=self.a1,a2=self.a2,coude=self.coude)
                
            if (val == False):
                    #unaligned position, not in the workspace 
                    return
                
            alpha, beta = float(val[0]), float(val[1])
            
            #depending on whether the pencil is raised or not, we assign self.new_z and the time to reach it 
            
            if (z==0.0 and self.z_before!=0.0):
                #Switching from high to low position. 
                print("Switching from high to low position")
                self.z_move = True
                self.new_z = bottom_position_z
                point.time_from_start.sec = self.time_z_move
                
            elif(z!=0.0 and self.z_before==0.0):
                #Switching from low to high position.
                print("Switching from low to high position") 
                self.z_move = True
                self.new_z = top_position_z
                point.time_from_start.sec = self.time_z_move
                
            elif(z==0.0):
                #low position
                print("low position") 
                self.z_move = False
                self.new_z=bottom_position_z
                self.timer_period = self.period
                point.time_from_start.nanosec = int(self.timer_period * 1e9)
                
            elif(z!=0.0):
                #high position
                print("high position")
                self.z_move = False
                self.new_z=top_position_z
                self.timer_period = self.period
                point.time_from_start.nanosec = int(self.timer_period * 1e9)
                
        elif(self.i == L):
            #end of the drawing, pu the pencil to the top position
            
            #Collect the informations (cartesian point) from the last point
            x = self.x_before
            y = self.y_before
            z = top_position_z
            
            #change the point mark
            x,y = repere_change(x,y,self.origin)
            x,y = repere_change_dxl(x,y)
            
            #get the angular values according to the point x,y
            val = coord_articulaire(x,y,a1=self.a1,a2=self.a2,coude=self.coude)
            
            if (val == False):
                #position inateignalbe, pas dans l'espace de travails 
                return
            
            alpha, beta = float(val[0]), float(val[1])
    
            self.z_move = True
            point.time_from_start.sec = self.time_z_move
            self.new_z = top_position_z

        else : 
            print(" End of drawings ")
            exit() 
        
        #we complete the message to be sent 
        point.positions = [self.new_z,alpha,beta]
            
        print(f"x = {x} ; y = {y} ; z = {self.new_z}") 
        print(f"alpha = {alpha} ; beta = {beta} ; z = {self.new_z}")
        print("----")
        
        msg.points.append(point)
        self.publisher_.publish(msg) 
        
        
        self.y_before = y
        self.x_before = x
        self.z_before = z  
        
        self.i += 1     
        
        #time to wait if the z move
        if self.z_move:
                time.sleep(self.time_z_move)     
        

def main(args=None):
    #robot features
    a1 = 0.8  #en meter see the URDF file in scara_tutorial_ros2/scara_description/urdf 
    a2 = 0.8  #en meter see the URDF file in scara_tutorial_ros2/scara_description/urdf 
    
    coude = 1
    
    alpha_max = (85/180)*np.pi  # 85 degrees in radian 
    alpha_min = -(85/180)*np.pi # -85 degrees in radian 
    beta_max = (115/180)*np.pi   # 115 degrees in radian 
    beta_min = -(115/180)*np.pi  # -115 degrees in radian 
    
    #Part to be used to form designs with corners (e.g. letters)
    # ---------------------------------------------
    l = 1.2#selon x
    graph1 = graph("./images/TTT.png")
    graph1.image2coord(pas=1 , fact_echelle= l)
    origin = [-0.6, 0.9]
    h=graph1.dim_reel_y
    #---------------------------------------------
    
    #Part to be used to form drawings with rounded shapes (example TPS icon) 
    #---------------------------------------------    
    """
    l = 0.6
    graph1 = Graph2("./images/TPS.png")
    graph1.ProcessingGene(pas=1 , fact_echelle= l,cadre=1, affichage=0)
    origin = [-0.3, 0.9]
    h=graph1.dim_reel_y
    """
    #---------------------------------------------

    #check that the image fits into the working area (4 corners) 
    pt_b_l = origin
    pt_b_r = [origin[0]+l,origin[1]]
    pt_t_r = [origin[0]+ l,origin[1] + h]
    pt_t_l=  [origin[0], origin[1] + h  ]
    
    test_b_l = is_in_workspace(pt_b_l[0], pt_b_l[1], a1,a2,alpha_min,alpha_max, beta_min, beta_max,coude)
    test_b_r = is_in_workspace(pt_b_r[0], pt_b_r[1], a1,a2,alpha_min,alpha_max, beta_min, beta_max,coude)
    test_t_r = is_in_workspace(pt_t_r[0], pt_t_r[1], a1,a2,alpha_min,alpha_max, beta_min, beta_max,coude)
    test_t_l = is_in_workspace(pt_t_l[0], pt_t_l[1], a1,a2,alpha_min,alpha_max, beta_min, beta_max,coude)
   
    if (test_b_l == True and test_t_r == True and test_t_l==True and test_b_r==True):
        print("Image OK")
        
        lst = graph1.trajectory_pts_reel #lst contains the xyz coordinatesfor the drawing 
        #Ros Node initialisation and launch
        rclpy.init(args=args)
        point_publisher = TrajectoryPublisher(lst,origin=origin,a1=a1,a2=a2, coude=coude)
        rclpy.spin(point_publisher)
        point_publisher.destroy_node()
        rclpy.shutdown()
        
    else:
        
        print("Error : l'image ne rentre pas dans l'espace de travails du robot")
        print(f"Top left : {test_t_l}; Top right : {test_t_r}; ")
        print(f"Bottom left : {test_b_l}; Bottom right : {test_b_r}; ")
        
if __name__ == '__main__':
    main()
