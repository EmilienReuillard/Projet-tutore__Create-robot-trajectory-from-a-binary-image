from class_graph import *
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


#%%

def coord_articulaire(x,y,coude=1):
    # prend en argument les coordonné x et y dand le repère de la base du robot (voir schéma)
    #retoune les angles à affecter aux articulation pour que l'effecteur ateingne le point (x,y) 
    # en fonction aussi de l'orientation du coude (1 ou -1)

    #caractéristiques du robot 
    a1 = 0.425  #en m voir le ficheir URDF dans scara_tutorial_ros2/scara_description/urdf
    a2 = 0.345  #en m voir le ficheir URDF dans scara_tutorial_ros2/scara_description/urdf

    D=(x**2. + y**2. - a1**2. - a2**2.)/(2. * a1 *a2)
    
    if abs(D)>1:
        print("Erreur - position inateignable")
        return False
    beta = coude * acos(D)
    
    k1 = a1 + a2 * cos(beta)
    k2 = a2 * sin(beta)

    alpha = atan2(k1*y - k2*x, k2*y + k1*x)

    return [alpha, beta]

def repere_change(x,y,pt_decallage):
    M01 = np.array([[1,0,0,pt_decallage[0]],[0,1,0,pt_decallage[1]], [0,0,1,0], [0,0,0,1]])
    pt_homo = np.array([[x],[y],[0],[1]])
    pt_new = np.dot(M01,pt_homo)
    pt_new = pt_new / pt_new[3][0]
    return pt_new[0][0], pt_new[1][0]

def is_in_workspace(x,y,a1,a2,alpha_min, alpha_max, beta_min, beta_max):
    marge = 0.05
    v_z = ((sqrt(sin(np.pi - beta_max))**2)*(a2**2) + (a1 - a2)**2)
    if (sqrt(x**2  + y**2) > ( a1 + a2 ) - marge):
        print("trop loin")
        return False
    elif((x**2  + y**2) < v_z + marge):
        print("trop près")
        return False
    else:
        return True

class TrajectoryPublisher(Node):

    def __init__(self,lst_point,origin):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scara_trajectory_controller/joint_trajectory', 10)
        self.period = 0.1
        self.timer_period = self.period # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.lst_point = lst_point
        self.origin = origin
        self.x_before = float()
        self.y_before = float()
        self.z_before = float()
        self.is_downhill = bool()
        print("test1")
    
    def timer_callback(self):
        
        
        L = len(self.lst_point[0])
        
        
        if (self.i < L): 
        
            msg = JointTrajectory()
        
            msg.header.stamp = self.get_clock().now().to_msg()
        
            msg.joint_names = ['joint1','joint2','joint3']
        
            msg.points = []
            point = JointTrajectoryPoint()
            
            x = float(self.lst_point[0][self.i])
            y = float(self.lst_point[1][self.i])
            z = float(self.lst_point[2][self.i])
            
            x,y = repere_change(x,y,self.origin)
            
            x = round(x,3)
            y = round(y,3)
            z = round(z,3)           
                        
            val = coord_articulaire(x,y,coude=-1)
            
            if (val == False):
                #position inateignalbe, pas dans l'espace de travails 
                return
            
            alpha, beta = float(val[0]), float(val[1])
            top_position_z = 0.2
            bottom_position_z = 0.5
            if (z==1.0 and self.z_before!=0.0):
                #passage de la position haute à le position basse. 
                self.timer_period = self.period
                self.is_downhill = False
                #tant que le z n'a pas atttein le sol alors les position x et y ne bougent pas.
                z=bottom_position_z
            elif(z!=0.0 and self.z_before==0.0):
                #passage de la position basse à la position haute. 
                z = top_position_z
                self.timer_period = 0.5
            elif(z==0.0):
                #position basse 
                z=bottom_position_z
                self.timer_period = self.period
            elif(z!=0.0):
                #position haute
                z=top_position_z
                self.timer_period = 0.5
               
            point.positions = [alpha,beta,z]
            
            print(f"x = {x} ; y = {y} ; z = {z}") 
            print(f"alpha = {alpha} ; beta = {beta} ; z = {z}")
            print("----")
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(self.timer_period * 1e9)
            msg.points.append(point)
            self.i += 1 
            self.z_before = z  
            self.publisher_.publish(msg) 
        else : 
            print(" End of communcation ")
            exit() 
             
        

def main(args=None):
    #paramètres du robot
    a1 = 0.425 
    a2 = 0.345
    
    alpha_max = np.pi
    alpha_min = -np.pi
    beta_max = np.pi
    beta_min = -np.pi
    
    #déclaration de l'élément graph de la classe graph
    graph1 = graph("TTT.png")
    l = 0.8#selon x
    graph1.image2coord(1,l)
    h = graph1.dim_reel_y #selon y
    print(h)
    
    #vérification que l'image rentre dans la zone de travails 
    origin = [-0.4, 0.3]
    pt_b_l = origin
    pt_b_r = [origin[0]+l,origin[1]]
    pt_t_r = [origin[0]+ l,origin[1] + h]
    pt_t_l=  [origin[0], origin[1] + h  ]
    
    test_b_l = is_in_workspace(pt_b_l[0], pt_b_l[1], a1,a2,alpha_min,alpha_max, beta_min, beta_max)
    test_b_r = is_in_workspace(pt_b_r[0], pt_b_r[1], a1,a2,alpha_min,alpha_max, beta_min, beta_max)
    test_t_r = is_in_workspace(pt_t_r[0], pt_t_r[1], a1,a2,alpha_min,alpha_max, beta_min, beta_max)
    test_t_l = is_in_workspace(pt_t_l[0], pt_t_l[1], a1,a2,alpha_min,alpha_max, beta_min, beta_max)
   
    if (test_b_l == True and test_t_r == True and test_t_l==True and test_b_r==True):
        print("Image OK")
        #graph1.affichage()
        #vérification que l'image rentre deq
        
        
        lst = graph1.trajectory_pts_reel    #lst contient les coordonées xyz
        
        #initialisation du node ros
        rclpy.init(args=args)
        point_publisher = TrajectoryPublisher(lst,origin=origin)
        
        rclpy.spin(point_publisher)
        #publishing
        #point_publisher.destroy_node()
        #rclpy.shutdown()
        print("test5")
    else:
        
        print("Error : l'image ne rentre pas dans l'espace de travails du robot")
        print(f"Top left : {test_t_l}; Top right : {test_t_r}; ")
        print(f"Bottom left : {test_b_l}; Bottom right : {test_b_r}; ")
        
if __name__ == '__main__':
    main()




        
        
    
    
    


