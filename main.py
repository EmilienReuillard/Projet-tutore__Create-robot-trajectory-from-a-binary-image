from class_graph import *
from class_graph2 import *
import matplotlib.pyplot as plt

import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def coord_articulaire(x,y,a1,a2,coude=1,):
    # prend en argument les coordonné x et y dand le repère de la base du robot (voir schéma)
    #retoune les angles à affecter aux articulation pour que l'effecteur ateingne le point (x,y) 
    # en fonction aussi de l'orientation du coude (1 ou -1)
    

    D=(x**2. + y**2. - a1**2. - a2**2.)/(2. * a1 *a2)
    
    if abs(D)>1:
        print("Erreur - position inateignable")
        return False
    beta = coude * acos(D)
    
    k1 = a1 + a2 * cos(beta)
    k2 = a2 * sin(beta)

    alpha = atan2(k1*y - k2*x, k2*y + k1*x)

    return [alpha, beta]

def repere_change_dxl(x,y):
    pt_decallage=[0,0]
    M01 = np.array([[0,1,0,pt_decallage[0]],[1,0,0,pt_decallage[1]], [0,0,1,0], [0,0,0,1]])
    pt_homo = np.array([[x],[y],[0],[1]])
    pt_new = np.dot(M01,pt_homo)
    pt_new = pt_new / pt_new[3][0]
    return pt_new[0][0], pt_new[1][0]

def repere_change(x,y,pt_decallage):
    M01 = np.array([[1,0,0,pt_decallage[0]],[0,1,0,pt_decallage[1]], [0,0,1,0], [0,0,0,1]])
    pt_homo = np.array([[x],[y],[0],[1]])
    pt_new = np.dot(M01,pt_homo)
    pt_new = pt_new / pt_new[3][0]
    return pt_new[0][0], pt_new[1][0]

def is_in_workspace(x,y,a1,a2,alpha_min, alpha_max, beta_min, beta_max, coude):
    marge = 0.05
    v_z = ((sqrt(sin(np.pi - beta_max))**2)*(a2**2) + (a1 - a2)**2)
    if (sqrt(x**2  + y**2) > ( a1 + a2 ) - marge):
        print("trop loin")
        return False
    elif((x**2  + y**2) < v_z + marge):
        print("trop près")
        return False
    else:
        #pour le point on calcul les angles articulaires et vérifie si ils ne dépassent pas les valeur max et min 
        val = coord_articulaire(x,y,a1,a2,coude)
        alpha, beta = float(val[0]), float(val[1])
        if not(alpha_min<=alpha<=alpha_max):
            return False
        
        elif not(beta_min<=beta<=beta_max):
            return False
        
        return True

class TrajectoryPublisher(Node):

    def __init__(self,lst_point,origin,a1,a2, coude):
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
        self.z_move = bool()
        self.new_z = float()
        self.time_z_move = 6 #en seconde 
        self.coude = coude
        self.a1 = a1
        self.a2 = a2
    
    def timer_callback(self):
        L = len(self.lst_point[0])
        
        top_position_z = 1.0
        bottom_position_z = 0.0
        msg = JointTrajectory()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.joint_names = ['joint1','joint2','joint3']
        
        msg.points = []
        point = JointTrajectoryPoint()
        
        #premier point 
        
        if (self.i == 0):
            
            x = float(self.lst_point[0][self.i])
            y = float(self.lst_point[1][self.i])
            z = float(self.lst_point[2][self.i])

            x,y = repere_change(x,y,self.origin)
            x,y = repere_change_dxl(x,y)

            print(f"x = {x} ; y = {y} ; z = {z}") 

            x = round(x,3)
            y = round(y,3)
            z = round(z,3)           

            val = coord_articulaire(x,y,a1=self.a1,a2=self.a2,coude=self.coude)

            if (val == False):
                    #position inateignalbe, pas dans l'espace de travails 
                    return

            alpha, beta = float(val[0]), float(val[1])

            #passage de la position haute à le position basse. 
            print("aller au premier point")
    
            self.z_move = True
            point.time_from_start.sec = self.time_z_move
            self.new_z = top_position_z
            
        elif (self.i < L): 
            
            
            x = float(self.lst_point[0][self.i])
            y = float(self.lst_point[1][self.i])
            z = float(self.lst_point[2][self.i])
                
            x,y = repere_change(x,y,self.origin)
            x,y = repere_change_dxl(x,y)
                
            print(f"x = {x} ; y = {y} ; z = {z}") 
                
            x = round(x,3)
            y = round(y,3)
            z = round(z,3)           
                      
            val = coord_articulaire(x,y,a1=self.a1,a2=self.a2,coude=self.coude)
                
            if (val == False):
                    #position inateignalbe, pas dans l'espace de travails 
                    return
                
            alpha, beta = float(val[0]), float(val[1])
                
            
            if (z==0.0 and self.z_before!=0.0):
                #passage de la position haute à le position basse. 
                print("passage de la position haute à le position basse")
                self.z_move = True
                self.new_z = bottom_position_z
                point.time_from_start.sec = self.time_z_move
                
            elif(z!=0.0 and self.z_before==0.0):
                #passage de la position basse à la position haute.
                print("passage de la position haute à le position basse") 
                self.z_move = True
                self.new_z = top_position_z
                point.time_from_start.sec = self.time_z_move
                
            elif(z==0.0):
                #position basse 
                print("position basse ") 
                self.z_move = False
                self.new_z=bottom_position_z
                self.timer_period = self.period
                point.time_from_start.nanosec = int(self.timer_period * 1e9)
                
            elif(z!=0.0):
                #position haute
                print("position haute")
                self.z_move = False
                self.new_z=top_position_z
                self.timer_period = self.period
                point.time_from_start.nanosec = int(self.timer_period * 1e9)
                
        elif(self.i == L):
            #fin du tracé, on met le crayon en haut
            
            x = self.x_before
            y = self.y_before
            z = top_position_z
            
            x,y = repere_change(x,y,self.origin)
            x,y = repere_change_dxl(x,y)
            
            val = coord_articulaire(x,y,a1=self.a1,a2=self.a2,coude=self.coude)
            
            if (val == False):
                #position inateignalbe, pas dans l'espace de travails 
                return
            
            alpha, beta = float(val[0]), float(val[1])
            
            #passage de la position haute à le position basse. 
            print("aller au premier point")
    
            self.z_move = True
            point.time_from_start.sec = self.time_z_move
            self.new_z = top_position_z

        else : 
            print(" Fin du tracé ")
            exit() 
        
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
        
        if self.z_move:
                time.sleep(self.time_z_move)     
        

def main(args=None):
    #paramètres du robot
    a1 = 0.8  #en m voir le ficheir URDF dans scara_tutorial_ros2/scara_description/urdf
    a2 = 0.8  #en m voir le ficheir URDF dans scara_tutorial_ros2/scara_description/urdf
    
    coude = 1
    
    alpha_max = (85/180)*np.pi  # 85 degres en radian 
    alpha_min = -(85/180)*np.pi # -85 degres en radian 
    beta_max = (95/180)*np.pi   # 95 degres en radian 
    beta_min = -(95/180)*np.pi  # - 95 degres en radian 
    
    #déclaration de l'élément graph de la classe graph
    graph1 = Graph2("TPS.png")
    l = 1#selon x
    graph1.ProcessingGene(pas=1 , fact_echelle= l,affichage=1)
    h = graph1.dim_reel_y #selon y
    print(h)
    
    #vérification que l'image rentre dans la zone de travails 
    origin = [-0.5, 1.0]
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
        #graph1.affichage()
        #vérification que l'image rentre deq
        
        
        lst = graph1.lst_tot_reel    #lst contient les coordonées xyz
        
        #initialisation du node ros
        rclpy.init(args=args)
        point_publisher = TrajectoryPublisher(lst,origin=origin,a1=a1,a2=a2, coude=coude)
        
        rclpy.spin(point_publisher)
        #publishing
        point_publisher.destroy_node()
        rclpy.shutdown()
        
    else:
        
        print("Error : l'image ne rentre pas dans l'espace de travails du robot")
        print(f"Top left : {test_t_l}; Top right : {test_t_r}; ")
        print(f"Bottom left : {test_b_l}; Bottom right : {test_b_r}; ")
        
if __name__ == '__main__':
    main()