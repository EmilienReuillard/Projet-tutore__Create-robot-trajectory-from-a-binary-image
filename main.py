from class_graph import *
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

#%%

class PointPublisher(Node):

    def __init__(self, lst_point):
        super().__init__('point_image_publisher')
        self.publisher_ = self.create_publisher(Point, '/point_image', 1)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.lst_point = lst_point
        
    def timer_callback(self):
        L = len(self.lst_point[0])
        if self.i < L:
            msg = Point()
            msg.x = float(self.lst_point[0][self.i])
            msg.y = float(self.lst_point[1][self.i])
            msg.z = float(self.lst_point[2][self.i])
            print(f"x = {msg.x} ; y = {msg.y} ; z = {msg.z}")
            self.publisher_.publish(msg)
        elif self.i == L:
            print("End of the communication")
        else:            
            exit()           
        self.i += 1



def main(args=None):
    
    #déggclaration de l'élément graph de la classe graph
    graph1 = graph("TE.png")
    graph1.image2coord(1,0.4)
    
    lst = graph1.trajectory_pts_reel    #lst contient les coordonées xyz
    
    #initialisation du node ros
    rclpy.init(args=args)
    point_publisher = PointPublisher(lst)
    rclpy.spin(point_publisher)
    
    #publishing
    point_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()




        
        
    
    
    


