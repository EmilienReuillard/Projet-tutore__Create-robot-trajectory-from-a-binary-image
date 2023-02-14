"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math as math

class LineStripPublisher(Node):

    def __init__(self):
        super().__init__('line_strip_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)

        timer_period = 0.1  # en secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.marker_id = 0

    def timer_callback(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = 'base_link'  # le frame_id dans lequel les points du marker sont définis
        marker_msg.header.stamp = self.get_clock().now().to_msg()

        marker_msg.ns = 'line_strip'
        marker_msg.id = self.marker_id
        self.marker_id += 1

        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD

        marker_msg.pose.position.x = 0.0
        marker_msg.pose.position.y = 0.0
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.x = 0.1  # l'épaisseur de la ligne en mètres
        marker_msg.color.a = 1.0  # l'opacité du marker, 1.0 signifie complètement opaque
        marker_msg.color.r = 1.0  # la composante rouge de la couleur
        marker_msg.color.g = 0.0  # la composante verte de la couleur
        marker_msg.color.b = 0.0  # la composante bleue de la couleur

        # Définition des points du LineStrip
        num_points = 100
        marker_msg.points = [Point() for _ in range(num_points)]
        for i in range(num_points):
            marker_msg.points[i].x = i / 10.0
            marker_msg.points[i].y = 0.5 * math.sin(i / 10.0)  # une fonction sinusoïdale simple
            marker_msg.points[i].z = 0.0

        self.publisher_.publish(marker_msg)

def main(args=None):
    rclpy.init(args=args)
    line_strip_publisher = LineStripPublisher()
    rclpy.spin(line_strip_publisher)
    line_strip_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

    """
   
import rclpy
import math
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class SinusoidalSurfacePublisher(Node):

    def __init__(self):
        super().__init__('sinusoidal_surface_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)

        timer_period = 0.1  # en secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.marker_id = 0

    def timer_callback(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = 'base_link'  # le frame_id dans lequel les points du marker sont définis
        marker_msg.header.stamp = self.get_clock().now().to_msg()

        marker_msg.ns = 'sinusoidal_surface'
        marker_msg.id = self.marker_id
        self.marker_id += 1

        marker_msg.type = Marker.TRIANGLE_STRIP
        marker_msg.action = Marker.ADD

        marker_msg.pose.position.x = 0.0
        marker_msg.pose.position.y = 0.0
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.x = 1.0  # la largeur de la surface en mètres
        marker_msg.scale.y = 0.1  # l'épaisseur de la surface en mètres
        marker_msg.scale.z = 1.0  # la longueur de la surface en mètres
        marker_msg.color.a = 1.0  # l'opacité du marker, 1.0 signifie complètement opaque
        marker_msg.color.r = 1.0  # la composante rouge de la couleur
        marker_msg.color.g = 0.0  # la composante verte de la couleur
        marker_msg.color.b = 0.0  # la composante bleue de la couleur

        # Définition des points pour le TriangleStrip
        num_points = 100
        marker_msg.points = [Point() for _ in range(2 * num_points)]
        for i in range(num_points):
            marker_msg.points[2*i].x = i / 10.0
            marker_msg.points[2*i].y = 0.5 * math.sin(i / 10.0)
            marker_msg.points[2*i].z = 0.0
            marker_msg.points[2*i+1].x = i / 10.0
            marker_msg.points[2*i+1].y = 0.5 * math.sin(i / 10.0) - 0.1
            marker_msg.points[2*i+1].z = 1.0

        self.publisher_.publish(marker_msg)

def main(args=None):
    rclpy.init(args=args)
    sinusoidal_surface_publisher = SinusoidalSurfacePublisher()
    rclpy.spin(sinusoidal_surface_publisher)
    sinusoidal_surface_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()