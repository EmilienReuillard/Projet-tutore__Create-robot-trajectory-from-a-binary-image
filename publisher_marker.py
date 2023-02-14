import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class SinusoidSurfaceMarker(Node):
    def __init__(self):
        super().__init__('sinusoid_surface_marker')
        self.marker_pub = self.create_publisher(Marker, 'sinusoid_surface_marker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.type = Marker.TRIANGLE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        
        self.num_points = 50
        self.amplitude = 1
        self.frequency = 2

    def timer_callback(self):
        self.marker.points = []
        for i in range(self.num_points):
            x = i * 0.1
            y = self.amplitude * math.sin(self.frequency * x)
            z = 0
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            self.marker.points.append(point)
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    sinusoid_surface_marker = SinusoidSurfaceMarker()
    rclpy.spin(sinusoid_surface_marker)
    sinusoid_surface_marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()