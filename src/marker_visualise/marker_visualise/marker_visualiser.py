import rclpy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry # Odometry needed to correctly get object's pose
from tf_transformations import euler_from_quaternion # Conversion between odometry quaternions to roll/pitch/yaw



class MarkerVisualise(Node):
    def __init__(self):
        # Initialize the ROS Node
        super().__init__('MarkerVisualise')

        # Instance variables to store color and coordinates
        self.color_str = None
        self.coor_vec = None
        self.marker_array = MarkerArray()

        # Subscriber for receiving marker color data
        self.color_sub = self.create_subscription(
            String,
            'cylinder_color',
            self.color_callback,
            10)

        # Subscriber for receiving marker coordinates
        self.coord_sub = self.create_subscription(
            Point,
            'cylinder_coord',
            self.coord_callback,
            10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.yaw_callback,
            10)

        # Publisher for sending marker array data
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'markers_visualised',
            10)

        # Timer to regularly check and publish marker data
        timer_period = 0.5  # Timer set to 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def yaw_callback(self, msg):
        global robotX
        robotX = msg.pose.pose.orientation.x
        global robotY
        robotY = msg.pose.pose.orientation.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        # Convert yaw (radians) into degrees
        global yaw_degrees
        yaw_degrees = np.degrees(yaw)
        # Get the z-coordinate of the robot
        global z_from_odom
        z_from_odom = msg.pose.pose.position.z
        print(yaw_degrees)

    def appendMarker(self, pos_x, pos_y, pos_z, rgb_color):
        # Create a new Marker
        marker = Marker()
        marker.header.frame_id = "src/wall_follower/my_map"  # Frame ID for the marker
        marker.header.stamp = self.get_clock().now().to_msg()  # Timestamp for the marker
        marker.type = Marker.CYLINDER  # Type of the marker (Cylinder)

        # Setting the position of the marker
        marker.pose.position.x = pos_x
        marker.pose.position.y = pos_y
        marker.pose.position.z = 0

        # Setting the scale (size) of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0  # Opacity of the marker

        # Setting the color of the marker based on the input string
        if rgb_color == 'blue':
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 255.0
        elif rgb_color == 'green':
            marker.color.r = 0.0
            marker.color.g = 255.0
            marker.color.b = 0.0
        elif rgb_color == 'yellow':
            marker.color.r = 255.0
            marker.color.g = 255.0
            marker.color.b = 0.0

        # Add the created marker to the marker array
        self.marker_array.markers.append(marker)

    def color_callback(self, msg):
        # Callback for updating the color from the color topic
        self.color_str = msg.data

    def coord_callback(self, msg):
        test = {'height': 60, 'angle': 13}
        print(test.height)
        # Callback for updating the coordinates from the coord topic
        # self.coor_vec = msg
        global distance
        distance = 0.0288252832636871 / test.height
        global markerAngle
        markerAngle = test.angle

    def timer_callback(self):
        global distance
        distance = 0.0288252832636871 / 60
        global markerAngle
        markerAngle = 13
        newX = robotX + (distance * (np.sin(markerAngle) + yaw_degrees))
        newY = robotX + (distance * (np.cos(markerAngle) + yaw_degrees))
        # Regularly called by the timer to process and publish markers
        if self.color_str is not None and self.coor_vec is not None:
            print(self.coor_vec.x, self.coor_vec.y, self.coor_vec.z, self.color_str)
            self.appendMarker(self.coor_vec.x, self.coor_vec.y, self.coor_vec.z, self.color_str)
            self.marker_pub.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    marker_visualise_subscriber = MarkerVisualise()
    rclpy.spin(marker_visualise_subscriber)
    # Spin keeps the script from exiting until the node is shutdown

    marker_visualise_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
