# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
import cv2  # OpenCV library
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import numpy as np

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Constructor
        super().__init__('image_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the '/camera/image_raw/uncompressed' topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw/uncompressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
 
        # Keep track of processed marker combinations
        self.processed_combinations = set()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        
        # Checks current window height and width
        frame_height, frame_width = current_frame.shape[:2]
        print(frame_height, frame_width)

        # The following code is a simple example of colour segmentation
        # and connected components analysis

        # Convert BGR image to HSV
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Define the color ranges for masking
        color_ranges = {
            'blue': ((0, 40, 90), (36, 210, 190)),
            'pink': ((125, 65, 130), (155, 170, 210)),
            'green': ((25, 50, 10), (60, 220, 110)),
            'yellow': ((65, 110, 120), (110, 255, 210))
        }
        # Mask for blue
        light_blue = (0, 40, 90)
        dark_blue = (36, 210, 190)
        mask_blue = cv2.inRange(hsv_frame, light_blue, dark_blue)
    
    # Mask for pink
        light_pink = (125, 65, 130)
        dark_pink = (155, 170, 210)
        mask_pink = cv2.inRange(hsv_frame, light_pink, dark_pink)

        # Mask for green
        light_green = (25, 50, 10)
        dark_green = (60, 220, 110)
        mask_green = cv2.inRange(hsv_frame, light_green, dark_green)

        # Mask for yellow 
        light_yellow = (65, 110, 120)
        dark_yellow = (110, 255, 210)
        mask_yellow = cv2.inRange(hsv_frame, light_yellow, dark_yellow)

    # Combine the two masks
        combined_mask = cv2.bitwise_or(mask_blue, mask_pink)
        combined_mask2 = cv2.bitwise_or(mask_green, mask_yellow)
        combined_mask_total = cv2.bitwise_or(combined_mask, combined_mask2)

    # Apply the combined mask to the original image
        result = cv2.bitwise_and(current_frame, current_frame, mask=combined_mask_total)

        # Create masks for each color and find connected components
        masks = {}
        for color, (lower, upper) in color_ranges.items():
            masks[color] = cv2.inRange(hsv_frame, lower, upper)

        # Run connected components for each mask
        filtered_stats = {}
        for color, mask in masks.items():
            output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
            # Get statistics for current mask, skip the background label 0
            stats_list = [output[2][i] for i in range(1, output[2].shape[0])]
            # Apply the filtering conditions
            filtered_stats[color] = [stat for stat in stats_list if (stat[cv2.CC_STAT_WIDTH] > 10 and 
                                                             stat[cv2.CC_STAT_HEIGHT] > 10 and 
                                                             stat[cv2.CC_STAT_AREA] > 200)]
        # print(filtered_stats)
        # Now you can use filtered_stats with the find_marker_height function
        print(filtered_stats)
        marker_heights = self.find_marker_height(filtered_stats)
        print(marker_heights)
        # Print the height of each detected marker
        #for color1, height in marker_heights:
            # self.get_logger().info(f'Detected vertical marker with colors {color_pair} and height: {height}')
            #print(f'Detected vertical marker with colors {color1} {color1[1]} and height: {height}')

        # Display camera image
        cv2.imshow("camera", current_frame)
        cv2.imshow("mask", result)
        cv2.waitKey(1)

    def is_aligned_vertically(self, box1, box2, x_tolerance=10):
        print(box1)
        x1, _, _, _, _ = box1
        x2, _, _, _, _ = box2
        return abs(x1 - x2) <= x_tolerance
    
    # current frame width is 160px. Left, middle, right of camera corresponds to 0, 80, 160 px respectively
    # Camera total angle is 62.2 degrees, corresponding to -31.1, 0, 31.1 respective to turtlebot
    # This ratio gets 0.38875 degrees per pixel, need to recalculate for other sizes	
    def find_angle(self ,left, box_width):
        box_middle = left + (box_width / 2)
        box_middle_angle = box_middle * 0.38875
        return (box_middle_angle - 31.1)
        

    def find_marker_height(self, stats, x_tolerance=10):
        # This dictionary will store the total height of each detected vertical marker with color pairs as keys
        marker_heights = {}

        # Compare each pair of bounding boxes to find vertical alignment
        for color1, stats1 in stats.items():
            for color2, stats2 in stats.items():
                if color1 == color2:
                    continue  # Skip comparing the same color
                for i in range(0, len(stats1)):
                    for j in range(0, len(stats2)):
                        # Ensure each combination is unique
                        #if (color1, i, color2, j) in self.processed_combinations:
                        #	continue
                        box1 = stats1[i]
                        box2 = stats2[j]

                        # Check if the x-coordinates are aligned
                        if self.is_aligned_vertically(box1, box2, x_tolerance):
                            # Sum the heights if aligned
                            y1, h1 = box1[cv2.CC_STAT_TOP], box1[cv2.CC_STAT_HEIGHT]
                            y2, h2 = box2[cv2.CC_STAT_TOP], box2[cv2.CC_STAT_HEIGHT]

                            top_color = color1 if y1 < y2 else color2
                            bottom_colour = color2 if top_color == color1 else color1

                            # Store the marker height with color pair as key
                            angle = self.find_angle(y1, box1[cv2.CC_STAT_HEIGHT])
                            print(angle)
                            marker_heights[(top_color, bottom_colour)] = ((h1 + h2), angle)

                            # Add the combination to the set of processed combinations
                            self.processed_combinations.add((color1, i, color2, j))
        return marker_heights

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()