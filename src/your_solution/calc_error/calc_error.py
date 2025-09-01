import math

from geometry_msgs.msg import TransformStamped, Transform

from tf_transformations import quaternion_from_euler

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration

import tf_transformations
from vision_msgs.msg import Detection2D, Detection2DArray
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from geometry_msgs.msg import Pose, PoseArray

from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float32

class CalcError(Node):
    def __init__(self):
        # Call constructor and give class a name
        super().__init__('calc_error')
        
        # Create callback delay in milliseconds
        self.callback_delay = 20

        # Look behind time in milliseconds
        self.lookbehind_time = 20
        # Convert to nanoseconds
        self.lookbehind_time_nano = Duration(nanoseconds=self.convert_ms_to_nano(self.lookbehind_time))
        
        # Set up empty transforms for all the transforms we need
        self.detected_panel = TransformStamped()
        # Creates a list of 4 panels (easier to iterate through in comparison)
        self.true_panel_list = [TransformStamped()] * 4

        self.x_err_publisher = self.create_publisher(Float32, 'your_solution/x_err', 10)
        self.y_err_publisher = self.create_publisher(Float32, 'your_solution/y_err', 10)
        self.z_err_publisher = self.create_publisher(Float32, 'your_solution/z_err', 10)

        # Set up buffer
        self.tf_buffer = Buffer()

        # We only need one listener and can lookup several times
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timer callback for loop_timer
        self.timer = self.create_timer(self.convert_ms_to_seconds(self.callback_delay), self.loop_timer)
    
    def lookup_transforms(self):

        # Get time now (so that it's the same for every lookup)
        measured_time = self.get_clock().now()
        # Get time now as a duration (not as a Time type)
        # Makes comparison easier
        measured_duration = Duration(nanoseconds=measured_time.nanoseconds)

        # Compare duration and lookbehind times
        if (measured_duration > self.lookbehind_time_nano):
            current_time = measured_time-self.lookbehind_time_nano
        # If the current duration is behind the lookbehind time, then current time is the same
        else:
            current_time = measured_time
        
        # Try to see if detected panel exists yet
        if (self.tf_buffer.can_transform('detected_panel', 'map', current_time)):
            self.detected_panel = self.tf_buffer.lookup_transform( # Look up transform
              'detected_panel',
             'map',
                current_time)
        
        for i in range(0, 4):
            # Check if true panels exist yet
            if (self.tf_buffer.can_transform(('panel_' + str(i)), 'map', current_time)):
                self.true_panel_list[i] = self.tf_buffer.lookup_transform( # Look up transform
                    ('panel_' + str(i)),
                    'map',
                    current_time)
                
        return current_time # Return current time for use later

    def panel_exists(self, num, current_time):
        return self.tf_buffer.can_transform(('panel_' + str(num)), 'map', current_time)

    def detection_exists(self, current_time):
        return self.tf_buffer.can_transform('detected_panel', 'map', current_time)

    def ret_closest_true_panel(self, current_time):

        initial_num = 3
 
        # Compute the first error for sorting purposes
        x_err_smallest = self.true_panel_list[initial_num].transform.translation.x - self.detected_panel.transform.translation.x
        y_err_smallest = self.true_panel_list[initial_num].transform.translation.y - self.detected_panel.transform.translation.y
        z_err_smallest = self.true_panel_list[initial_num].transform.translation.z - self.detected_panel.transform.translation.z

        # Compute mean error for overall error
        smallest_err = (abs(x_err_smallest) + abs(y_err_smallest))/2.0
        smallest_panel_num = initial_num

        # Tried to track closest panel (not working well)
        '''
        for i in range(1, 4):
            x_err_current = self.true_panel_list[i].transform.translation.x - self.detected_panel.transform.translation.x
            y_err_current = self.true_panel_list[i].transform.translation.y - self.detected_panel.transform.translation.y
            z_err_current = self.true_panel_list[i].transform.translation.z - self.detected_panel.transform.translation.z

            # Tried RMSE but idk if it works or not
            # total_err_current = math.sqrt(x_err_current**2 + y_err_current**2 + z_err_current**2)

            total_err_current = (abs(x_err_current) + abs(y_err_current))/2.0
            smallest_panel_num = 0

            # Compare current nth+1 panel error to last smallest error
            if total_err_current < smallest_err:
                    smallest_err = total_err_current
                    x_err_smallest = x_err_current
                    y_err_smallest = y_err_current
                    z_err_smallest = z_err_current
                    smallest_panel_num = i
        '''
        return x_err_smallest, y_err_smallest, z_err_smallest, smallest_panel_num
    
    def loop_timer(self):
        # Lookup transforms and current time
        current_time = self.lookup_transforms()

        # Try to find the error
        # Currently error is too big to be real (when panel and detected panel are close, error is too big)
        x_err, y_err, z_err, panel_num = self.ret_closest_true_panel(current_time)

        # Publish error
        self.publish_float_msg(x_err, self.x_err_publisher, "X error")
        self.publish_float_msg(y_err, self.y_err_publisher, "Y error")
        self.publish_float_msg(z_err, self.z_err_publisher, "Z error")

        self.get_logger().info("Closest panel is: " + str(panel_num))

        self.get_logger().info("X of closest panel: " + str(self.true_panel_list[panel_num].transform.translation.x))
        self.get_logger().info("Y of closest panel: " + str(self.true_panel_list[panel_num].transform.translation.y))
        self.get_logger().info("Z of closest panel: " + str(self.true_panel_list[panel_num].transform.translation.z))

        self.get_logger().info("X of detected panel: " + str(self.detected_panel.transform.translation.x))
        self.get_logger().info("Y of detected panel: " + str(self.detected_panel.transform.translation.y))
        self.get_logger().info("Z of detected panel: " + str(self.detected_panel.transform.translation.z))

    def publish_float_msg(self, float_data, pub, description):
        msg = Float32()
        msg.data = float_data
        pub.publish(msg)
        self.get_logger().info("Publishing " + description + ": " + str(msg.data))

    def convert_ms_to_seconds(self, millis):
        return millis/1000
    
    def convert_ms_to_nano(self, millis):
        return millis*1000000

def main():
    rclpy.init()
    node = CalcError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()