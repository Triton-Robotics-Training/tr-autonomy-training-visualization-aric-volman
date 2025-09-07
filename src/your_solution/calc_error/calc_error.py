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

        # Create x y z error publishers
        self.x_err_publisher = self.create_publisher(Float32, 'your_solution/x_err', 10)
        self.y_err_publisher = self.create_publisher(Float32, 'your_solution/y_err', 10)
        self.z_err_publisher = self.create_publisher(Float32, 'your_solution/z_err', 10)

        # Initialize time: this object will be our current timestamp
        self.current_offset_time = Time()

        # Set up buffer
        self.tf_buffer = Buffer()

        # We only need one listener and can lookup several times
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timer callback for loop_timer
        self.timer = self.create_timer(self.convert_ms_to_seconds(self.callback_delay), self.loop_timer)
    
    def lookup_transforms(self):

        # Get time now (so that it's the same for every lookup)
        measured_current_time = self.get_clock().now()
        # Get time now as a duration (not as a Time type)
        # Makes comparison easier
        measured_current_duration = Duration(nanoseconds=measured_current_time.nanoseconds)

        # Compare duration and lookbehind times
        # Fixes a bug where ros2 complains about negative time (not a good thing)
        if (measured_current_duration > self.lookbehind_time_nano):
            # If the current duration is ahead of lookbehind time, then current time is offset
            self.current_offset_time = measured_current_time-self.lookbehind_time_nano
        else:
            # If the current duration is behind the lookbehind time, then current time is the same
            self.current_offset_time = measured_current_time
        
        # Try to see if detected panel exists yet
        if (self.detection_exists()):
            self.detected_panel = self.tf_buffer.lookup_transform( # Look up transform
              'map',
              'detected_panel',
                self.current_offset_time)
        
        for i in range(0, 4):
            # Check if true panels exist yet
            if (self.panel_exists(i)):
                self.true_panel_list[i] = self.tf_buffer.lookup_transform( # Look up transform
                    'map',
                    ('panel_' + str(i)),
                    self.current_offset_time)

    def panel_exists(self, num):
        return self.tf_buffer.can_transform('map', ('panel_' + str(num)), self.current_offset_time)

    def detection_exists(self):
        return self.tf_buffer.can_transform('map', 'detected_panel', self.current_offset_time)

    def ret_closest_true_panel(self):

        # How it works: we start out with None to represent non-existence
        # If it exists we replace None with numbers
        x_err = [None] * 4
        y_err = [None] * 4
        z_err = [None] * 4

        total_err = [None] * 4

        for i in range(0,4):

            if self.panel_exists(i) and self.detection_exists():
                self.get_logger().info("\r\nStarting comparison of panel #:" + str(i))

                # Compute the error from the detection
                x_err[i]= self.true_panel_list[i].transform.translation.x - self.detected_panel.transform.translation.x
                y_err[i] = self.true_panel_list[i].transform.translation.y - self.detected_panel.transform.translation.y
                z_err[i] = self.true_panel_list[i].transform.translation.z - self.detected_panel.transform.translation.z


                self.get_logger().info("X of panel: " + str(self.true_panel_list[i].transform.translation.x))
                self.get_logger().info("Y of panel: " + str(self.true_panel_list[i].transform.translation.y))
                self.get_logger().info("Z of panel: " + str(self.true_panel_list[i].transform.translation.z) + "\n\r")


                # Compute Root Mean Squared Error (RMSE)
                # total_err[i] = math.sqrt((x_err[i]**2 + y_err[i]**2 + z_err[i]**2)/3)
                # TODO include z error
                total_err[i] = math.sqrt((x_err[i]**2 + y_err[i]**2)/2.0)

        # Filter out nones (i.e. if one true panel exists but the others don't)
        filtered_err = list(filter(lambda x: x is not None, total_err))

        # If there is a minimum at all then return the minimum, otherwise if None then return None
        i_min = total_err.index(min(filtered_err, default=None))

        # Returns the error at the minimum index
        return x_err[i_min], y_err[i_min], z_err[i_min], i_min
    
    def loop_timer(self):
        # Lookup transforms and current time
        self.lookup_transforms()

        # Try to find the error
        # TODO clean up comments
        # Currently error is too big to be real (when panel and detected panel are close, error is too big)
        x_err, y_err, z_err, panel_num = self.ret_closest_true_panel()

        # Only publish error if it exists (not none)
        if x_err != None:

            # Publish error
            self.publish_float_msg(x_err, self.x_err_publisher, "X error")
            self.publish_float_msg(y_err, self.y_err_publisher, "Y error")
            self.publish_float_msg(z_err, self.z_err_publisher, "Z error")

            # Publish a bunch of messages and info
            self.get_logger().info("Closest panel is: " + str(panel_num))

            self.get_logger().info("X of closest panel: " + str(self.true_panel_list[panel_num].transform.translation.x))
            self.get_logger().info("Y of closest panel: " + str(self.true_panel_list[panel_num].transform.translation.y))
            self.get_logger().info("Z of closest panel: " + str(self.true_panel_list[panel_num].transform.translation.z))

            self.get_logger().info("X of detected panel: " + str(self.detected_panel.transform.translation.x))
            self.get_logger().info("Y of detected panel: " + str(self.detected_panel.transform.translation.y))
            self.get_logger().info("Z of detected panel: " + str(self.detected_panel.transform.translation.z))

    # A message too annoying to do by hand
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