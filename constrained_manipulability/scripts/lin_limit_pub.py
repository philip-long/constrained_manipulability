#!/usr/bin/env python3

import tkinter

# ROS stuff
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class LinearizationLimitPublisher(Node):

    def __init__(self):
        super().__init__('lin_limit_pub')
        
        self.publisher_ = self.create_publisher(Float32, 'lin_limit', 10)

        # Tkinter GUI setup
        master = tkinter.Tk()
        self.xslider = tkinter.Scale(master, from_=0.001, to=0.5, tickinterval=0.1,
                             resolution=0.001, length=900, orient=tkinter.HORIZONTAL, command=self.slider_event)
        self.xslider.pack()
        
        # Start GUI loop
        master.mainloop()
        

    def slider_event(self, scale):
        lin_limit = self.xslider.get()
        self.get_logger().info("{}".format(lin_limit))
        
        lin_limit_msg = Float32()
        lin_limit_msg.data = lin_limit
        self.publisher_.publish(lin_limit_msg)


def main(args=None):
    rclpy.init(args=args)

    lin_limit_publisher = LinearizationLimitPublisher()

    rclpy.spin(lin_limit_publisher)

    # Destroy the node explicitly
    lin_limit_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()