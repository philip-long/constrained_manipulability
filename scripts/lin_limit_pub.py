#!/usr/bin/env python3

import rospy
from tkinter import *
from std_msgs.msg import Float32


class pub_class:
    def __init__(self):
        rospy.init_node('talker', anonymous=True)
        self.master = Tk()
        self.xslider = Scale(self.master, from_=0.001, to=0.5, tickinterval=0.1,
                             resolution=0.001, length=900, orient=HORIZONTAL, command=self.slider_event)
        self.xslider.pack()
        self.pub = rospy.Publisher('lin_limit', Float32, queue_size=10)
        rate = rospy.Rate(10)  # 10hz

    def slider_event(self, scale):
        self.lin_limit = self.xslider.get()
        rospy.loginfo(self.lin_limit)
        self.pub.publish(self.lin_limit)

    def start(self):
        self.master.mainloop()


if __name__ == '__main__':
    try:
        gui = pub_class()
        gui.start()
    except rospy.ROSInterruptException:
        pass
