#!/usr/bin/env python3

import numpy as np
import rospy
from collections import deque

from vision_ctl.msg import ThrustHeading
from sensor_msgs.msg import Joy

class StepupFreqControl:
    def __init__(self):
        print("[Stepup Frequency Control Node] Initializing...")
        rospy.init_node("stepup_freq_ctl", anonymous=False)
        self.rate = rospy.Rate(100)

        # parameters
        self.manual_thrust_clip = rospy.get_param('~manual_thrust_clip') #0.5
        window_size = rospy.get_param('~window_size') #200
        self.thrust_scaler = rospy.get_param('~thrust_scaler') #0.8
        self.brake_window = rospy.get_param('~brake_window') #0.1

        # subscribers
        self.ctl_sub = rospy.Subscriber(
            "/ctl_cmd",
            ThrustHeading,
            self.ctl_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        # publishers
        self.stepup_ctl_pub = rospy.Publisher(
            "/ctl_cmd_stepup",
            ThrustHeading,
            queue_size=1,
        )
        self.joy_pub = rospy.Publisher(
            "/joy",
            Joy,
            queue_size=1,
        )

        self.button_codes = {
            0: "BTN_EAST",
            1: "BTN_SOUTH",
            2: "BTN_WEST",
            3: "BTN_NORTH",
            4: "BTN_TR",
            5: "BTN_TL",
            6: "BTN_THUMBR",
            7: "BTN_THUMBL",
            8: "BTN_START",
            9: "BTN_SELECT"
        }

        self.axis_codes = {
            0: "ABS_X",
            1: "ABS_Y",
            2: "ABS_RX",
            3: "ABS_RY",
            4: "ABS_HAT0X",
            5: "ABS_HAT0Y",
            6: "ABS_Z",
            7: "ABS_RZ",
        }

        self.r_btc = {value: key for key, value in self.button_codes.items()}
        self.r_atc = {value: key for key, value in self.axis_codes.items()}

        self.cmd_msg = ThrustHeading()
        self.cmd_msg.thrust = 0.0
        self.cmd_msg.heading = 0.0
        self.cmd_msg_joy = None

        self.queue = deque(maxlen=window_size)
        [self.queue.append(0) for i in range(window_size)]

    def ctl_callback(self, msg):
        self.cmd_msg = msg

    def upsample(self):
        self.queue.append(self.cmd_msg.thrust)
        upsampled_cmd = sum(self.queue) / len(self.queue)

        return upsampled_cmd

    def publish_ctl_joy(self, msg_out):

        self.cmd_msg_joy = Joy()
        # print(self.cmd_msg_joy)

        # init button list
        self.cmd_msg_joy.buttons = []
        for i in range(10):
            self.cmd_msg_joy.buttons.append(0)

        # init axes list
        self.cmd_msg_joy.axes = []
        for i in range(8):
            self.cmd_msg_joy.axes.append(0)

        # print(self.cmd_msg_joy)
        
        # set turning command
        self.cmd_msg_joy.axes[self.r_atc["ABS_X"]] = msg_out.heading

        # set thrust values
        msg_out.thrust = np.clip(msg_out.thrust * self.thrust_scaler, -self.manual_thrust_clip, self.manual_thrust_clip)
        # print('[VISION_TO_LOWCTL] thrust = {}'.format(msg_out.thrust))

        # set brake if thrust values close to 0 or not found object in image
        if (msg_out.thrust > self.thrust_scaler*-self.brake_window and msg_out.thrust < self.thrust_scaler*self.brake_window): # or self.object_not_found:
            # print('[VISION_TO_LOWCTL] Braking')
            self.cmd_msg_joy.buttons[self.r_btc["BTN_EAST"]] = 1
            self.cmd_msg_joy.axes[self.r_atc["ABS_RZ"]] = 0.0
            self.cmd_msg_joy.axes[self.r_atc["ABS_Z"]] = 0.0

        # commanding forward thrust
        elif msg_out.thrust > 0:
            # print('[VISION_TO_LOWCTL] Forward')
            self.cmd_msg_joy.buttons[self.r_btc["BTN_EAST"]] = 0
            self.cmd_msg_joy.axes[self.r_atc["ABS_RZ"]] = msg_out.thrust
            self.cmd_msg_joy.axes[self.r_atc["ABS_Z"]] = 0.0

        # command reverse thrust
        elif msg_out.thrust < 0:
            # print('[VISION_TO_LOWCTL] Reverse')
            self.cmd_msg_joy.buttons[self.r_btc["BTN_EAST"]] = 0
            self.cmd_msg_joy.axes[self.r_atc["ABS_RZ"]] = 0.0
            self.cmd_msg_joy.axes[self.r_atc["ABS_Z"]] = -msg_out.thrust

        # in case none of the if's are triggered
        else:
            print('[VISION_TO_LOWCTL] ERROR ERROR not recognizing thrust request!')
            return

        self.joy_pub.publish(self.cmd_msg_joy)

    def spin(self):

        while not rospy.is_shutdown():

            if True:
                self.upsampled_cmd = self.upsample()

                #publish boi
                # make self.upsampled_cmd into thrustheading msg
                stepup_msg = ThrustHeading()
                stepup_msg.thrust = self.upsampled_cmd
                stepup_msg.heading = self.cmd_msg.heading 

                #publish
                self.stepup_ctl_pub.publish(stepup_msg)
                self.publish_ctl_joy(stepup_msg)

            # print(f'{rospy.Time.now().to_sec()}')

            self.rate.sleep()

if __name__ == "__main__":

    node = StepupFreqControl()
    node.spin()
