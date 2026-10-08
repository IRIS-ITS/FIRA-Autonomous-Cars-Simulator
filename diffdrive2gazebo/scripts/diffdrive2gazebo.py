#!/usr/bin/env python
#
# This node converts differential-drive wheel velocity commands into the
# ROS topics exposed in Gazebo for driving catvehicle's rear wheels
# directly (skid-drive style). The input is a pair of integer wheel
# speeds (vel_left, vel_right) given in RPM; these are converted to the
# rad/s expected by Gazebo's joint velocity controllers, with an optional
# gain to account for a gearbox or other mechanical reduction between the
# commanded RPM and the joint's actual angular velocity.
#
# catvehicle's front wheels steer rather than skid, so a pure rear-wheel
# differential barely turns the chassis: the straight, high-friction
# front wheels resist the yaw a skid-drive relies on. To get a usable
# turn out of an Ackermann chassis, the wheel-speed difference is also
# fed to the front steering controllers (scaled by ~steer_gain, clamped
# to ~max_steer), so the front wheels assist the turn instead of
# resisting it.

import rospy
from std_msgs.msg import Float64, Int32MultiArray
import math

class diffdrive2gazebo:

    def __init__(self):
        rospy.init_node('diffdrive2gazebo', anonymous=True)

        rospy.Subscriber('/catvehicle/diffdrive_vel', Int32MultiArray, self.callback)
        self.pub_left = rospy.Publisher('/catvehicle/joint1_velocity_controller/command', Float64, queue_size=1)
        self.pub_right = rospy.Publisher('/catvehicle/joint2_velocity_controller/command', Float64, queue_size=1)
        self.pub_steerL = rospy.Publisher('/catvehicle/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/catvehicle/front_right_steering_position_controller/command', Float64, queue_size=1)

        # initial target wheel velocities (rad/s) are 0
        self.vel_left = 0.0
        self.vel_right = 0.0

        # gain to account for a gearbox/mechanical reduction between the
        # commanded RPM and the joint's actual angular velocity; tune with
        # the ~gain param if the simulated wheel speed doesn't match the
        # commanded RPM
        self.gain = rospy.get_param('~gain', 1.0)

        # how much front steering angle (rad) to add per rad/s of
        # (vel_right - vel_left); positive steer turns left, matching a
        # faster right wheel the same way cmdvel2gazebo's Ackermann model
        # does
        self.steer_gain = rospy.get_param('~steer_gain', 0.05)

        # clamp to the same ideal max steering angle cmdvel2gazebo uses
        self.max_steer = rospy.get_param('~max_steer', 0.6)

        # how many seconds delay for the dead man's switch
        self.timeout = rospy.Duration.from_sec(rospy.get_param('~timeout', 0.2))
        self.lastMsg = None

        # whether we're currently asserting commands onto the shared
        # rear-wheel/steering topics; starts false (no message yet) so we
        # don't fight cmdvel2gazebo/ackermann2gazebo before anyone has
        # actually commanded this node
        self.active = False

    def callback(self, data):
        # data.data is [vel_left, vel_right] in RPM; convert to rad/s, then
        # apply the mechanical reduction gain
        self.vel_left = self.gain*data.data[0]*2.0*math.pi/60.0
        self.vel_right = self.gain*data.data[1]*2.0*math.pi/60.0
        self.lastMsg = rospy.Time.now()
        self.active = True

    def publish(self):
        if self.lastMsg is None:
            # never received a command; stay silent so we don't fight
            # whichever other node is actually driving the vehicle
            return

        if rospy.Time.now() - self.lastMsg > self.timeout:
            if not self.active:
                # already idle; stay silent and yield the shared topics
                return
            # just went idle: publish one final stop, then go silent
            self.vel_left = 0.0
            self.vel_right = 0.0
            self.active = False

        msgLeft = Float64()
        msgLeft.data = self.vel_left
        self.pub_left.publish(msgLeft)

        msgRight = Float64()
        msgRight.data = self.vel_right
        self.pub_right.publish(msgRight)

        steer = max(-self.max_steer, min(self.max_steer, self.steer_gain*(self.vel_right - self.vel_left)))
        msgSteer = Float64()
        msgSteer.data = steer
        self.pub_steerL.publish(msgSteer)
        self.pub_steerR.publish(msgSteer)


def main():
    node = diffdrive2gazebo()
    rate = rospy.Rate(100, reset=True) # run at 100Hz; reset=True so a Gazebo reload (sim clock jumping backwards) doesn't kill this node
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
