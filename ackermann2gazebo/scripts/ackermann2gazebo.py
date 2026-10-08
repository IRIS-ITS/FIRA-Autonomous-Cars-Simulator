#!/usr/bin/env python
#
# This node converts a motor PWM + front steering angle command into the
# ROS topics exposed in Gazebo for driving catvehicle. The input is a
# pair of integers [motor_pwm, steering_angle]: motor_pwm (clamped to
# [-255, 255]) sets the rear wheel speed, scaled to the rad/s expected by
# Gazebo's joint velocity controllers via ~max_rpm (the wheel RPM at full
# -scale PWM) and an optional ~gain for a gearbox or other mechanical
# reduction; steering_angle (in degrees) sets the front wheel angle
# directly, with no clamping, so the caller is responsible for keeping it
# within a sane range.

import rospy
from std_msgs.msg import Float64, Int32MultiArray
import math

class ackermann2gazebo:

    def __init__(self):
        rospy.init_node('ackermann2gazebo', anonymous=True)

        rospy.Subscriber('/catvehicle/ackermann_vel', Int32MultiArray, self.callback)
        self.pub_rearL = rospy.Publisher('/catvehicle/joint1_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('/catvehicle/joint2_velocity_controller/command', Float64, queue_size=1)
        self.pub_steerL = rospy.Publisher('/catvehicle/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/catvehicle/front_right_steering_position_controller/command', Float64, queue_size=1)

        # initial target rear wheel velocity (rad/s) and steering angle (rad)
        self.vel = 0.0
        self.steer = 0.0

        # wheel RPM at full-scale (255) motor_pwm
        self.max_rpm = rospy.get_param('~max_rpm', 100.0)

        # gain to account for a gearbox/mechanical reduction between the
        # commanded PWM and the joint's actual angular velocity; tune with
        # the ~gain param if the simulated wheel speed doesn't match the
        # commanded PWM
        self.gain = rospy.get_param('~gain', 1.0)

        # how many seconds delay for the dead man's switch
        self.timeout = rospy.Duration.from_sec(rospy.get_param('~timeout', 0.2))
        self.lastMsg = rospy.Time.now()

    def callback(self, data):
        # data.data is [motor_pwm, steering_angle]; motor_pwm is clamped to
        # [-255, 255] and converted to rad/s, steering_angle (degrees) is
        # converted to radians with no clamping
        motor_pwm = max(-255, min(255, data.data[0]))
        self.vel = self.gain*(motor_pwm/255.0)*self.max_rpm*2.0*math.pi/60.0
        self.steer = math.radians(data.data[1])
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # if we haven't heard a new command recently, zero the target
        # velocity so the vehicle stops if the commander dies or
        # disconnects; note that the steering angle is left unchanged,
        # matching cmdvel2gazebo's behavior
        if rospy.Time.now() - self.lastMsg > self.timeout:
            self.vel = 0.0

        msgRear = Float64()
        msgRear.data = self.vel
        self.pub_rearL.publish(msgRear)
        self.pub_rearR.publish(msgRear)

        msgSteer = Float64()
        msgSteer.data = self.steer
        self.pub_steerL.publish(msgSteer)
        self.pub_steerR.publish(msgSteer)


def main():
    node = ackermann2gazebo()
    rate = rospy.Rate(100, reset=True) # run at 100Hz; reset=True so a Gazebo reload (sim clock jumping backwards) doesn't kill this node
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
