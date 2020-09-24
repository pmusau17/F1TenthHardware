#! /usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32, Header
import time

'''This node was written in an attempt to control the acceleration of the car. As far as I can tell,
   the servo control node does not care about the acceleration info sent in the message. It does not provide that much
   control'''


class DriveNode:
    def __init__(self):
        self.velocity = 4
        self.acceleration = 0.0
        # self.drive_pub = rospy.Publisher('vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped,
        #                                  queue_size=10)
        # safer way, publish to input/teleop. The deadman switch still works
        self.drive_pub = rospy.Publisher('/ackermann_cmd_mux/input/teleop', AckermannDriveStamped,
                                         queue_size=10)
        self.vel_pub = rospy.Publisher('current_velocity', Float32, queue_size=5)
        self.accel_sub = rospy.Subscriber('acceleration', Float32, self.accel_callback)
        self.rate = rospy.Rate(40)
        self.dt = 0.025
        while not rospy.is_shutdown():
            self.drive_car()
            self.rate.sleep()

    def step_acceleration(self):
        if self.velocity <= 0 and self.acceleration <= 0:  # we don't want the car to go backwards
            self.velocity = 0
        else:
            self.velocity = self.velocity + self.dt * self.acceleration

    def accel_callback(self, msg):
        self.acceleration = msg.data
        # log_str = "Acceleration: ", self.acceleration
        # rospy.loginfo(log_str)

    def drive_car(self):

        msg_header = Header()
        ackermann_drive = AckermannDrive(acceleration=self.acceleration, speed=self.velocity)
        msg_header.stamp = rospy.Time.now()
        drive_msg = AckermannDriveStamped(header=msg_header, drive=ackermann_drive)
        self.step_acceleration()
        self.drive_pub.publish(drive_msg)
        self.vel_pub.publish(self.velocity)
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('drive_node')
    time.sleep(3)
    node = DriveNode()
    rospy.spin()
