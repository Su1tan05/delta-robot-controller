#! /usr/bin/env python3

import rospy
from delta_robot_controller import DeltaRobotKinematic
from std_msgs.msg import Float32MultiArray, String, Float32
import json
import ast

rospy.init_node('trajectory_contraller_node', disable_signals=True, anonymous=True)
motor1_angle_pub = rospy.Publisher('/set_angle_motor1', Float32, queue_size=10)
motor2_angle_pub = rospy.Publisher('/set_angle_motor2', Float32, queue_size=10)
motor3_angle_pub = rospy.Publisher('/set_angle_motor3', Float32, queue_size=10)

calculated_trajectory_pub = rospy.Publisher('/calculated_trajectory', String, queue_size=10)
position = [0, 0, 0]

delta = DeltaRobotKinematic.SimulatedDeltaBot(servo_link_length = 85.0, parallel_link_length = 210.0,
                            servo_displacement = 72, effector_displacement = 20)

def set_trajectory_callback(data):
    data = json.loads(str(data.data).replace("\\n", "").replace("\\", ""))
    print(data['trajectoryPoints'][2])
    calculated_trajectory_pub.publish(str(data['trajectoryPoints'][1]))

trajectory_sub = rospy.Subscriber('/trajectory_points', String, set_trajectory_callback, queue_size=10)

def get_motor_data(data):
  global position
  position = delta.forward(data.data[1], data.data[4], data.data[7])

sub = rospy.Subscriber('/monitoring', Float32MultiArray, get_motor_data, queue_size=10)

# def publish_position_data():
#   pub = rospy.Publisher('/robot_position', Float32MultiArray , queue_size=10)
#   rate = rospy.Rate(10)
#   while not rospy.is_shutdown():
#     pub.publish(Float32MultiArray(data=position))
#     rate.sleep()

if __name__=="__main__":
  try:
    rospy.loginfo("RUN publish_position_data()")
    rospy.spin()
    # publish_position_data()
  except rospy.ROSInterruptException:
    pass