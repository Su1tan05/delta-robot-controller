#! /usr/bin/env python3

import rospy
from delta_robot_controller import DeltaRobotKinematic
from std_msgs.msg import Float32MultiArray, String, Float32
import json
import ast
import numpy as np
from scipy.interpolate import CubicSpline

rospy.init_node('trajectory_contraller_node', disable_signals=True, anonymous=True)
motor1_angle_pub = rospy.Publisher('/set_angle_motor1', Float32, queue_size=10)
motor2_angle_pub = rospy.Publisher('/set_angle_motor2', Float32, queue_size=10)
motor3_angle_pub = rospy.Publisher('/set_angle_motor3', Float32, queue_size=10)

calculated_trajectory_pub = rospy.Publisher('/calculated_trajectory', String, queue_size=10)
position = [0, 0, 0]

delta = DeltaRobotKinematic.SimulatedDeltaBot(servo_link_length = 150.0, parallel_link_length = 317.0,
                            servo_displacement = 65, effector_displacement = 35)

def interpolate (trajectory_points, N):
    theta1 = []
    theta2 = []
    theta3 = []
    for point in trajectory_points:
        th1, th2, th3 = (delta.inverse(point[0], point[1], point[2]))
        if (th1 or th2 or th3 != None):
            theta1.append(th1)
            theta2.append(th2)
            theta3.append(th3)

    x = np.linspace(0, len(trajectory_points), num=len(trajectory_points))
    theta1Spline = CubicSpline(x, theta1)
    theta2Spline = CubicSpline(x, theta2)
    theta3Spline = CubicSpline(x, theta3)

    xnew = np.linspace(0, len(trajectory_points), num=N)

    theta1Interpolated = theta1Spline(xnew)
    theta2Interpolated = theta2Spline(xnew)
    theta3Interpolated = theta3Spline(xnew)

    return [theta1Interpolated, theta2Interpolated, theta3Interpolated, xnew, theta1, theta2, theta3]


def set_trajectory_callback(data):
    data = json.loads(str(data.data).replace("\\n", "").replace("\\", ""))
    print(data['trajectoryPoints'][2])
    calculated_trajectory_pub.publish(str(data['trajectoryPoints'][1]))

trajectory_sub = rospy.Subscriber('/trajectory_points', String, set_trajectory_callback, queue_size=10)
position_pub = rospy.Publisher('/robot_position', Float32MultiArray , queue_size=10)

def get_motor_data(data):
  global position
  position = delta.forward(data.data[1], data.data[4], data.data[7])
  position_pub.publish(Float32MultiArray(data=position))

sub = rospy.Subscriber('/monitoring', Float32MultiArray, get_motor_data, queue_size=10)

def set_position_callback(data):
  th1, th2, th3 = (delta.inverce(data.data[0], data.data[1], data.data[2]))
  if (th1 or th2 or th3 != None):
    motor1_angle_pub.publish(th1)
    motor2_angle_pub.publish(th2)
    motor3_angle_pub.publish(th3)

position_sub = rospy.Subscriber('/set_position', Float32MultiArray, set_position_callback, queue_size=10)

if __name__=="__main__":
  try:
    rospy.loginfo("RUN publish_position_data()")
    rospy.spin()
  except rospy.ROSInterruptException:
    pass