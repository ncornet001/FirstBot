import numpy as np
import matplotlib.pyplot as plt
import pypot.dynamixel
import Odometry

X = [0]
Y = [0]
angle = 0

def record_new_position(new_x,new_y,delta_time):
    X.append(new_x)
    Y.append(new_y)

def record_new_wheel_movement(ws1,ws2,delta_time):
    linear_speed,angular_speed = Odometry.direct_kinematics(ws1,ws2)
    new_x,new_y,new_angle = Odometry.tick_odom(X[-1],Y[-1],angle,linear_speed,angular_speed,delta_time)
    angle = new_angle
    X.append(new_x)
    Y.append(new_y)

def draw_map():
    fig, axs = plt.subplots(1, 1)
    axs.scatter(X,Y)
    axs.set_aspect('equal')
    plt.show()
