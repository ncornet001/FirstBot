import numpy as np
import matplotlib.pyplot as plt
import pypot.dynamixel
import Odometry
from datetime import datetime

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

def record_reset():
    X = [0]
    Y = [0]
    angle = 0

def draw_map():
    plt.plot(X,Y)
    plt.axis('equal')
    plt.xlabel("x [meters]")
    plt.ylabel("y [meters]")
    plt.savefig("map" + datetime.today().strftime('%Y-%m-%d_%H-%M-%S')+".png")
    #plt.show()
