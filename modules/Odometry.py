
import numpy as np
import matplotlib.pyplot as plt
import random
import time
import Kinematic

global last_recorded_time
current_position = [0,0,0] # x,y,angle

def tick_odom(x,y,angle,linear_speed,angular_speed,delta_time):
    dx,dy,dangle = odom(linear_speed,angular_speed,delta_time)
    new_angle = angle + dangle
    new_x = x + dx*np.cos(np.deg2rad(angle))-dy*np.sin(np.deg2rad(angle))
    new_y = y + dx*np.sin(np.deg2rad(angle))+dy*np.cos(np.deg2rad(angle))
    return [new_x,new_y,new_angle]

def odom(linear_speed,angular_speed,delta_time):
    dangle = angular_speed*delta_time
    dx = linear_speed/angular_speed*np.sin(np.deg2rad(dangle))
    dy = linear_speed/angular_speed*(1-np.cos(np.deg2rad(dangle)))
    return [dx,dy,dangle]

def get_current_pos():
    return current_position

def draw_random_trajectory():
    logs_x = [0]
    logs_y = [0]
    logs_angle = [0]

    cur_x = logs_x[0]
    cur_y = logs_y[0]
    cur_angle = logs_angle[0]

    kinematic = Kinematic.direct(0,360)
    for i in range(1,800):
        if((i % 25 == 0)):
            kinematic = Kinematic.direct(random.randrange(-5, 5)*180,random.randrange(-5, 5)*180)
            print(kinematic)
        res = tick_odom(cur_x,cur_y,cur_angle,kinematic[0],kinematic[1],0.1)
        cur_x = res[0]
        cur_y = res[1]
        cur_angle = res[2]
        logs_x.append(cur_x)
        logs_y.append(cur_y)
        logs_angle.append(cur_angle)
    fig, axs = plt.subplots(1, 1)
    axs.scatter(logs_x,logs_y)
    axs.set_aspect('equal')
    plt.show()

def draw_trajectory():
    logs_x = [0]
    logs_y = [0]
    logs_angle = [0]

    cur_x = logs_x[0]
    cur_y = logs_y[0]
    cur_angle = logs_angle[0]

    kinematic = Kinematic.direct(0,360)
    for i in range(1,800):
        if((i % 25 == 0)):
            kinematic = Kinematic.direct(random.randrange(-5, 5)*180,random.randrange(-5, 5)*180)
            print(kinematic)
        res = tick_odom(cur_x,cur_y,cur_angle,kinematic[0],kinematic[1],0.1)
        cur_x = res[0]
        cur_y = res[1]
        cur_angle = res[2]
        logs_x.append(cur_x)
        logs_y.append(cur_y)
        logs_angle.append(cur_angle)
    fig, axs = plt.subplots(1, 1)
    axs.scatter(logs_x,logs_y)
    axs.set_aspect('equal')
    plt.show()
