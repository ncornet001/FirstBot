
import numpy as np
import matplotlib.pyplot as plt
import random
import time

global last_recorded_time

#ROBOT PROPERTIES

distance_between_wheels = 0.118
wheel_radius = 0.025
focal_distance = 0.030
camera_angle = -45
camera_height = 0.08
current_pos = [0,0,0]
    
def direct_kinematics(ws1,ws2): # Wheel Speed 1, Wheel Speed 2
    ws1_ms = (ws1/360)*(2*wheel_radius)*np.pi
    ws2_ms = (ws2/360)*(2*wheel_radius)*np.pi

    turning_angle = (ws1_ms/((distance_between_wheels)*np.pi)) * 360 -(ws2_ms/((distance_between_wheels)*np.pi)) * 360
    linear_speed = (ws1_ms + ws2_ms)/2

    return [linear_speed,turning_angle]

def inverse_kinematics(target_speed,target_angle):
    ws1_ms = (distance_between_wheels*np.pi*target_angle/720) + target_speed
    ws2_ms = -(distance_between_wheels*np.pi*target_angle/720) + target_speed

    ws1 = 360*(ws1_ms/(2*wheel_radius*np.pi))
    ws2 = 360*(ws2_ms/(2*wheel_radius*np.pi))
    return [ws1,ws2]

def odom(x,y,dir,linear_speed,turning_angle,delta):
    differential = odom_differential(linear_speed,turning_angle,delta)
    new_dir = dir + differential[2]
    new_x = x + differential[0]*np.cos(np.deg2rad(dir))-differential[1]*np.sin(np.deg2rad(dir))
    new_y = y + differential[0]*np.sin(np.deg2rad(dir))+differential[1]*np.cos(np.deg2rad(dir))
    return [new_x,new_y,new_dir]

def odom_differential(linear_speed,turning_angle,delta,steps = 10):
    dir = 0
    dx = 0
    dy = 0
    for i in range(1,steps):
        dir += turning_angle*(delta/steps)
        dx += linear_speed*np.cos(np.deg2rad(dir))*(delta/steps)
        dy += linear_speed*np.sin(np.deg2rad(dir))*(delta/steps)
    return [dx,dy,dir]

def image_to_robot(u,v):
    x = u-0.5
    y = v-0.5
    z = 1

    #Apply camera rotation
    y = y * np.cos(np.deg2rad(camera_angle)) + z * np.sin(np.deg2rad(camera_angle))
    z = -y * np.sin(np.deg2rad(camera_angle)) + z * np.cos(np.deg2rad(camera_angle))

    t = camera_height/y

    return [x*t,y*t - camera_height,z*t]

def draw_random_trajectory():
    logs_x = [0]
    logs_y = [0]
    logs_dir = [0]

    cur_x = logs_x[0]
    cur_y = logs_y[0]
    cur_dir = logs_dir[0]

    kinematic = direct_kinematics(360,360)
    for i in range(1,800):
        if(i % 25 == 0):
            kinematic = direct_kinematics(random.randrange(-5, 5)*180,random.randrange(-5, 5)*180)
            print(kinematic)
        res = odom(cur_x,cur_y,cur_dir,kinematic[0],kinematic[1],0.1)
        cur_x = res[0]
        cur_y = res[1]
        cur_dir = res[2]
        logs_x.append(cur_x)
        logs_y.append(cur_y)
        logs_dir.append(cur_dir)
    fig, axs = plt.subplots(1, 1)
    axs.scatter(logs_x,logs_y)
    axs.set_aspect('equal')
    plt.show()
