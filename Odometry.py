
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
    v = wheel_radius/2*(ws1+ws2)*180/np.pi
    angular_velocity = wheel_radius/distance_between_wheels*(ws1 - ws2)*180/np.pi
    return[v,angular_velocity]


def inverse_kinematics(target_speed,target_angle):
    target_speed = target_speed*np.pi/180
    target_angle = target_angle*np.pi/180
    ws1 = (target_speed + target_angle*distance_between_wheels/2)/(2*wheel_radius)
    ws2 = (target_speed - target_angle*distance_between_wheels/2)/(2*wheel_radius)
    return [ws1,ws2]

def odom(linear_speed,angular_speed,delta_time):
    dangle = angular_speed*delta_time
    dx = linear_speed/angular_speed*np.sin(np.deg2rad(dangle))
    dy = linear_speed/angular_speed*(1-np.cos(np.deg2rad(dangle)))
    return [dx,dy,dangle]

def tick_odom(x,y,angle,linear_speed,angular_speed,delta_time):
    dx,dy,dangle = odom(linear_speed,angular_speed,delta_time)
    new_angle = angle + dangle
    new_x = x + dx*np.cos(np.deg2rad(angle))-dy*np.sin(np.deg2rad(angle))
    new_y = y + dx*np.sin(np.deg2rad(angle))+dy*np.cos(np.deg2rad(angle))
    return [new_x,new_y,new_angle]

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
    logs_angle = [0]

    cur_x = logs_x[0]
    cur_y = logs_y[0]
    cur_angle = logs_angle[0]

    kinematic = direct_kinematics(0,360)
    for i in range(1,800):
        if((i % 25 == 0)):
            kinematic = direct_kinematics(random.randrange(-5, 5)*180,random.randrange(-5, 5)*180)
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

    kinematic = direct_kinematics(0,360)
    for i in range(1,800):
        if((i % 25 == 0)):
            kinematic = direct_kinematics(random.randrange(-5, 5)*180,random.randrange(-5, 5)*180)
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
