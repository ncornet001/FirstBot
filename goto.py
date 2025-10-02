import datetime
import numpy as np
import pypot.dynamixel

motor_on = False

distance_between_wheels = 0.118
wheel_radius = 0.025

BASE_RIGHT_SPEED = -100
BASE_LEFT_SPEED = 100
LEFT_ID = 2
RIGHT_ID = 1

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

def odom(x,y,dir,linear_speed,turning_angle,delta):
    differential = odom_differential(linear_speed,turning_angle,delta)
    new_dir = dir + differential[2]
    new_x = x + differential[0]*np.cos(np.deg2rad(dir))-differential[1]*np.sin(np.deg2rad(dir))
    new_y = y + differential[0]*np.sin(np.deg2rad(dir))+differential[1]*np.cos(np.deg2rad(dir))
    return [new_x,new_y,new_dir]

def odom_differential(linear_speed,turning_angle,delta,steps = 1):
    dir = 0
    dx = 0
    dy = 0
    for i in range(steps):
        dir += turning_angle*(delta/steps)
        dx += linear_speed*np.cos(np.deg2rad(dir))*(delta/steps)
        dy += linear_speed*np.sin(np.deg2rad(dir))*(delta/steps)
    return [dx,dy,dir]

cur_pos = [0,0,0] # [X, Y, Dir]

def goto(x,y,dir):
    if motor_on:
        dxl_io = setup_motors()
    color_index = 0

    start_time = datetime.datetime.now()

    try:
        while True:
            #Update current position by odometry
            wsd = dxl_io.get_moving_speed([LEFT_ID,RIGHT_ID])
            kinematic = direct_kinematics(wsd[RIGHT_ID], wsd[LEFT_ID])
            new_position = odom(cur_pos[0],cur_pos[1],cur_pos[2],kinematic[0],kinematic[1])

            break
                   
    except KeyboardInterrupt:
        print("Stopping motors due to Ctrl+C...")
        if motor_on:
            dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})

    except Exception as e:
        print(e)
        if motor_on:
            dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})
            