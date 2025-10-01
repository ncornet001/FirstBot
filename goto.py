import datetime
import numpy as np
import pypot.dynamixel

motor_on = False

distance_between_wheels = 0.118
wheel_radius = 0.025

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

def odom_differential(linear_speed,turning_angle,delta,steps = 1):
    dir = 0
    dx = 0
    dy = 0
    for i in range(steps):
        dir += turning_angle*(delta/steps)
        dx += linear_speed*np.cos(np.deg2rad(dir))*(delta/steps)
        dy += linear_speed*np.sin(np.deg2rad(dir))*(delta/steps)
    return [dx,dy,dir]

def goto(x,y,dir):
    if motor_on:
        dxl_io = setup_motors()
    color_index = 0

    start_time = datetime.datetime.now()

    try:
        while True:
            break
                   
    except KeyboardInterrupt:
        print("Stopping motors due to Ctrl+C...")
        if motor_on:
            dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})

    except Exception as e:
        print(e)
        if motor_on:
            dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})
            