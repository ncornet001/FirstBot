import datetime
import numpy as np
import pypot.dynamixel
import time

motor_on = True

distance_between_wheels = 0.118
wheel_radius = 0.025

BASE_SPEED = 0.2
BASE_TURN_SPEED = 120
RIGHT_SPEED_MULT = -1
LEFT_SPEED_MULT = 1
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


def set_target_kinematic(io,speed,angle):
    left_speed, right_speed = inverse_kinematics(speed, angle)
    io.set_moving_speed({RIGHT_ID: right_speed*RIGHT_SPEED_MULT})
    io.set_moving_speed({LEFT_ID: left_speed})


def setup_motors():
    ports = pypot.dynamixel.get_available_ports()
    print(ports)
    if not ports:
        exit('No motor port')

    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([RIGHT_ID, LEFT_ID])

    return dxl_io

def vec_angle(x,y):
    if((x == 0) & (y == 0)):
        return 0
    if(y < 0):
        return -180*np.arccos(x/vec_length(x,y))/np.pi
    else:
        return 180*np.arccos(x/vec_length(x,y))/np.pi
    
def angle_distance(angle1, angle2):
    diff = (angle2 - angle1 + 180) % 360 - 180
    return diff

def vec_length(x,y):
    return np.sqrt(x*x + y*y)

def clamp(x,mini,maxi):
    return min(maxi,max(mini,x))

def sign(x):
    if(x>0):
        return 1
    elif(x<0):
        return -1
    return 0




def goto(x,y,dir):
    distance_threshold = 0.05
    angle_threshold = 0.1

    if motor_on:
        dxl_io = setup_motors()
    cur_pos = [0,0,0] # [X, Y, Dir]
    target_pos = [x,y,dir]

    elapsed_time = 0

    last_frame_time = time.time()
    try:
        while True:
            delta_time = time.time() - last_frame_time
            last_frame_time = time.time()
            elapsed_time += delta_time
            
            #Update current position by odometry
            wsd = dxl_io.get_moving_speed([LEFT_ID,RIGHT_ID])
            kinematic = direct_kinematics(wsd[1]*RIGHT_SPEED_MULT, wsd[0])
            new_position = odom(cur_pos[0],cur_pos[1],cur_pos[2],kinematic[0],kinematic[1],delta_time)
            cur_pos = new_position
            print(cur_pos)

            dx = target_pos[0] - cur_pos[0]
            dy = target_pos[1] - cur_pos[1]

            dir_to_target = vec_angle(dx,dy) #The angle toward the target

            distance_to_target = vec_length(dx,dy)

            ddir = angle_distance(cur_pos[2],target_pos[2])
            ddir_to_target = angle_distance(cur_pos[2],dir_to_target)
            print(ddir)
            if(distance_to_target >= distance_threshold):
                #Reach position
                speed_mult = clamp(1-(abs(ddir_to_target)/45),0,1)
                angle_mult = clamp(abs(ddir_to_target)/45,0.2,1)
                set_target_kinematic(dxl_io,BASE_SPEED*speed_mult,BASE_TURN_SPEED*sign(ddir_to_target)*angle_mult)
            else:
                #Final rotation
                if(ddir > angle_threshold):
                    angle_mult = clamp(abs(ddir_to_target)/45,0.2,1)
                    set_target_kinematic(dxl_io,0,BASE_TURN_SPEED*sign(ddir_to_target)*angle_mult)

            
            if((distance_to_target <= distance_threshold) & (abs(ddir) <= angle_threshold)):
                print("Target reached!")
                dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})
                break
                   
    except KeyboardInterrupt:
        print("Stopping motors due to Ctrl+C...")
        if motor_on:
            dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})

    except Exception as e:
        print(e)
        if motor_on:
            dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})
goto(0,0,120)