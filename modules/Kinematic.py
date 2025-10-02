import numpy as np
import modules.Metrics as Metrics

def direct(ws1,ws2): # Wheel Speed 1, Wheel Speed 2
    v = Metrics.wheel_radius*(ws1+ws2)/2*(np.pi/180)
    angular_velocity = (Metrics.wheel_radius/Metrics.distance_between_wheels) * (ws1 - ws2)
    return[v,angular_velocity]

def inverse(target_speed,target_angle): # linear speed, angular speed 
    target_angle = target_angle*np.pi/180
    ws1 = (180/np.pi)*(target_speed + target_angle*Metrics.distance_between_wheels/2)/(2*Metrics.wheel_radius)
    ws2 = (180/np.pi)*(target_speed - target_angle*Metrics.distance_between_wheels/2)/(2*Metrics.wheel_radius)
    return [ws1,ws2]

