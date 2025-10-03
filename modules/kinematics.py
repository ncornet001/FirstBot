import numpy as np
from . import metrics

def direct(ws1,ws2): # Wheel Speed 1 (Left), Wheel Speed 2 (Right)
    v = metrics.wheel_radius*(ws1+ws2)/2*(np.pi/180)
    angular_velocity = (metrics.wheel_radius/metrics.distance_between_wheels) * (ws1 - ws2)
    return[v,angular_velocity]

def inverse(target_speed,target_angle): # linear speed, angular speed 
    target_angle = target_angle*np.pi/180
    ws1 = (180/np.pi)*(target_speed + target_angle*metrics.distance_between_wheels/2)/(2*metrics.wheel_radius)
    ws2 = (180/np.pi)*(target_speed - target_angle*metrics.distance_between_wheels/2)/(2*metrics.wheel_radius)
    return [ws1,ws2]

