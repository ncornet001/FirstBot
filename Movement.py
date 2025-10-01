import Odometry
import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1])

def move(speed,angle):
    dxl_io.set_wheel_mode([1,2])
    ws = Odometry.inverse_kinematics(speed,angle)
    dxl_io.set_moving_speed({1: ws[1], 2: -ws[2]}) # Degrees / s    

def stop():
    dxl_io.set_wheel_mode([1,2])
    dxl_io.set_moving_speed({1: 0,2 : 0}) # Degrees / s  

Odometry.draw_random_trajectory()