"""Module pour contrôler les moteurs du robot."""
from . import Kinematic
import pypot.dynamixel as Dynamixel


class MotorController:    
    def __init__(self, left_id=2, right_id=1, base_speed=0.2):
        self.LEFT_ID = left_id
        self.RIGHT_ID = right_id
        self.BASE_SPEED = base_speed
        self.RIGHT_SPEED_MULT = -1
        self.LEFT_SPEED_MULT = 1
        self.dxl_io = None
        
    def setup_motors(self):
        ports = Dynamixel.get_available_ports()
        if not ports:
            raise RuntimeError('Aucun port moteur trouvé')
        
        self.dxl_io = Dynamixel.DxlIO(ports[0])
        self.dxl_io.set_wheel_mode([self.RIGHT_ID, self.LEFT_ID])
        
    def _ensure_initialized(self):
        if self.dxl_io is None:
            raise RuntimeError("Les moteurs ne sont pas initialisés. Appelez setup_motors() d'abord.")
    
    def get_speed(self):
        self._ensure_initialized()
        speeds = self.dxl_io.get_moving_speed([self.LEFT_ID,self.RIGHT_ID])
        return Kinematic.direct(speeds[1]*self.RIGHT_SPEED_MULT, speeds[0])            
    
    def move(self, speed_factor, turning_angle):
        self._ensure_initialized()
        left_speed, right_speed = Kinematic.inverse(
            self.BASE_SPEED * speed_factor, 
            turning_angle
        )
        self.dxl_io.set_moving_speed({
            self.RIGHT_ID: right_speed * self.RIGHT_SPEED_MULT,
            self.LEFT_ID: left_speed * self.LEFT_SPEED_MULT
        })
    
    def stop(self):
        self._ensure_initialized()
        self.dxl_io.set_moving_speed({
            self.RIGHT_ID: 0,
            self.LEFT_ID: 0
        })