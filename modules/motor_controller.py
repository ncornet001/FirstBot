from . import kinematics
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
            raise RuntimeError('No motor port found')
        
        self.dxl_io = Dynamixel.DxlIO(ports[0])
        self.dxl_io.set_wheel_mode([self.RIGHT_ID, self.LEFT_ID])
        
    def _ensure_initialized(self):
        if self.dxl_io is None:
            raise RuntimeError("Motors are not initialized. Call setup_motors() first.")
    
    def get_speed(self):
        self._ensure_initialized()
        speeds = self.dxl_io.get_moving_speed([self.LEFT_ID,self.RIGHT_ID])
        return kinematics.direct(speeds[1]*self.RIGHT_SPEED_MULT, speeds[0])            
    
    def move(self, speed_factor, turning_angle):
        self._ensure_initialized()
        left_speed, right_speed = kinematics.inverse(
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
    
    def enable_passive_mode(self):
        self._ensure_initialized()
        try:
            # Disable torque to allow free movement
            self.dxl_io.enable_torque([self.RIGHT_ID, self.LEFT_ID], False)
            print("Motors set to passive mode (torque disabled)")
        except Exception as e:
            print(f"Could not set passive mode: {e}")
            print("Motors may still be under power")
    
    def disable_passive_mode(self):
        self._ensure_initialized()
        try:
            # Re-enable torque 
            self.dxl_io.enable_torque([self.RIGHT_ID, self.LEFT_ID], True)
            print("Motors returned to active mode (torque enabled)")
        except Exception as e:
            print(f"Could not disable passive mode: {e}")

    def get_positions(self):
        self._ensure_initialized()
        try:
            positions = self.dxl_io.get_present_position([self.LEFT_ID, self.RIGHT_ID])
            return positions[0], positions[1]  # left, right positions
        except Exception as e:
            print(f"Error reading motor positions: {e}")
            return 0.0, 0.0