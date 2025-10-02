import time
import numpy as np
from modules.motor_controller import MotorController


class PassiveMode:
    def __init__(self, motors, odometry):
        self.motors = motors
        self.odometry = odometry
        
    def _setup_passive_mode(self):
        try:
            self.motors.enable_passive_mode()
        except Exception as e:
            print(f"Error setting up passive mode: {e}")
            raise
        
    def start(self):
        print("Starting passive mode")
        print("Press Ctrl+C to stop")
        
        try:
            self._setup_passive_mode()
            
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            self._show_final_results()
            return 0
            
        except Exception as e:
            print(f"Error in passive mode: {e}")
            return 1
            
        finally:
            self._cleanup()
            
    def _show_final_results(self):
        pos = self.odometry.get_position()
        print(f"Final position: x={pos['x']:.3f}, y={pos['y']:.3f}, theta={pos['angle']:.1f}")
        
    def _cleanup(self):
        try:
            self.motors.disable_passive_mode()
        except Exception as e:
            print(f"Error during cleanup: {e}")
            
