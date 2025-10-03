import datetime
import numpy as np
import time
from modules.motor_controller import MotorController

BASE_SPEED = 0.2
BASE_TURN_SPEED = 120
DISTANCE_THRESHOLD = 0.05
ANGLE_THRESHOLD = 5.0  # degrees
MOVE_ANGLE = 90

display_on = True

class GoTo:
    def __init__(self, motors, odometry):
        self.motors: MotorController = motors
        self.odometry = odometry
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_angle = 0.0
        self.current_position = [0.0, 0.0, 0.0]  # [x, y, angle]
        
    def set_target(self, x, y, angle):
        self.target_x = x
        self.target_y = y
        self.target_angle = angle
        print(f"Target set: ({x:.3f}, {y:.3f}), angle: {angle:.1f}°")
        
    @staticmethod
    def vec_angle(x, y):
        if x == 0 and y == 0:
            return 0
        if y < 0:
            # TODO: Can vec_length return 0?
            return -180 * np.arccos(x / GoTo.vec_length(x, y)) / np.pi
        else:
            return 180 * np.arccos(x / GoTo.vec_length(x, y)) / np.pi
    
    @staticmethod
    def angle_distance(angle1, angle2):
        diff = (angle2 - angle1 + 180) % 360 - 180
        if diff == -180:
            return 180
        return diff

    @staticmethod
    def vec_length(x, y):
        return np.sqrt(x * x + y * y)

    @staticmethod
    def clamp(x, mini, maxi):
        return min(maxi, max(mini, x))

    @staticmethod
    def sign(x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        return 0
        
    def get_current_position(self):
        pos = self.odometry.get_position()
        return [pos['x'], pos['y'], pos['angle']]
        
    def start(self):
        start_time = time.time()
        loop_count = 0
        
        try:
            print(f"Going to ({self.target_x:.3f}, {self.target_y:.3f}), angle: {self.target_angle:.1f}°")

            while True:
                loop_count += 1
                
                self.current_position = self.get_current_position()

                dx = self.target_x - self.current_position[0]
                dy = self.target_y - self.current_position[1]
                
                dir_to_target = self.vec_angle(dx, dy)  # Angle toward the target
                distance_to_target = self.vec_length(dx, dy)
                
                ddir = self.angle_distance(self.current_position[2], self.target_angle)
                ddir_to_target = self.angle_distance(self.current_position[2], dir_to_target)
                
                # Display status
                if display_on and loop_count % 20 == 0:
                    print(f"Position: ({self.current_position[0]:.3f}, {self.current_position[1]:.3f}), "
                          f"Angle: {self.current_position[2]:.1f}°")
                    print(f"Distance to target: {distance_to_target:.3f}m, "
                          f"Angle to target: {ddir_to_target:.1f}°, "
                          f"Final angle error: {ddir:.1f}°")
                
                if distance_to_target >= DISTANCE_THRESHOLD:
                    # Navigate to position
                    speed_mult = self.clamp(1 - (abs(ddir_to_target) / MOVE_ANGLE), 0, 1)
                    angle_mult = self.clamp(abs(ddir_to_target) / MOVE_ANGLE, 0.2, 1)
                    
                    self.motors.move(
                        speed_mult,
                        BASE_TURN_SPEED * -self.sign(ddir_to_target) * angle_mult
                    )
                    
                else:
                    # Final rotation
                    if abs(ddir) > ANGLE_THRESHOLD:
                        angle_mult = self.clamp(abs(ddir) / MOVE_ANGLE, 0.2, 1)
                        self.motors.move(
                            0,
                            BASE_TURN_SPEED * -self.sign(ddir) * angle_mult
                        )
                    else:
                        print("Target reached")
                        self.motors.stop()
                        break
                
                # is target reached
                if (distance_to_target <= DISTANCE_THRESHOLD and 
                    abs(ddir) <= ANGLE_THRESHOLD):
                    print("Target reached successfully!")
                    self.motors.stop()
                    break
                    
                # prevent excessive CPU usage
                time.sleep(0.05)
            
            elapsed_time = time.time() - start_time
            final_pos = self.get_current_position()
            print(f"Final position: ({final_pos[0]:.3f}, {final_pos[1]:.3f})")
            print(f"Final angle: {final_pos[2]:.1f}°")

            return 0
            
        except KeyboardInterrupt:
            print("Stopping navigation due to Ctrl+C...")
            self.motors.stop()
            return 1
            
        except Exception as e:
            print(f"Navigation error: {e}")
            self.motors.stop()
            return 1
            
        finally:
            self.motors.stop()