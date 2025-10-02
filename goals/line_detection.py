import cv2
import datetime
import numpy as np
import matplotlib.pyplot as plt
from modules.Controls import MotorController
import modules.NewOdometry as Odometry
import modules.Camera as Camera

# CAMERA_ID = 0

BLUE_LOW = np.array([100, 50, 50])
BLUE_HIGH = np.array([110, 255, 255])

RED_LOW = np.array([0, 50, 50])
RED_HIGH = np.array([20, 255, 255])

YELLOW_LOW = np.array([25, 50, 50])
YELLOW_HIGH = np.array([85, 255, 255])

BROWN_LOW = np.array([0, 0, 0])
BROWN_HIGH = np.array([180, 150, 150]) #to be adjusted

COLORS = [(YELLOW_LOW, YELLOW_HIGH),(BLUE_LOW, BLUE_HIGH), (RED_LOW, RED_HIGH)]

BASE_SPEED = 0.2
RIGHT_SPEED_MULT = -1
LEFT_SPEED_MULT = 1
LEFT_ID = 2
RIGHT_ID = 1

THETA_CONST = 140

display_on = True
motor_on = True
map_on = True

distance_between_wheels = 0.118
wheel_radius = 0.025
  
class FollowLine():
    def __init__(self, motors, odometry):
        self.motors:MotorController = motors
        self.odometry:Odometry = odometry
        self.current_color_index = 0

    def check_color_line(self, hsv):
        pass
        # if(self.detect_color(hsv,BROWN_LOW,BROWN_HIGH)):
        #     if(not seen_brown_last_frame):
        #         color_index = color_index+1//3
        #     seen_brown_last_frame = True
        # else:
        #     seen_brown_last_frame = False
        # print("color index : ",color_index)

    def get_direction(self): 
        ret, frame = Camera.read_frame()
        current_low, current_high = COLORS[self.current_color_index]

        if not ret:
            print("No frame received")
            return 0, 0

        frame = frame[-20:, :, :]
        #frame = frame[frame.shape[0]-20:, :, :]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = Camera.get_line_mask(hsv, current_low, current_high)
        contour = Camera.get_biggest_contour(mask)

        if contour is not None:
            center = Camera.get_contour_center(contour)
            if center is not None:
                frame_center = frame.shape[1] // 2
                error = center[0] - frame_center

                error_norm = (error / frame.shape[1])*2

                speed_mult = 1-abs(error_norm)
                return speed_mult, error_norm*THETA_CONST
        return 0,0
        

    def detect_color(self,hsv,color_low,color_high,pixel_amount_threshold = 80):
        mask = cv2.inRange(hsv,color_low,color_high)
        return np.sum(mask) > (pixel_amount_threshold*255)

    def start(self):
        self.motors.setup_motors()
        Camera.setup()
        try:
            while True:
                current_low, current_high = COLORS[self.current_color_index]
                ret, frame = Camera.read_frame()

                if not ret:
                    print("No frame received")
                    break

                frame = frame[-20:, :, :]
                #frame = frame[frame.shape[0]-20:, :, :]

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # if(self.detect_color(hsv,BROWN_LOW,BROWN_HIGH)):
                #     if(not seen_brown_last_frame):
                #         color_index = min(2,color_index+1)
                #     seen_brown_last_frame = True
                # else:
                #     seen_brown_last_frame = False
                # print("color index : ",color_index)

                mask = Camera.get_line_mask(hsv, current_low, current_high)
                contour = Camera.get_biggest_contour(mask)

                if contour is not None:
                    center = Camera.get_contour_center(contour)
                    if center is not None:
                        frame_center = frame.shape[1] // 2
                        error = center[0] - frame_center

                        error_norm = (error / frame.shape[1])*2

                        speed_mult = 1-abs(error_norm)
                        linear_speed, angular_speed = self.motors.get_speed()
                        self.odometry.update_odometry(linear_speed, angular_speed)

                        self.motors.move(speed_mult, error_norm*THETA_CONST)

                        if display_on:
                            cv2.putText(frame, f"ErrorNorm: {error_norm}", (10, 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                            cv2.circle(frame, (center[0], center[1]), 5, (255,255,255), -1)

                if display_on:
                    Camera.display(frame, mask)

                    if cv2.waitKey(1) == ord('q'):
                        break
            return 0
        
        except KeyboardInterrupt:
            print("Stopping motors due to Ctrl+C...")
            self.motors.stop()
            return 1

        except Exception as e:
            print(e)
            self.motors.stop()
            return 1

        finally:
            Camera.release()
            self.motors.stop()


