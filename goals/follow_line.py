import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
from modules.motor_controller import MotorController
from modules.camera import Camera

# CAMERA_ID = 0

BLUE_LOW = np.array([100, 50, 50])
BLUE_HIGH = np.array([110, 255, 255])

RED_LOW = np.array([0, 50, 50])
RED_HIGH = np.array([20, 255, 255])

YELLOW_LOW = np.array([25, 50, 50])
YELLOW_HIGH = np.array([85, 255, 255])

BROWN_LOW = np.array([0, 30, 0])
BROWN_HIGH = np.array([180, 150, 150]) #to be adjusted

BROWN_DETECTION = 0.5
BROWN_DETECTION_FULL = 0.2

COLORS = [(YELLOW_LOW, YELLOW_HIGH),(BLUE_LOW, BLUE_HIGH), (RED_LOW, RED_HIGH)]

BROWN_COOLDOWN = 5
BROWN_START_IGNORE_TIME = 5

THETA_CONST = 140

display_on = False
motor_on = True
map_on = True

distance_between_wheels = 0.118
wheel_radius = 0.025
  
class FollowLine():
    def __init__(self, motors, camera):
        self.motors:MotorController = motors
        self.camera:Camera = camera
        self.current_color_index = 0
        self.frame_count = 0

    def get_direction(self): 
        ret, frame = self.camera.read_frame()
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
        
    def detect_color_full_frame(self, frame, color_low, color_high, full_frame_threshold=0.2):
        print("Assuring previous detection")
        mask = cv2.inRange(frame, color_low, color_high)
        color_pixels = cv2.countNonZero(mask)
        total_pixels = mask.size
        return (color_pixels / total_pixels) > full_frame_threshold

    def detect_color(self, frame, cropped_frame, color_low, color_high, percentage_threshold=0.7, full_frame_threshold=0.2):
        mask = cv2.inRange(frame, color_low, color_high)
        color_pixels = cv2.countNonZero(mask)
        total_pixels = mask.size
        if (color_pixels / total_pixels) > percentage_threshold:
            return self.detect_color_full_frame(frame, color_low, color_high, full_frame_threshold)

        return False

    def start(self):
        try:

            last_time = 0
            last_brown_seen = time.time() - (BROWN_COOLDOWN - BROWN_START_IGNORE_TIME)

            while True:

                last_time = time.time()

                self.frame_count += 1
                current_low, current_high = COLORS[self.current_color_index]
                ret, frame = self.camera.read_frame()

                if not ret:
                    print("No frame received")
                    break

                full_frame = frame
                frame = frame[-20:, :, :]
                #frame = frame[frame.shape[0]-20:, :, :]

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                full_hsv = cv2.cvtColor(full_frame, cv2.COLOR_BGR2HSV)

                if(self.detect_color(full_hsv, hsv, BROWN_LOW, BROWN_HIGH, BROWN_DETECTION, BROWN_DETECTION_FULL)):
                    if(time.time() - last_brown_seen > BROWN_COOLDOWN):
                        self.current_color_index += 1
                        if self.current_color_index > len(COLORS)-1:
                            print("Finished last color")
                            break
                        print("Color changed to index " + str(self.current_color_index))
                    last_brown_seen = time.time()
                        
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
            self.motors.stop()


