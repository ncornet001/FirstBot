import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
import threading
from modules.motor_controller import MotorController
from modules.camera import Camera

# CAMERA_ID = 0

BLUE_LOW = np.array([100, 50, 50])
BLUE_HIGH = np.array([110, 255, 255])

RED_LOW = np.array([0, 50, 50])
RED_HIGH = np.array([20, 255, 255])

YELLOW_LOW = np.array([25, 50, 50])
YELLOW_HIGH = np.array([85, 255, 255])

BROWN_LOW1 = np.array([0, 8, 56])
BROWN_HIGH1 = np.array([4, 140, 140]) #to be adjusted

BROWN_LOW2 = np.array([147, 8, 56])
BROWN_HIGH2 = np.array([178, 140, 140])

BROWN_DETECTION = 0.25
BROWN_DETECTION_FULL = 0.1

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
        self.should_exit = False
        self.keyboard_thread = None
        self.is_top_line = False

    def next_color(self):
        self.current_color_index += 1
        if self.current_color_index >= len(COLORS):
            self.should_exit = True
            return False

        return True

    def keyboard_listener(self):
        try:
            while not self.should_exit:
                user_input = input("Press 'c' to change color, 'q' to quit: ").strip().lower()
                if user_input == 'c':
                    if not self.next_color():
                        print("Last color done...")
                        break
                    print(f"New color index: {self.current_color_index}")
                elif user_input == 'q':
                    print("Quitting...")
                    self.should_exit = True
                    break
        except EOFError:
            # Ctrl+D
            self.should_exit = True

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
        
    def detect_color_full_frame(self, frame, color_low1, color_high1, color_low2, color_high2, full_frame_threshold=0.2):
        print("Assuring previous detection")
        mask = cv2.inRange(frame, color_low1, color_high1)
        mask2 = cv2.inRange(frame, color_low2, color_high2)

        mixed = cv2.bitwise_or(mask, mask2)

        color_pixels = cv2.countNonZero(mixed)
        total_pixels = mixed.size
        return (color_pixels / total_pixels) > full_frame_threshold

    def detect_color(self, frame, cropped_frame, color_low1, color_high1, color_low2, color_high2, percentage_threshold=0.3, full_frame_threshold=0.2):
        mask = cv2.inRange(frame, color_low1, color_high1)
        mask2 = cv2.inRange(frame, color_low2, color_high2)

        mixed = cv2.bitwise_or(mask, mask2)

        color_pixels = cv2.countNonZero(mixed)
        total_pixels = mask.size
        if (color_pixels / total_pixels) > percentage_threshold:
            return self.detect_color_full_frame(frame, color_low1, color_high1, color_low2, color_high2, full_frame_threshold)

        return False

    def start(self, manual_switch=False):

        if manual_switch:
            self.should_exit = False
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
            self.keyboard_thread.start()
            print("Manual mode activated. Press 'c' to change color, 'q' to quit.")
        
        try:

            last_time = 0
            last_brown_seen = time.time() - (BROWN_COOLDOWN - BROWN_START_IGNORE_TIME)

            while not self.should_exit:

                last_time = time.time()

                self.frame_count += 1
                current_low, current_high = COLORS[self.current_color_index]
                ret, frame = self.camera.read_frame()

                if not ret:
                    print("No frame received")
                    break

                full_frame = frame
                top_frame = frame[0:20, :, :]
                frame = frame[-20:, :, :]

                #frame = frame[frame.shape[0]-20:, :, :]

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                if not manual_switch:
                    full_hsv = cv2.cvtColor(full_frame, cv2.COLOR_BGR2HSV)

                    if(self.detect_color(full_hsv, hsv, BROWN_LOW1, BROWN_HIGH1, BROWN_LOW2, BROWN_HIGH2, BROWN_DETECTION, BROWN_DETECTION_FULL)):
                        if(time.time() - last_brown_seen > BROWN_COOLDOWN):
                            self.current_color_index += 1
                            if self.current_color_index > len(COLORS)-1:
                                print("Finished last color")
                                break
                            print("Color changed to index " + str(self.current_color_index))
                        last_brown_seen = time.time()
                        self.motors.move(0.3, 0)

                    if (time.time() - last_brown_seen < 3):
                        hsv = cv2.cvtColor(top_frame, cv2.COLOR_BGR2HSV)

                mask = Camera.get_line_mask(hsv, current_low, current_high)
                contour = Camera.get_biggest_contour(mask)

                speed_mult = 1
                if (time.time() - last_brown_seen < 3):
                    print("Slow mode")
                    speed_mult = .3

                if contour is not None:
                    center = Camera.get_contour_center(contour)
                    if center is not None:
                        frame_center = frame.shape[1] // 2
                        error = center[0] - frame_center

                        error_norm = (error / frame.shape[1])*2

                        speed_mult = speed_mult * (1-abs(error_norm))
                        
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
            self.should_exit = True
            self.motors.stop()
            return 1

        except Exception as e:
            print(e)
            self.should_exit = True
            self.motors.stop()
            return 1

        finally:
            self.should_exit = True
            if manual_switch and self.keyboard_thread and self.keyboard_thread.is_alive():
                self.keyboard_thread.join(timeout=1.0)
            self.motors.stop()


