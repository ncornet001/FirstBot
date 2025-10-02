"""Module for camera image processing."""
import cv2
import numpy as np

BLUE_LOW = np.array([100, 50, 50])
BLUE_HIGH = np.array([110, 255, 255])

RED_LOW = np.array([0, 50, 50])
RED_HIGH = np.array([20, 255, 255])

YELLOW_LOW = np.array([25, 50, 50])
YELLOW_HIGH = np.array([85, 255, 255])

BROWN_LOW = np.array([0, 0, 0])
BROWN_HIGH = np.array([180, 150, 150]) #to be adjusted

COLORS = [(BLUE_LOW, BLUE_HIGH), (RED_LOW, RED_HIGH), (YELLOW_LOW, YELLOW_HIGH)]

class NewCamera:
    
    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.capture = None
        
        
    def setup(self):
        self.capture = cv2.VideoCapture(self.camera_id)
        
        if not self.capture.isOpened():
            raise RuntimeError(f'Unable to open camera {self.camera_id}')
            
    def _ensure_initialized(self):
        if self.capture is None or not self.capture.isOpened():
            raise RuntimeError("Camera not initialized. Call setup() first.")
    
    def read_frame(self):
        self._ensure_initialized()
        ret, frame = self.capture.read()
        return frame if ret else None
    
    def get_direction(self):
        self._ensure_initialized()
        
        return NewCamera.get_direction(self, )
         
    
    def release(self):
        if self.capture is not None:
            self.capture.release()
            cv2.destroyAllWindows()
    
    @staticmethod
    def get_contour_center(contour):
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
        return None

    @staticmethod
    def get_biggest_contour(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None
        
        return max(contours, key=cv2.contourArea)

    @staticmethod
    def get_line_mask(hsv, color_low, color_high):
        return cv2.inRange(hsv, color_low, color_high)

    @staticmethod
    def display(frame, mask):
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)  # Necessary to refresh windows

    @staticmethod
    def get_direction(contour, frame_width):
        if frame_width <= 0:
            raise ValueError("frame_width must be positive")

        center = NewCamera.get_contour_center(contour)
        
        if center is None:
            return None
        
        error = center[0] - frame_width / 2
        error_norm = error / (frame_width / 2)
        
        return error_norm
    