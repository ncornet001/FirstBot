"""Camera module for robot vision processing."""
import cv2

MINIMUM_PERCENTAGE_DETECTION = 0.03

class Camera:
    """Camera class for robot vision processing."""
    
    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.capture = None
        
    def setup(self):
        """Initialize camera connection."""
        self.capture = cv2.VideoCapture(self.camera_id)
        if not self.capture.isOpened():
            raise RuntimeError(f'Unable to open camera {self.camera_id}')
        print(f"✓ Camera {self.camera_id} initialized")
            
    def _ensure_initialized(self):
        """Check if camera is initialized."""
        if self.capture is None or not self.capture.isOpened():
            raise RuntimeError("Camera not initialized. Call setup() first.")
    
    def read_frame(self):
        """
        Capture a frame from the camera.
        
        Returns:
            tuple: (success, frame) where success is bool and frame is the image
        """
        self._ensure_initialized()
        ret, frame = self.capture.read()
        return ret, frame
    
    def release(self):
        """Release camera resources."""
        if self.capture is not None:
            self.capture.release()
            cv2.destroyAllWindows()
            print("✓ Camera released")
    
    @staticmethod
    def get_contour_center(contour):
        """
        Calculate the center of a contour.
        
        Args:
            contour: OpenCV contour
            
        Returns:
            tuple: (cx, cy) or None if contour is empty
        """
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
        return None

    @staticmethod
    def get_biggest_contour(mask):
        """
        Find the largest contour in a mask.
        
        Args:
            mask: Binary mask image
            
        Returns:
            The largest contour, or None if no contours found
        """
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None
        
        contour_max = max(contours, key=cv2.contourArea)
        if (cv2.contourArea(contour_max) < (MINIMUM_PERCENTAGE_DETECTION*mask.size)):
            return None

        return contour_max

    @staticmethod
    def get_line_mask(hsv, color_low, color_high):
        """
        Create a binary mask based on HSV color range.
        
        Args:
            hsv: Image in HSV color space
            color_low: Lower bound (H, S, V) tuple
            color_high: Upper bound (H, S, V) tuple
            
        Returns:
            Binary mask
        """
        return cv2.inRange(hsv, color_low, color_high)

    @staticmethod
    def display(frame, mask):
        """
        Display frame and mask in separate windows.
        
        Args:
            frame: Original image
            mask: Binary mask
        """
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

    @staticmethod
    def get_direction(contour, frame_width):
        """
        Calculate normalized direction error from center.
        
        Args:
            contour: Detected line contour
            frame_width: Image width in pixels
            
        Returns:
            float: Normalized error between -1 and 1, or None if no center
        """
        if frame_width <= 0:
            raise ValueError("frame_width must be positive")
            
        center = Camera.get_contour_center(contour)
        
        if center is None:
            return None
        
        error = center[0] - frame_width / 2
        error_norm = error / (frame_width / 2)
        
        return error_norm