import cv2

CAMERA_ID = 0
capture = None
def setup():
    global capture
    capture = cv2.VideoCapture(CAMERA_ID)

def release():
    global capture
    if capture is not None:
        capture.release()
        cv2.destroyAllWindows()

def read_frame():
    global capture
    if capture is None:
        raise RuntimeError("Camera not initialized. Call setup() first.")
    ret, frame = capture.read()
    return ret, frame

def get_contour_center(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)

    return None

def get_biggest_contour(mask):

    contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)

    if len(contours) == 0:
        return None

    return max(contours, key=cv2.contourArea)

def get_line_mask(hsv, color_low, color_high):
    mask = cv2.inRange(hsv, color_low, color_high)

    return mask


def display(frame, mask):
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask",mask)

def get_direction(contour, frame_width):
    center = get_contour_center(contour)

    if center is None:
        return None

    error = center[0] - frame_width/2
    error_norm = error/(frame_width/2)

    return error_norm