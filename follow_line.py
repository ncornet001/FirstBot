import cv2
import datetime
import numpy as np
import pypot.dynamixel

CAMERA_ID = 0

BLUE_LOW = np.array([100, 50, 50])
BLUE_HIGH = np.array([110, 255, 255])

RED_LOW = np.array([0, 50, 50])
RED_HIGH = np.array([20, 255, 255])

YELLOW_LOW = np.array([25, 50, 50])
YELLOW_HIGH = np.array([85, 255, 255])

BROWN_LOW = np.array([0, 0, 0])
BROWN_HIGH = np.array([180, 150, 150]) #to be adjusted

COLORS = [(BLUE_LOW, BLUE_HIGH), (RED_LOW, RED_HIGH), (YELLOW_LOW, YELLOW_HIGH)]

BASE_RIGHT_SPEED = -100
BASE_LEFT_SPEED = 100
LEFT_ID = 2
RIGHT_ID = 1

ERROR_IMPACT = 0.5

display_on = False
motor_on = True

frames_analyzed = 0

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

def setup_motors():
    ports = pypot.dynamixel.get_available_ports()
    print(ports)
    if not ports:
        exit('No motor port')

    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([RIGHT_ID, LEFT_ID])

    return dxl_io

def adjust_speed(io, error):

    left_speed = BASE_LEFT_SPEED
    right_speed = BASE_RIGHT_SPEED

    if error < 0:
        left_speed = BASE_LEFT_SPEED*(1-(abs(error)*ERROR_IMPACT))
    else:
        right_speed = BASE_RIGHT_SPEED*(1-(error*ERROR_IMPACT))

    io.set_moving_speed({RIGHT_ID: right_speed})
    io.set_moving_speed({LEFT_ID: left_speed})

def display(frame, mask):
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask",mask)

def start():
    global frames_analyzed

    capture = cv2.VideoCapture(CAMERA_ID)

    if motor_on:
        dxl_io = setup_motors()
    color_index = 0

    start_time = datetime.datetime.now()

    try:
        while True:

            current_low, current_high = COLORS[color_index]

            ret, frame = capture.read()

            if not ret:
                print("No frame received")
                break

            frame = frame[-20:, :, :]
            #frame = frame[frame.shape[0]-20:, :, :]
            frames_analyzed += 1

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = get_line_mask(hsv, current_low, current_high)
            contour = get_biggest_contour(mask)

            if contour is not None:
                center = get_contour_center(contour)
                if center is not None:
                    frame_center = frame.shape[1] // 2
                    error = center[0] - frame_center
                    error_norm = (error / frame.shape[1])*2

                    if motor_on:
                        adjust_speed(dxl_io, error_norm)

                    if display_on:
                        cv2.putText(frame, f"ErrorNorm: {error_norm}", (10, 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                        cv2.circle(frame, (center[0], center[1]), 5, (255,255,255), -1)

            if display_on:
                display(frame, mask)

                if cv2.waitKey(1) == ord('q'):
                    break
    except KeyboardInterrupt:
        print("Stopping motors due to Ctrl+C...")
        if motor_on:
            dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})

    except Exception as e:
        print(e)
        if motor_on:
            dxl_io.set_moving_speed({RIGHT_ID: 0, LEFT_ID: 0})
            
    finally:
        capture.release()
        cv2.destroyAllWindows()

start()
