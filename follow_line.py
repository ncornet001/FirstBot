import cv2
import datetime
import numpy as np
import matplotlib.pyplot as plt
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

COLORS = [(YELLOW_LOW, YELLOW_HIGH), (BLUE_LOW, BLUE_HIGH), (RED_LOW, RED_HIGH)]

BASE_SPEED = 0.2
RIGHT_SPEED_MULT = -1
LEFT_SPEED_MULT = 1
LEFT_ID = 2
RIGHT_ID = 1

THETA_CONST = 140 # TODO

display_on = True
motor_on = False
map_on = True

distance_between_wheels = 0.118
wheel_radius = 0.025

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

def detect_color(hsv,color_low,color_high):
    mask = cv2.inRange(hsv,color_low,color_high)
    return np.any(mask > 0)

def setup_motors():
    ports = pypot.dynamixel.get_available_ports()
    print(ports)
    if not ports:
        exit('No motor port')

    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([RIGHT_ID, LEFT_ID])

    return dxl_io

def inverse_kinematics(target_speed,target_angle):
    ws1_ms = (distance_between_wheels*np.pi*target_angle/360) + target_speed
    ws2_ms = -(distance_between_wheels*np.pi*target_angle/360) + target_speed

    ws1 = 360*(ws1_ms/(2*wheel_radius*np.pi))
    ws2 = 360*(ws2_ms/(2*wheel_radius*np.pi))
    return [ws1,ws2]

def adjust_speed(io, error_norm):

    speed_mult = 1-abs(error_norm)

    left_speed, right_speed = inverse_kinematics(BASE_SPEED*speed_mult, error_norm*THETA_CONST)
    #print(left_speed)
    #print(right_speed)
    io.set_moving_speed({RIGHT_ID: right_speed*RIGHT_SPEED_MULT})
    io.set_moving_speed({LEFT_ID: left_speed})

def display(frame, mask):
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask",mask)

def draw_map():
    ws1 = get_present_speed(1)
    ws2 = get_present_speed(2)
    
    X=[]
    Y=[]

    while start():
        aze

    return(X,Y)


def start():

    capture = cv2.VideoCapture(CAMERA_ID)

    if motor_on:
        dxl_io = setup_motors()
    color_index = 0

    start_time = datetime.datetime.now()

    try:
        seen_brown_last_frame = False
        while True:

            current_low, current_high = COLORS[color_index]

            ret, frame = capture.read()

            if not ret:
                print("No frame received")
                break

            frame = frame[-20:, :, :]
            #frame = frame[frame.shape[0]-20:, :, :]

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            if(detect_color(hsv,BROWN_LOW,BROWN_HIGH)):
                if(not seen_brown_last_frame):
                    color_index = min(2,color_index+1)
                seen_brown_last_frame = True
            else:
                seen_brown_last_frame = False

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