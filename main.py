import argparse
from datetime import datetime
from modules.camera import Camera
from modules.motor_controller import MotorController
from modules.odometry import Odometry
from goals.follow_line import FollowLine  
from goals.go_to import GoTo
from goals.passive_mode import PassiveMode

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Robot control with multiple goals')
    parser.add_argument('--follow-line', action='store_true', help='Enable line following')
    parser.add_argument('--manual-switch', action='store_true', 
                       help='Enable manual color switching (use with --follow-line)')
    parser.add_argument('--goto', nargs=3, type=float, metavar=('X', 'Y', 'ANGLE'),
                       help='Navigate to position (X, Y) and align to given angle')
    parser.add_argument('--passive-mode', action='store_true', 
                       help='Passive mode - track position during manual movement')

    args = parser.parse_args()
    print(args)

    if args.manual_switch and not args.follow_line:
        print('Error: --manual-switch can only be used with --follow-line')
        exit(1)

    # check if there isn't several arguments
    goal_count = sum([args.follow_line, bool(args.goto), args.passive_mode])
    if goal_count > 1:
        print('Too many goals! Only one at a time.')
        exit(1)

    if goal_count == 0:
        args.follow_line = True
        print("No goal specified, activating line following by default")
    
    print("=" * 50)
    print("ROBOT INITIALIZATION")
    print("=" * 50)

    try : 
        motors = MotorController(base_speed=0.4)
        motors.setup_motors()
        odometry = Odometry()
        odometry.reset()
        
        odometry.start_thread(motors, frequency=100)
        
    except Exception as e:
        print(f"Initialization error: {e}")
        return 1
    
    print("\n" + "=" * 50)
    print("STARTING GOALS")
    print("=" * 50)

    if args.follow_line:

        try : 
            camera = Camera()
            camera.setup()
            
        except Exception as e:
            print(f"Initialization error: {e}")
            return 1

        fl = FollowLine(motors, camera)
        fl.start(args.manual_switch)
        odometry.save_map("follow_line_map_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".png")

    elif args.goto:
        print('going to :', args.goto[0], args.goto[1], args.goto[2])
        gt = GoTo(motors, odometry)
        gt.set_target(args.goto[0], args.goto[1], args.goto[2])
        gt.start()
        odometry.save_map("go_to_map_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".png")

    elif args.passive_mode:
        print('Activating passive mode')
        pm = PassiveMode(motors, odometry)
        pm.start()
        odometry.save_map("passive_mode_map_"+datetime.now().strftime("%Y%m%d_%H%M%S")+".png")

    else:
        print("No goal activated.")
        return 1
    
    print("\nStopping...")
    
    odometry.stop_thread()
    
    motors.stop()
    camera.release()
    print("Robot stopped")
    return 0

main()