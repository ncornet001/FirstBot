import argparse
import modules.Camera as Camera
from modules.Controls import MotorController
from modules.NewOdometry import Odometry
from goals.line_detection import FollowLine
import goals.goto as goto

def main():
    
    # Parser les arguments
    parser = argparse.ArgumentParser(description='Contrôle du robot avec goals multiples')
    parser.add_argument('--follow-line', action='store_true', help='Activer le suivi de ligne')
    parser.add_argument('--goto', nargs=3, type=float, metavar=('X', 'Y', 'ANGLE'),
                       help='Naviguer vers la position (X, Y) et s\'aligner sur un angle donné')

    args = parser.parse_args()
    print(args)


    # check if there isn't several arguments
    if (args.follow_line and args.goto):
        print('Trop de Goal!')
        exit(1)

    if not any([args.follow_line, args.goto]):
        args.follow_line = True
        print("Aucun goal spécifié, activation du suivi de ligne par défaut")
    
    print("=" * 50)
    print("INITIALISATION DU ROBOT")
    print("=" * 50)

    try : 
        Camera.setup()
        motors = MotorController()
        motors.setup_motors()
        odometry = Odometry()
        odometry.reset()
    except Exception as e:
        print(f"Erreur d'initialisation: {e}")
        return 1
    
    print("\n" + "=" * 50)
    print("DÉMARRAGE DES GOALS")
    print("=" * 50)

    if args.follow_line:
        fl = FollowLine(motors, odometry)
        fl.start()

    elif args.goto:
        print('going to :', args.goto[0], args.goto[1], args.goto[2])
        goto.start(motors,args.goto[0], args.goto[1], args.goto[2])

    else:
        print("Aucun goal activé.")

    # try:
    #     while True:
    #         # odometry
    #         linear_speed, angular_speed = motors.get_speed()
    #         odometry.odom(linear_speed, angular_speed)

    #         # calcul de vitesses selon le mode selectionner
    #         linear_speed, angular_speed = 
    #         if args.follow_line:
    #             linear_speed, angular_speed = fl.get_direction(odometry)

    #         elif args.goto:
    #             print('going to :', args.goto[0], args.goto[1], args.goto[2])
    #             goto.start(motors,args.goto[0], args.goto[1], args.goto[2])

    #         else:
    #             print("Aucun goal activé.")


    #         # donner aux controller les infos 
    #         motors.move(linear_speed, angular_speed)

    # except KeyboardInterrupt:
    #     print("Stopping motors due to Ctrl+C...")
    #     motors.stop()

    # except Exception as e:
    #     motors.stop()
    #     print(e)
    
    print("\nArrêt en cours...")
    motors.stop()
    Camera.release()
    print("Robot arrêté")
    return 0

main()