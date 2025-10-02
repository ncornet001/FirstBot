
import numpy as np
import matplotlib.pyplot as plt
import time
import datetime
from . import Kinematic

# Tracking de position
class Odometry:    
    def __init__(self, start_x=0.0, start_y=0.0, start_angle=0.0):

        self.x = start_x
        self.y = start_y
        self.angle = start_angle
        
        # # Historique de la trajectoire
        self.history_x = [start_x]
        self.history_y = [start_y]
        self.history_angle = [start_angle]
        self.history_time = [time.time()]
        
        # Dernier temps d'update
        self.last_update_time = time.time()
        
    def reset(self, x=0.0, y=0.0, angle=0.0):
        self.x = x
        self.y = y
        self.angle = angle
        self.history_x = [x]
        self.history_y = [y]
        self.history_angle = [angle]
        self.history_time = [time.time()]
        self.last_update_time = time.time()
        
    def get_position(self):
        return {
            'x': self.x,
            'y': self.y,
            'angle': self.angle
        }

    # def odom_ws(self, left_speed, right_speed, delta_time=None):
    #     linear_speed, angular_speed = Kinematic.direct(left_speed, right_speed)        
    #     self.update_odometry(linear_speed, angular_speed, delta_time)
    
    def odom(self, linear_speed, turning_angle, delta_time):
        dangle = turning_angle * delta_time
        angle_rad = np.deg2rad(dangle)
        dx = linear_speed * np.cos(angle_rad) * delta_time
        dy = linear_speed * np.sin(angle_rad) * delta_time
        return dx, dy, dangle
    
    def tick_odom(self,x,y,angle,linear_speed,angular_speed,delta_time):
        dx,dy,dangle = self.odom(linear_speed,angular_speed,delta_time)
        new_angle = angle + dangle
        new_x = x + dx*np.cos(np.deg2rad(angle))-dy*np.sin(np.deg2rad(angle))
        new_y = y + dx*np.sin(np.deg2rad(angle))+dy*np.cos(np.deg2rad(angle))
        return [new_x,new_y,new_angle]
    
    def update_odometry(self, linear_speed, turning_angle, delta_time=None):
        if delta_time is None:
            current_time = time.time()
            delta_time = current_time - self.last_update_time
            self.last_update_time = current_time
        
        dx, dy, dangle = self.odom(linear_speed, turning_angle, delta_time)
        
        self.angle += dangle
        
        self.angle = ((self.angle + 180) % 360) - 180
        
        angle_rad = np.deg2rad(self.angle - dangle)  
        self.x += dx * np.cos(angle_rad) - dy * np.sin(angle_rad)
        self.y += dx * np.sin(angle_rad) + dy * np.cos(angle_rad)
        
        # Enregistrer dans l'historique
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_angle.append(self.angle)
        self.history_time.append(time.time())
        
    
    def get_history(self):
        return {
            'x': self.history_x.copy(),
            'y': self.history_y.copy(),
            'angle': self.history_angle.copy(),
            'time': self.history_time.copy()
        }
    
    def plot_trajectory(self, show=True, save_path=None):
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Trajectoire 2D
        ax1.plot(self.history_x, self.history_y, 'b-', linewidth=2, label='Trajectoire')
        ax1.scatter([self.history_x[0]], [self.history_y[0]], 
                   c='green', s=100, marker='o', label='Départ', zorder=5)
        ax1.scatter([self.x], [self.y], 
                   c='red', s=100, marker='x', label='Position actuelle', zorder=5)
        
        # Flèche pour l'orientation actuelle
        arrow_length = 0.1
        dx_arrow = arrow_length * np.cos(np.deg2rad(self.angle))
        dy_arrow = arrow_length * np.sin(np.deg2rad(self.angle))
        ax1.arrow(self.x, self.y, dx_arrow, dy_arrow,
                 head_width=0.05, head_length=0.05, fc='red', ec='red')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectoire du robot')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        
        # Angle au fil du temps
        elapsed_times = [t - self.history_time[0] for t in self.history_time]
        ax2.plot(elapsed_times, self.history_angle, 'r-', linewidth=2)
        ax2.set_xlabel('Temps (s)')
        ax2.set_ylabel('Angle (deg)')
        ax2.set_title('Orientation au fil du temps')
        ax2.grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✓ Trajectoire sauvegardée: {save_path}")
        
        if show:
            plt.show()
        
        return fig
    
    def save_map(self, pathname):
        fig = self.plot_trajectory(show=False)
        fig.savefig(pathname+ datetime.today().strftime('%Y-%m-%d_%H-%M-%S')+".png")

    
    def __str__(self):
        """Représentation string de la position."""
        return f"Odometry(x={self.x:.3f}m, y={self.y:.3f}m, angle={self.angle:.1f}°)"
    
    def __repr__(self):
        """Représentation détaillée."""
        distance = self.get_history()
        return (f"Odometry(pos=({self.x:.3f}, {self.y:.3f}), "
                f"angle={self.angle:.1f}°, distance={distance:.3f}m, "
                f"points={len(self.history_x)})")
