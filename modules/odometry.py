
import numpy as np
import matplotlib.pyplot as plt
import time
import datetime
import threading
from . import kinematics

# Position tracking
class Odometry:    
    def __init__(self, start_x=0.0, start_y=0.0, start_angle=0.0):

        self.x = start_x
        self.y = start_y
        self.angle = start_angle
        
        # History
        self.history_x = [start_x]
        self.history_y = [start_y]
        self.history_angle = [start_angle]
        self.history_time = [time.time()]

        self.last_update_time = time.time()
        
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._motor_controller = None
        self._frequency = 100  # Hz
        self._update_count = 0
        self._start_time = time.time()
        
    def reset(self, x=0.0, y=0.0, angle=0.0):
        with self._lock:
            self.x = x
            self.y = y
            self.angle = angle
            self.history_x = [x]
            self.history_y = [y]
            self.history_angle = [angle]
            self.history_time = [time.time()]
            self.last_update_time = time.time()
            self._update_count = 0
            self._start_time = time.time()
        
    def get_position(self):
        with self._lock:
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
        with self._lock:
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
            
            # Save to history (decimated to avoid memory issues)
            current_time = time.time()
            if len(self.history_time) == 0 or current_time - self.history_time[-1] > 0.05:  # Every 50ms
                self.history_x.append(self.x)
                self.history_y.append(self.y)
                self.history_angle.append(self.angle)
                self.history_time.append(current_time)
        
    
    def get_history(self):
        with self._lock:
            return {
                'x': self.history_x.copy(),
                'y': self.history_y.copy(),
                'angle': self.history_angle.copy(),
                'time': self.history_time.copy()
            }
    
    def start_thread(self, motor_controller, frequency=100):
        if self._running:
            print("Odometry thread already running")
            return
            
        self._motor_controller = motor_controller
        self._frequency = frequency
        self._running = True
        self._thread = threading.Thread(target=self._odometry_loop, daemon=True)
        self._thread.start()
        print(f"Odometry thread started at {frequency}Hz")
        
    def stop_thread(self):
        if not self._running:
            return
            
        self._running = False
        if self._thread:
            self._thread.join()
        print("Odometry thread stopped")
        
    def _odometry_loop(self):
        dt = 1.0 / self._frequency
        last_time = time.time()
        
        while self._running:
            loop_start = time.time()
            
            try:
                linear_speed, angular_speed = self._motor_controller.get_speed()

                current_time = time.time()
                actual_dt = current_time - last_time
                last_time = current_time
                
                # Update odometry
                self.update_odometry(linear_speed, angular_speed, actual_dt)
                self._update_count += 1
                
            except Exception as e:
                print(f"Odometry thread error: {e}")
                
            # target frequency
            loop_time = time.time() - loop_start
            sleep_time = dt - loop_time
            if sleep_time > 0:
                time.sleep(sleep_time)
                

    def plot_trajectory(self, show=True, save_path=None):
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        with self._lock:
            if len(self.history_x) == 0:
                print("No trajectory data to plot")
                return fig
            hist_x = self.history_x.copy()
            hist_y = self.history_y.copy()
            hist_angle = self.history_angle.copy()
            hist_time = self.history_time.copy()
            current_x, current_y = self.x, self.y
        
        # 2D Trajectory plot
        ax1.plot(hist_x, hist_y, 'b-', linewidth=2, label='Trajectory')
        ax1.scatter([hist_x[0]], [hist_y[0]], 
                   c='green', s=100, marker='o', label='Start', zorder=5)
        ax1.scatter([current_x], [current_y], 
                   c='red', s=100, marker='x', label='Current position', zorder=5)
        
        # Actual orientation arrow using thread-safe data
        with self._lock:
            current_angle = self.angle
        arrow_length = 0.1
        dx_arrow = arrow_length * np.cos(np.deg2rad(current_angle))
        dy_arrow = arrow_length * np.sin(np.deg2rad(current_angle))
        ax1.arrow(current_x, current_y, dx_arrow, dy_arrow,
                 head_width=0.05, head_length=0.05, fc='red', ec='red')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Robot trajectory')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        
        # Angle over time
        elapsed_times = [t - hist_time[0] for t in hist_time]
        ax2.plot(elapsed_times, hist_angle, 'r-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angle (deg)')
        ax2.set_title('Orientation over time')
        ax2.grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Trajectory saved: {save_path}")
        
        if show:
            plt.show()
        
        return fig
    
    def save_map(self, pathname):
        fig = self.plot_trajectory(show=False)
        fig.savefig(pathname+ datetime.today().strftime('%Y-%m-%d_%H-%M-%S')+".png")