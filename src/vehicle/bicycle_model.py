import math

class KinematicBicycleModel:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, L=2.5, max_steer=0.5, max_accel=3.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        
        self.L = L # Wheelbase
        self.max_steer = max_steer
        self.max_accel = max_accel
        
    def update(self, accel, delta, dt):
        """
        Updates the vehicle state.
        accel: Acceleration (m/s^2)
        delta: Steering angle (rad)
        dt: Time step (s)
        """
        # Clamp inputs
        accel = max(min(accel, self.max_accel), -self.max_accel * 2) # Allow stronger braking
        delta = max(min(delta, self.max_steer), -self.max_steer)
        
        # Kinematic bicycle equations
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += (self.v / self.L) * math.tan(delta) * dt
        
        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        self.v += accel * dt
        
        # Prevent reverse driving for now
        if self.v < 0:
            self.v = 0
            
    def get_state(self):
        return {
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'vx': self.v * math.cos(self.yaw),
            'vy': self.v * math.sin(self.yaw),
            'v': self.v
        }
