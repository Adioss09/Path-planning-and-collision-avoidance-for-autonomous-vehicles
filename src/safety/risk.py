import math

class RiskAssessor:
    def __init__(self, config=None):
        if config is None:
            self.critical_ttc = 1.5
            self.high_ttc = 2.5
            self.medium_ttc = 4.0
            self.min_clearance = 1.5
        else:
            self.critical_ttc = config.get('critical_ttc', 1.5)
            self.high_ttc = config.get('high_ttc', 2.5)
            self.medium_ttc = config.get('medium_ttc', 4.0)
            self.min_clearance = config.get('minimum_clearance_m', 1.5)

    def calculate_ttc(self, ego_pos, ego_v, obs_pos, obs_v):
        """
        Calculate instantaneous TTC.
        """
        dx = obs_pos[0] - ego_pos[0]
        dy = obs_pos[1] - ego_pos[1]
        
        dvx = ego_v[0] - obs_v[0]
        dvy = ego_v[1] - obs_v[1]
        
        dist = math.hypot(dx, dy)
        if dist == 0:
            return 0.0
            
        closing_speed = (dx * dvx + dy * dvy) / dist
        
        if closing_speed <= 0.1: # Not closing fast enough
            return float('inf')
            
        return dist / closing_speed
        
    def assess_risk(self, ego_state, ego_trajectory, obstacles):
        """
        ego_state: dict with 'x', 'y', 'vx', 'vy', 'v'
        ego_trajectory: list of predicted [x, y, t] for the ego vehicle
        obstacles: list of dicts with 'x', 'y', 'vx', 'vy', 'predicted_trajectory'
        """
        highest_risk = 'LOW'
        min_ttc = float('inf')
        min_clear = float('inf')
        
        ego_pos = (ego_state['x'], ego_state['y'])
        ego_v = (ego_state['vx'], ego_state['vy'])
        
        for obs in obstacles:
            obs_pos = (obs['x'], obs['y'])
            obs_v = (obs['vx'], obs['vy'])
            
            # Current raw Euclidean distance
            current_dist = math.hypot(obs_pos[0] - ego_pos[0], obs_pos[1] - ego_pos[1])
            
            # Base TTC on velocity vectors
            ttc = self.calculate_ttc(ego_pos, ego_v, obs_pos, obs_v)
            
            traj_min_clear = float('inf')
            
            # Trajectory-based dynamic clearance and risk
            if ego_trajectory and obs.get('predicted_trajectory'):
                for ep in ego_trajectory:
                    # Find closest obstacle prediction in time
                    closest_opt = None
                    min_time_diff = float('inf')
                    for opt in obs['predicted_trajectory']:
                        time_diff = abs(opt[2] - ep[2])
                        if time_diff < min_time_diff:
                            min_time_diff = time_diff
                            closest_opt = opt
                            
                    if closest_opt and min_time_diff < 0.5:
                        traj_dist = math.hypot(ep[0] - closest_opt[0], ep[1] - closest_opt[1])
                        if traj_dist < traj_min_clear:
                            traj_min_clear = traj_dist
                            
                        # If a collision happens along the predicted trajectory
                        if traj_dist < 40.0:
                            t_to_collision = ep[2] - ego_trajectory[0][2]
                            if 0 < t_to_collision < ttc:
                                ttc = t_to_collision
            else:
                traj_min_clear = current_dist
            
            if traj_min_clear < min_clear:
                min_clear = traj_min_clear
                
            if ttc < min_ttc:
                min_ttc = ttc
                
            # If we are physically colliding right now, it's critical regardless of trajectory
            if current_dist < 25.0:
                highest_risk = 'CRITICAL'
                min_clear = current_dist
                min_ttc = 0.0
                return highest_risk, min_ttc, min_clear
                
        pixel_clearance_critical = 40.0
        pixel_clearance_high = 60.0
        
        if min_clear < pixel_clearance_critical or min_ttc < self.critical_ttc:
            highest_risk = 'CRITICAL'
        elif min_clear < pixel_clearance_high or min_ttc < self.high_ttc:
            highest_risk = 'HIGH'
        elif min_ttc < self.medium_ttc:
            highest_risk = 'MEDIUM'
            
        return highest_risk, min_ttc, min_clear
