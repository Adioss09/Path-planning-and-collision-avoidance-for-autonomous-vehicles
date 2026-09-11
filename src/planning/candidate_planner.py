import math
import numpy as np

class CandidatePlanner:
    def __init__(self, lanes):
        """
        lanes: list of dicts [{'id': 1, 'x': 60}, {'id': 2, 'x': 120}, ...]
        """
        self.lanes = lanes
        self.lane_centers = {lane['id']: lane['x'] for lane in lanes}
        self.path_counter = 0
        
    def generate_candidates(self, current_x, current_y, current_v, current_time, goal_x, goal_y, num_points=60):
        """
        Generates dynamic candidate trajectories using lateral offsets from the current heading/position.
        This completely eliminates hardcoded lane dependencies.
        """
        candidates = []
        
        # Generate dynamic lateral offsets: [-60, -30, 0, 30, 60] pixels
        offsets = [-80.0, -40.0, 0.0, 40.0, 80.0]
        
        total_dy = current_y - goal_y
        
        for offset in offsets:
            path = []
            
            # The target intermediate X is our current X plus the lateral offset
            target_intermediate_x = current_x + offset
            
            # Clamp to physical road boundaries (assumed 15 to 225 for generic straight road)
            target_intermediate_x = max(20.0, min(220.0, target_intermediate_x))
            
            if total_dy <= 0:
                path.append([goal_x, goal_y, current_time])
            else:
                for i in range(num_points):
                    s = i / (num_points - 1)
                    y = current_y - s * total_dy
                    
                    if total_dy >= 150:
                        shift_dist = 75.0
                        shift_fraction = shift_dist / total_dy
                        
                        if s < shift_fraction:
                            # Shift from current_x to target offset
                            ls = s / shift_fraction
                            smooth = 3*ls**2 - 2*ls**3
                            x = current_x + (target_intermediate_x - current_x) * smooth
                        elif s > 1.0 - shift_fraction:
                            # Shift from offset back to goal_x
                            ls = (s - (1.0 - shift_fraction)) / shift_fraction
                            smooth = 3*ls**2 - 2*ls**3
                            x = target_intermediate_x + (goal_x - target_intermediate_x) * smooth
                        else:
                            # Maintain offset
                            x = target_intermediate_x
                    else:
                        # Direct to goal
                        smooth = 3*s**2 - 2*s**3
                        x = current_x + (goal_x - current_x) * smooth
                        
                    if i == 0:
                        t = current_time
                    else:
                        dist = math.hypot(x - path[-1][0], y - path[-1][1])
                        t = path[-1][2] + (dist / max(current_v, 1.0))
                        
                    path.append([x, y, t])
                    
            # Cost function
            deviation_cost = abs(offset) * 0.5
            
            self.path_counter += 1
            path_id = f"PATH_{self.path_counter:03d}"
            
            candidates.append({
                'lane_id': f"OFFSET_{offset}",
                'path_id': path_id,
                'path': path,
                'cost': deviation_cost,
                'safe': True,
                'is_lane_change': offset != 0.0
            })
            
        return candidates
        
    def generate_intersection_candidates(self, current_x, current_y, current_v, current_time, num_points=80):
        # Implementation remains the same as it correctly generates mathematical turning routes
        candidates = []
        routes = [
            {'id': 'STRAIGHT', 'target_x': 566.0, 'target_y': 50.0},
            {'id': 'LEFT', 'target_x': 50.0, 'target_y': 366.0},
            {'id': 'RIGHT', 'target_x': 950.0, 'target_y': 466.0}
        ]
        for r in routes:
            path = []
            P0 = np.array([current_x, current_y])
            P3 = np.array([r['target_x'], r['target_y']])
            if r['id'] == 'STRAIGHT':
                P1 = np.array([current_x, current_y - 200])
                P2 = np.array([r['target_x'], r['target_y'] + 200])
            elif r['id'] == 'LEFT':
                P1 = np.array([current_x, 366.0])
                P2 = np.array([500.0, r['target_y']])
            elif r['id'] == 'RIGHT':
                P1 = np.array([current_x, 466.0])
                P2 = np.array([600.0, r['target_y']])
            for i in range(num_points):
                t = i / (num_points - 1)
                p = ((1-t)**3)*P0 + 3*((1-t)**2)*t*P1 + 3*(1-t)*(t**2)*P2 + (t**3)*P3
                x, y = p[0], p[1]
                if i == 0:
                    time_t = current_time
                else:
                    dist = math.hypot(x - path[-1][0], y - path[-1][1])
                    time_t = path[-1][2] + (dist / max(current_v, 1.0))
                path.append([x, y, time_t])
            self.path_counter += 1
            cost = 0.0
            if r['id'] == 'LEFT': cost += 15.0
            elif r['id'] == 'RIGHT': cost += 10.0
            candidates.append({
                'lane_id': r['id'],
                'path_id': f"PATH_{self.path_counter:03d}",
                'path': path,
                'cost': cost,
                'safe': True,
                'is_lane_change': r['id'] != 'STRAIGHT'
            })
        return candidates
        
    def evaluate_candidates(self, candidates, obstacles):
        base_collision_threshold = 40.0
        
        for cand in candidates:
            path = cand['path']
            cand['min_clearance'] = float('inf')
            
            for pt in path:
                pt_x, pt_y, pt_t = pt[0], pt[1], pt[2]
                
                # Calculate time into the future for uncertainty scaling
                time_future = pt_t - path[0][2]
                
                for obs in obstacles:
                    if not obs.get('predicted_trajectory'):
                        continue
                        
                    closest_obs_pt = None
                    min_time_diff = float('inf')
                    for opt in obs['predicted_trajectory']:
                        time_diff = abs(opt[2] - pt_t)
                        if time_diff < min_time_diff:
                            min_time_diff = time_diff
                            closest_obs_pt = opt
                            
                    if closest_obs_pt and min_time_diff < 0.5:
                        dist = math.hypot(pt_x - closest_obs_pt[0], pt_y - closest_obs_pt[1])
                        
                        if dist < cand['min_clearance']:
                            cand['min_clearance'] = dist
                            
                        # Uncertainty-aware collision threshold (expands by 5px per second into future)
                        dynamic_threshold = base_collision_threshold + (time_future * 5.0)
                        
                        if dist < dynamic_threshold:
                            cand['safe'] = False
                            cand['cost'] += 10000.0
                            
            if cand['min_clearance'] != float('inf'):
                if cand['min_clearance'] < 120.0:
                    cand['cost'] += (120.0 - cand['min_clearance']) * 3.0
                    
        return candidates
