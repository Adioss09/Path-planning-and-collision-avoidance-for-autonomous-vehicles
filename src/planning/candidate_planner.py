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
        Generates smooth paths that mathematically guarantee starting at (current_x, current_y)
        and ending precisely at (goal_x, goal_y) by shifting to an intermediate lane first.
        """
        candidates = []
        
        # Determine current lane based on proximity
        closest_lane = min(self.lane_centers.keys(), key=lambda k: abs(self.lane_centers[k] - current_x))
        
        total_dy = current_y - goal_y
        
        for lane_id, lane_x in self.lane_centers.items():
            path = []
            
            if total_dy <= 0:
                path.append([goal_x, goal_y, current_time])
            else:
                for i in range(num_points):
                    s = i / (num_points - 1)
                    y = current_y - s * total_dy
                    
                    if total_dy >= 200:
                        shift_dist = 100.0
                        shift_fraction = shift_dist / total_dy
                        
                        if s < shift_fraction:
                            # Shift from current_x to lane_x
                            ls = s / shift_fraction
                            smooth = 3*ls**2 - 2*ls**3
                            x = current_x + (lane_x - current_x) * smooth
                        elif s > 1.0 - shift_fraction:
                            # Shift from lane_x to goal_x
                            ls = (s - (1.0 - shift_fraction)) / shift_fraction
                            smooth = 3*ls**2 - 2*ls**3
                            x = lane_x + (goal_x - lane_x) * smooth
                        else:
                            # Maintain lane_x
                            x = lane_x
                    else:
                        # Too close for intermediate lane shifting, shift directly to goal_x
                        smooth = 3*s**2 - 2*s**3
                        x = current_x + (goal_x - current_x) * smooth
                        
                    # Calculate time
                    if i == 0:
                        t = current_time
                    else:
                        dist = math.hypot(x - path[-1][0], y - path[-1][1])
                        t = path[-1][2] + (dist / max(current_v, 1.0))
                        
                    path.append([x, y, t])
                    
            is_lane_change = lane_id != closest_lane
            change_cost = 10.0 if is_lane_change else 0.0
            
            # Higher cost for 2 lane changes at once
            if abs(lane_id - closest_lane) > 1:
                change_cost += 20.0
                
            self.path_counter += 1
            path_id = f"PATH_{self.path_counter:03d}"
            
            candidates.append({
                'lane_id': lane_id,
                'path_id': path_id,
                'path': path,
                'cost': change_cost,
                'safe': True,
                'is_lane_change': is_lane_change
            })
            
        return candidates
        
    def generate_intersection_candidates(self, current_x, current_y, current_v, current_time, num_points=80):
        """
        Generates 3 mathematical Bezier curves for crossing a 4-way intersection.
        """
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
                
                # Cubic Bezier
                p = ((1-t)**3)*P0 + 3*((1-t)**2)*t*P1 + 3*(1-t)*(t**2)*P2 + (t**3)*P3
                x, y = p[0], p[1]
                
                if i == 0:
                    time_t = current_time
                else:
                    dist = math.hypot(x - path[-1][0], y - path[-1][1])
                    time_t = path[-1][2] + (dist / max(current_v, 1.0))
                    
                path.append([x, y, time_t])
                
            self.path_counter += 1
            path_id = f"PATH_{self.path_counter:03d}"
            
            cost = 0.0
            if r['id'] == 'LEFT': cost += 15.0 # slightly penalize turns if straight is safe
            elif r['id'] == 'RIGHT': cost += 10.0
                
            candidates.append({
                'lane_id': r['id'],
                'path_id': path_id,
                'path': path,
                'cost': cost,
                'safe': True,
                'is_lane_change': r['id'] != 'STRAIGHT'
            })
            
        return candidates
        
    def evaluate_candidates(self, candidates, obstacles):
        """
        Scores candidates based on dynamic collision risk.
        obstacles: list of dicts with 'predicted_trajectory'
        """
        collision_threshold = 45.0 # pixels
        
        for cand in candidates:
            path = cand['path']
            cand['min_clearance'] = float('inf')
            
            for pt in path:
                pt_x, pt_y, pt_t = pt[0], pt[1], pt[2]
                
                for obs in obstacles:
                    if not obs.get('predicted_trajectory'):
                        continue
                        
                    # Find obstacle position at time pt_t
                    obs_traj = obs['predicted_trajectory']
                    
                    # Interpolate or find closest time
                    closest_obs_pt = None
                    min_time_diff = float('inf')
                    for opt in obs_traj:
                        time_diff = abs(opt[2] - pt_t)
                        if time_diff < min_time_diff:
                            min_time_diff = time_diff
                            closest_obs_pt = opt
                            
                    if closest_obs_pt and min_time_diff < 0.5:
                        dist = math.hypot(pt_x - closest_obs_pt[0], pt_y - closest_obs_pt[1])
                        
                        if dist < cand['min_clearance']:
                            cand['min_clearance'] = dist
                            
                        if dist < collision_threshold:
                            cand['safe'] = False
                            cand['cost'] += 10000.0 # Massive penalty for collision
                            
            # Add cost inversely proportional to clearance
            if cand['min_clearance'] != float('inf'):
                if cand['min_clearance'] < 100:
                    cand['cost'] += (100 - cand['min_clearance']) * 2.0
                    
        return candidates
