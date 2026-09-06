import json
import time

class MetricsTracker:
    def __init__(self):
        self.metrics = {
            "scenario_completed": False,
            "collision_count": 0,
            "replanning_count": 0,
            "emergency_interventions": 0,
            "min_clearance": float('inf'),
            "path_length": 0.0,
            "travel_time": 0.0
        }
        self.start_time = None
        self.last_pos = None

    def start_scenario(self, ego_pos):
        self.start_time = time.time()
        self.last_pos = ego_pos

    def update(self, ego_pos, current_risk, is_replanning, min_clearance):
        if self.start_time is None:
            self.start_time = time.time()
            self.last_pos = ego_pos
            
        import math
        dist = math.hypot(ego_pos[0] - self.last_pos[0], ego_pos[1] - self.last_pos[1])
        self.metrics["path_length"] += dist
        self.last_pos = ego_pos
        
        if min_clearance < self.metrics["min_clearance"]:
            self.metrics["min_clearance"] = min_clearance
            
        if current_risk == 'CRITICAL' and min_clearance < 0.5:
            # Assuming collision threshold is 0.5m
            self.metrics["collision_count"] += 1
            
        if is_replanning:
            self.metrics["replanning_count"] += 1
            
        if current_risk == 'CRITICAL':
            self.metrics["emergency_interventions"] += 1

    def finish_scenario(self, success=True):
        self.metrics["scenario_completed"] = success
        if self.start_time:
            self.metrics["travel_time"] = time.time() - self.start_time
            
    def save(self, filepath):
        with open(filepath, 'w') as f:
            json.dump(self.metrics, f, indent=4)
            
    def get_summary(self):
        return (f"Completed: {self.metrics['scenario_completed']} | "
                f"Collisions: {self.metrics['collision_count']} | "
                f"Replans: {self.metrics['replanning_count']} | "
                f"Min Clearance: {self.metrics['min_clearance']:.2f}m")
