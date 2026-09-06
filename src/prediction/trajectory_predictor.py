import numpy as np

class TrajectoryPredictor:
    def __init__(self, horizon_seconds=3.0, timestep=0.2):
        self.horizon_seconds = horizon_seconds
        self.timestep = timestep
        
    def predict(self, track_history):
        """
        track_history: list of [x, y, timestamp] for a single object
        Returns: list of predicted [x, y, timestamp]
        """
        if len(track_history) < 2:
            # Not enough data for velocity
            if len(track_history) == 1:
                return []
            return []
            
        # Use last two points for velocity
        p1 = track_history[-2]
        p2 = track_history[-1]
        
        dt = p2[2] - p1[2]
        if dt <= 0:
            return []
            
        vx = (p2[0] - p1[0]) / dt
        vy = (p2[1] - p1[1]) / dt
        
        # Simple velocity smoothing (if we have more points)
        if len(track_history) >= 4:
            p0 = track_history[-4]
            dt_long = p2[2] - p0[2]
            if dt_long > 0:
                vx = (p2[0] - p0[0]) / dt_long
                vy = (p2[1] - p0[1]) / dt_long
                
        predictions = []
        curr_x, curr_y = p2[0], p2[1]
        curr_t = p2[2]
        
        steps = int(self.horizon_seconds / self.timestep)
        
        for _ in range(steps):
            curr_x += vx * self.timestep
            curr_y += vy * self.timestep
            curr_t += self.timestep
            predictions.append([curr_x, curr_y, curr_t])
            
        return predictions
