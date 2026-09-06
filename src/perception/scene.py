class Scene:
    def __init__(self):
        self.drivable_area = None
        self.dynamic_agents = []
        self.static_obstacles = []
        self.timestamp = 0.0

    def update(self, drivable_mask, detections, timestamp):
        self.timestamp = timestamp
        self.drivable_area = drivable_mask
        
        self.dynamic_agents = []
        self.static_obstacles = []
        
        for det in detections:
            # Simple grouping based on class name for PoC
            cls = det['class']
            if cls in ['pedestrian', 'bicycle', 'car', 'motorcycle', 'bus', 'truck', 'animal']:
                self.dynamic_agents.append(det)
            else:
                self.static_obstacles.append(det)

    def get_navigable_mask(self):
        """
        Returns a binary mask where 255 is navigable (drivable and free of static obstacles)
        For PoC, dynamic agents are handled by the predictive planner, 
        but static obstacles can be baked into the navigable mask.
        """
        if self.drivable_area is None:
            return None
            
        mask = self.drivable_area.copy()
        
        # Remove static obstacles from drivable area
        import cv2
        for obs in self.static_obstacles:
            x1, y1, x2, y2 = map(int, obs['bbox'])
            cv2.rectangle(mask, (x1, y1), (x2, y2), 0, -1)
            
        return mask
