import heapq
import numpy as np
import math

class AStarPlanner:
    def __init__(self, grid_resolution=0.5):
        self.resolution = grid_resolution

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
        
    def plan(self, costmap, start, goal):
        """
        costmap: 2D numpy array where higher values are higher cost, and inf/255 is obstacle
        start: (x, y) grid coordinates
        goal: (x, y) grid coordinates
        Returns: list of (x, y) grid coordinates representing the path
        """
        # Ensure start and goal are within bounds
        h, w = costmap.shape
        start = (int(max(0, min(start[0], w-1))), int(max(0, min(start[1], h-1))))
        goal = (int(max(0, min(goal[0], w-1))), int(max(0, min(goal[1], h-1))))
        
        # If start is in an obstacle, try to find nearest free space
        if costmap[start[1], start[0]] >= 250:
            return [] # In a real system, we'd search for a valid start
            
        frontier = []
        heapq.heappush(frontier, (0, start))
        
        came_from = {}
        cost_so_far = {}
        
        came_from[start] = None
        cost_so_far[start] = 0
        
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        
        while frontier:
            current_priority, current = heapq.heappop(frontier)
            
            # Allow early exit if close enough
            if self.heuristic(current, goal) < 2.0:
                goal = current
                break
                
            for dx, dy in directions:
                next_node = (current[0] + dx, current[1] + dy)
                
                # Check bounds
                if not (0 <= next_node[0] < w and 0 <= next_node[1] < h):
                    continue
                    
                # Check obstacles (cost >= 250)
                cell_cost = costmap[next_node[1], next_node[0]]
                if cell_cost >= 250:
                    continue
                    
                move_cost = math.hypot(dx, dy)
                new_cost = cost_so_far[current] + move_cost + (cell_cost * 0.1) # Weight the cell cost
                
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current
                    
        # Reconstruct path
        path = []
        current = goal
        if current not in came_from:
            return [] # No path found
            
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        
        return path
