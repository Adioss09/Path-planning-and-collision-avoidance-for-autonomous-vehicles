import pygame
import sys
import os
import math
import argparse
import time

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.prediction.tracker import Tracker
from src.prediction.trajectory_predictor import TrajectoryPredictor
from src.safety.risk import RiskAssessor
from src.decision.decision_engine import DecisionEngine
from src.planning.planner import AStarPlanner
from src.planning.path_smoother import PathSmoother
from src.vehicle.bicycle_model import KinematicBicycleModel
from src.evaluation.metrics import MetricsTracker
import numpy as np

# Pygame Colors
BG_COLOR = (40, 44, 52)
ROAD_COLOR = (70, 75, 85)
EGO_COLOR = (0, 150, 255)
PED_COLOR = (255, 180, 0)
CAR_COLOR = (220, 50, 50)
ANIMAL_COLOR = (139, 69, 19)
PATH_COLOR = (0, 255, 100)
OLD_PATH_COLOR = (0, 100, 50)
PRED_COLOR = (255, 150, 255)
TEXT_COLOR = (220, 220, 220)

class Simulation:
    def __init__(self, scenario_name):
        pygame.init()
        self.width, self.height = 1000, 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(f"SIH26037 Adaptive Navigation - {scenario_name.capitalize()}")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Consolas', 18)
        self.large_font = pygame.font.SysFont('Consolas', 24, bold=True)
        
        self.scenario_name = scenario_name
        self.setup_scenario()
        
        # Modules
        self.tracker = Tracker(max_distance=100)
        self.predictor = TrajectoryPredictor(horizon_seconds=2.0, timestep=0.1)
        self.risk_assessor = RiskAssessor()
        self.decision_engine = DecisionEngine()
        self.planner = AStarPlanner(grid_resolution=1.0)
        self.smoother = PathSmoother()
        self.metrics = MetricsTracker()
        
        self.metrics.start_scenario((self.vehicle.x, self.vehicle.y))
        
        self.active_path = []
        self.old_path = []
        self.current_risk = "LOW"
        self.current_action = "CRUISE"
        self.min_ttc = float('inf')
        self.sim_time = 0.0
        self.last_replan_time = 0.0
        
    def setup_scenario(self):
        # Default start and goal
        self.vehicle = KinematicBicycleModel(x=100, y=750, yaw=-math.pi/2, v=30.0)
        self.goal = (100, 50)
        self.obstacles = [] # list of dicts
        
        if self.scenario_name == 'market':
            self.obstacles = [
                {'x': 110.0, 'y': 500.0, 'vx': -10.0, 'vy': 5.0, 'type': 'pedestrian', 'id': 1},
                {'x': 80.0, 'y': 350.0, 'vx': 0.0, 'vy': -20.0, 'type': 'bicycle', 'id': 2},
                {'x': 130.0, 'y': 200.0, 'vx': -5.0, 'vy': 0.0, 'type': 'pushcart', 'id': 3}
            ]
        elif self.scenario_name == 'cattle':
            self.vehicle = KinematicBicycleModel(x=100, y=750, yaw=-math.pi/2, v=40.0) # Faster
            self.goal = (100, 50)
            # Cattle starts far off the road on the right, moves left fast when vehicle approaches
            self.obstacles = [
                {'x': 300.0, 'y': 400.0, 'vx': -35.0, 'vy': 0.0, 'type': 'animal', 'id': 1}
            ]
        elif self.scenario_name == 'intersection':
            self.goal = (500, 100)
            self.obstacles = [
                {'x': 50.0, 'y': 400.0, 'vx': 30.0, 'vy': 0.0, 'type': 'car', 'id': 1},
                {'x': 400.0, 'y': 400.0, 'vx': -25.0, 'vy': 0.0, 'type': 'car', 'id': 2},
            ]
        else: # unmarked
            self.goal = (100, 50)
            self.obstacles = [
                {'x': 105.0, 'y': 400.0, 'vx': 0.0, 'vy': -15.0, 'type': 'car', 'id': 1}
            ]
            
        for obs in self.obstacles:
            obs['history'] = []
            obs['predicted_trajectory'] = []
            
    def get_costmap(self):
        # 1000x800 costmap
        costmap = np.zeros((self.height, self.width), dtype=np.uint8)
        
        # Add road boundaries as high cost
        if self.scenario_name != 'intersection':
            costmap[:, :30] = 255
            costmap[:, 170:] = 255
            
        # Add dynamic obstacles to costmap (inflated)
        import cv2
        for obs in self.obstacles:
            ox, oy = int(obs['x']), int(obs['y'])
            if 0 <= ox < self.width and 0 <= oy < self.height:
                cv2.circle(costmap, (ox, oy), 35, 255, -1)
                
                # Also penalize the immediate predicted path
                for pt in obs['predicted_trajectory'][:10]:
                    cv2.circle(costmap, (int(pt[0]), int(pt[1])), 25, 200, -1)
                
        return costmap

    def get_ego_predicted_trajectory(self, dt):
        """Simple ego trajectory prediction based on current active path or current velocity"""
        traj = []
        if self.active_path:
            # Use active path
            t = self.sim_time
            for p in self.active_path[::5]:
                traj.append([p[0], p[1], t])
                t += 0.5
        else:
            # Linear extrapolation
            ex, ey = self.vehicle.x, self.vehicle.y
            vx = self.vehicle.v * math.cos(self.vehicle.yaw)
            vy = self.vehicle.v * math.sin(self.vehicle.yaw)
            t = self.sim_time
            for _ in range(20):
                ex += vx * 0.1
                ey += vy * 0.1
                t += 0.1
                traj.append([ex, ey, t])
        return traj

    def pure_pursuit(self):
        if not self.active_path:
            return 0.0, 0.0
            
        lookahead = 40.0
        
        # 1. Find closest point on path
        closest_idx = 0
        min_dist = float('inf')
        for i, p in enumerate(self.active_path):
            dist = math.hypot(p[0] - self.vehicle.x, p[1] - self.vehicle.y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        # 2. Find target point at least `lookahead` away, searching forward
        target_idx = closest_idx
        for i in range(closest_idx, len(self.active_path)):
            p = self.active_path[i]
            dist = math.hypot(p[0] - self.vehicle.x, p[1] - self.vehicle.y)
            if dist > lookahead:
                target_idx = i
                break
                
        # If we reached the end of the path
        if target_idx == len(self.active_path) - 1:
            # Stop if very close to goal
            goal_dist = math.hypot(self.goal[0] - self.vehicle.x, self.goal[1] - self.vehicle.y)
            if goal_dist < 20.0:
                self.current_action = 'STOP'
                return -10.0, 0.0
                
        target = self.active_path[target_idx]
            
        dx = target[0] - self.vehicle.x
        dy = target[1] - self.vehicle.y
        target_yaw = math.atan2(dy, dx)
        
        yaw_diff = target_yaw - self.vehicle.yaw
        
        # normalize
        while yaw_diff > math.pi: yaw_diff -= 2*math.pi
        while yaw_diff < -math.pi: yaw_diff += 2*math.pi
        
        steering = yaw_diff * 0.5 # Proportional control
        
        # Speed control
        if self.current_action == 'EMERGENCY_BRAKE':
            accel = -100.0 # Stop immediately
        elif self.current_action == 'SLOW_DOWN':
            accel = -10.0
        elif self.current_action == 'STOP':
            accel = -15.0
        else:
            accel = 5.0 if self.vehicle.v < 40.0 else 0.0
            
        return accel, steering

    def step(self, dt):
        self.sim_time += dt
        
        # Update obstacles
        for obs in self.obstacles:
            obs['x'] += obs['vx'] * dt
            obs['y'] += obs['vy'] * dt
            
            # Tracker integration: save history
            obs['history'].append([obs['x'], obs['y'], self.sim_time])
            if len(obs['history']) > 10:
                obs['history'].pop(0)
                
            # Predictor integration
            obs['predicted_trajectory'] = self.predictor.predict(obs['history'])
            
        # Ego trajectory prediction
        ego_traj = self.get_ego_predicted_trajectory(dt)
            
        # Risk & Decision
        ego_state = self.vehicle.get_state()
        ego_state['t'] = self.sim_time
        
        self.current_risk, self.min_ttc, min_clear = self.risk_assessor.assess_risk(ego_state, ego_traj, self.obstacles)
        self.current_action = self.decision_engine.decide(self.current_risk)
        
        is_replanning = False
        
        # Replanning logic
        # Replan if no active path, or if high risk and cooldown passed
        needs_replan = not self.active_path
        if self.current_action == 'REPLAN' and (self.sim_time - self.last_replan_time > 1.0):
            needs_replan = True
            
        if needs_replan:
            costmap = self.get_costmap()
            path = self.planner.plan(costmap, (self.vehicle.x, self.vehicle.y), self.goal)
            if path and len(path) > 2:
                # Only swap paths if it's actually different or we had no path
                self.old_path = self.active_path
                self.active_path = self.smoother.smooth(path, s=10.0)
                is_replanning = True
                self.last_replan_time = self.sim_time
                
        self.metrics.update((self.vehicle.x, self.vehicle.y), self.current_risk, is_replanning, min_clear)
        
        # Vehicle Control
        accel, steering = self.pure_pursuit()
        self.vehicle.update(accel, steering, dt)
        
    def draw_dashboard(self):
        panel_rect = pygame.Rect(self.width - 270, 0, 270, self.height)
        pygame.draw.rect(self.screen, (30, 32, 40), panel_rect)
        
        title = self.large_font.render("SYSTEM STATUS", True, TEXT_COLOR)
        self.screen.blit(title, (self.width - 250, 20))
        
        y = 70
        def draw_stat(label, value, color=TEXT_COLOR):
            nonlocal y
            text = self.font.render(f"{label}: {value}", True, color)
            self.screen.blit(text, (self.width - 250, y))
            y += 30
            
        draw_stat("Scenario", self.scenario_name.upper())
        draw_stat("Speed", f"{self.vehicle.v:.1f} px/s")
        
        risk_color = (0, 255, 0)
        if self.current_risk == 'CRITICAL': risk_color = (255, 0, 0)
        elif self.current_risk == 'HIGH': risk_color = (255, 100, 0)
        elif self.current_risk == 'MEDIUM': risk_color = (255, 200, 0)
        
        draw_stat("Risk Level", self.current_risk, risk_color)
        ttc_str = f"{self.min_ttc:.1f} s" if self.min_ttc != float('inf') else "INF"
        draw_stat("TTC", ttc_str)
        draw_stat("Action", self.current_action, risk_color)
        
        y += 20
        draw_stat("Replans", self.metrics.metrics['replanning_count'])
        draw_stat("Collisions", self.metrics.metrics['collision_count'])
        draw_stat("Clearance", f"{self.metrics.metrics['min_clearance']:.1f} px")
        
        goal_dist = math.hypot(self.goal[0] - self.vehicle.x, self.goal[1] - self.vehicle.y)
        draw_stat("Dist to Goal", f"{goal_dist:.1f} px")

    def render(self):
        self.screen.fill(BG_COLOR)
        
        # Draw road
        if self.scenario_name != 'intersection':
            pygame.draw.rect(self.screen, ROAD_COLOR, (30, 0, 140, self.height))
            
        # Draw old path
        if len(self.old_path) > 1:
            points = [(p[0], p[1]) for p in self.old_path]
            pygame.draw.lines(self.screen, OLD_PATH_COLOR, False, points, 2)
            
        # Draw active path
        if len(self.active_path) > 1:
            points = [(p[0], p[1]) for p in self.active_path]
            pygame.draw.lines(self.screen, PATH_COLOR, False, points, 4)
            
        # Draw Goal
        pygame.draw.circle(self.screen, (0, 255, 0), (int(self.goal[0]), int(self.goal[1])), 10, 2)
            
        # Draw Obstacles & Predictions
        for obs in self.obstacles:
            color = CAR_COLOR
            if obs['type'] == 'pedestrian': color = PED_COLOR
            elif obs['type'] == 'animal': color = ANIMAL_COLOR
            
            ox, oy = int(obs['x']), int(obs['y'])
            pygame.draw.circle(self.screen, color, (ox, oy), 15)
            
            # Label
            label = self.font.render(obs['type'], True, (255,255,255))
            self.screen.blit(label, (ox - 20, oy - 30))
            
            # Predicted Trajectory
            if len(obs['predicted_trajectory']) > 1:
                pts = [(pt[0], pt[1]) for pt in obs['predicted_trajectory']]
                pygame.draw.lines(self.screen, PRED_COLOR, False, pts, 2)
            
        # Draw Vehicle
        vx, vy = int(self.vehicle.x), int(self.vehicle.y)
        end_x = vx + int(25 * math.cos(self.vehicle.yaw))
        end_y = vy + int(25 * math.sin(self.vehicle.yaw))
        
        pygame.draw.circle(self.screen, EGO_COLOR, (vx, vy), 12)
        pygame.draw.line(self.screen, (255, 255, 255), (vx, vy), (end_x, end_y), 4)
        
        self.draw_dashboard()
        pygame.display.flip()
        
    def run(self):
        running = True
        dt = 0.05 # simulation step size
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
            self.step(dt)
            self.render()
            
            # Check goal reach
            if math.hypot(self.vehicle.x - self.goal[0], self.vehicle.y - self.goal[1]) < 25:
                print("Goal Reached!")
                self.metrics.finish_scenario(success=True)
                running = False
                
            self.clock.tick(30) # 30 FPS visual speed
            
        metrics_path = os.path.join(os.path.dirname(__file__), '..', 'results', 'metrics', f'{self.scenario_name}_metrics.json')
        os.makedirs(os.path.dirname(metrics_path), exist_ok=True)
        self.metrics.save(metrics_path)
        print(f"Metrics saved to {metrics_path}")
        pygame.quit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', type=str, default='cattle', choices=['unmarked', 'intersection', 'market', 'cattle'])
    args = parser.parse_args()
    
    sim = Simulation(args.scenario)
    sim.run()
