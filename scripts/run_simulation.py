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
from src.planning.candidate_planner import CandidatePlanner
from src.vehicle.bicycle_model import KinematicBicycleModel
from src.evaluation.metrics import MetricsTracker

# Pygame Colors
BG_COLOR = (40, 44, 52)
ROAD_COLOR = (70, 75, 85)
LANE_COLOR = (150, 150, 150)
EGO_COLOR = (0, 150, 255)
PED_COLOR = (255, 180, 0)
CAR_COLOR = (220, 50, 50)
ANIMAL_COLOR = (139, 69, 19)
PATH_COLOR = (0, 255, 100)
OLD_PATH_COLOR = (0, 100, 50)
CANDIDATE_SAFE = (100, 100, 100)
CANDIDATE_BLOCKED = (200, 50, 50)
PRED_COLOR = (255, 150, 255)
TEXT_COLOR = (220, 220, 220)
MARKER_COLOR = (255, 200, 0)

class Simulation:
    def __init__(self, scenario_name, debug=False):
        pygame.init()
        self.width, self.height = 1000, 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(f"SIH26037 Multi-Lane Adaptive Navigation - {scenario_name.capitalize()}")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Consolas', 15)
        self.large_font = pygame.font.SysFont('Consolas', 20, bold=True)
        self.debug_font = pygame.font.SysFont('Consolas', 11)
        
        self.scenario_name = scenario_name
        self.debug = debug
        self.lanes = [{'id': 1, 'x': 60.0}, {'id': 2, 'x': 120.0}, {'id': 3, 'x': 180.0}]
        
        # Single World Coordinate Goal
        self.goal = (120.0, 50.0) 
        
        self.setup_scenario()
        
        # Modules
        self.tracker = Tracker(max_distance=100)
        self.predictor = TrajectoryPredictor(horizon_seconds=2.0, timestep=0.1)
        self.risk_assessor = RiskAssessor()
        self.decision_engine = DecisionEngine()
        self.candidate_planner = CandidatePlanner(self.lanes)
        self.metrics = MetricsTracker()
        
        self.metrics.start_scenario((self.vehicle.x, self.vehicle.y))
        
        self.active_path = []
        self.historical_paths = [] # Frozen exact copies of previous active paths
        self.all_candidates = []
        
        self.current_risk = "LOW"
        self.current_action = "CRUISE"
        self.min_ttc = float('inf')
        self.sim_time = 0.0
        self.last_eval_time = -5.0
        
        self.current_lane = 2
        self.target_lane = 'STRAIGHT' if self.scenario_name == 'intersection' else 2
        self.safe_alts_str = "NONE"
        self.replan_reason = "Initial"
        
        # Logger Setup
        self.log_dir = os.path.join(os.path.dirname(__file__), '..', 'results', 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, 'path_change_events.log')
        with open(self.log_file, 'w') as f:
            f.write(f"SIH26037 PATH CHANGE EVENTS LOG - {scenario_name.upper()}\n")
            f.write("="*60 + "\n")
            
        self.event_count = 0
        self.semantic_path_count = 1
        self.active_path_id = f"PATH_{self.semantic_path_count:03d}"
        self.prev_path_id = "NONE"
        self.replanned_markers = [] # list of (x, y, event_id)
        
    def setup_scenario(self):
        self.vehicle = KinematicBicycleModel(x=120.0, y=750.0, yaw=-math.pi/2, v=40.0)
        self.obstacles = []
        
        if self.scenario_name == 'market':
            self.obstacles = [
                {'x': 60.0, 'y': 500.0, 'vx': 0.0, 'vy': 5.0, 'type': 'pedestrian', 'id': 'PED #01'},
                {'x': 120.0, 'y': 350.0, 'vx': 0.0, 'vy': -10.0, 'type': 'bicycle', 'id': 'BIKE #02'},
                {'x': 180.0, 'y': 200.0, 'vx': 0.0, 'vy': -5.0, 'type': 'pushcart', 'id': 'CART #03'}
            ]
        elif self.scenario_name == 'cattle':
            self.obstacles = [
                {'x': 300.0, 'y': 450.0, 'vx': -35.0, 'vy': 0.0, 'type': 'animal', 'id': 'CATTLE #01'}
            ]
        elif self.scenario_name == 'intersection':
            self.vehicle = KinematicBicycleModel(x=566.0, y=750.0, yaw=-math.pi/2, v=40.0)
            self.goal = (566.0, 50.0) # Primary goal
            
            self.obstacles = [
                # Crossing West to East (Horizontal bottom lane)
                {'x': -50.0, 'y': 466.0, 'vx': 38.0, 'vy': 0.0, 'type': 'auto', 'id': 'AUTO #01'},
                {'x': 150.0, 'y': 433.0, 'vx': 42.0, 'vy': 0.0, 'type': 'car', 'id': 'CAR #02'},
                
                # Crossing East to West (Horizontal top lane)
                {'x': 950.0, 'y': 333.0, 'vx': -45.0, 'vy': 0.0, 'type': 'car', 'id': 'CAR #03'},
                {'x': 800.0, 'y': 366.0, 'vx': -35.0, 'vy': 0.0, 'type': 'bike', 'id': 'BIKE #04'},
                
                # North to South (Vertical left lane)
                {'x': 433.0, 'y': -50.0, 'vx': 0.0, 'vy': 40.0, 'type': 'car', 'id': 'CAR #05'},
                {'x': 466.0, 'y': 150.0, 'vx': 0.0, 'vy': 35.0, 'type': 'bike', 'id': 'BIKE #06'},
                
                # Pedestrians near intersection
                {'x': 400.0, 'y': 250.0, 'vx': 12.0, 'vy': 0.0, 'type': 'pedestrian', 'id': 'PED #07'},
                {'x': 600.0, 'y': 550.0, 'vx': -10.0, 'vy': 0.0, 'type': 'pedestrian', 'id': 'PED #08'},
            ]
        else: # unmarked
            self.obstacles = [
                {'x': 120.0, 'y': 400.0, 'vx': 0.0, 'vy': -20.0, 'type': 'car', 'id': 'CAR #01'}
            ]
            
        for obs in self.obstacles:
            obs['history'] = []
            obs['predicted_trajectory'] = []

    def get_ego_predicted_trajectory(self, dt):
        traj = []
        if self.active_path:
            t = self.sim_time
            for p in self.active_path[::5]:
                traj.append([p[0], p[1], t])
                t += 0.5
        else:
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
        
    def log_path_change(self, prev_lane, new_lane, reason, clearance, latency):
        self.event_count += 1
        self.semantic_path_count += 1
        new_path_id = f"PATH_{self.semantic_path_count:03d}"
        
        log_str = (
            f"PATH CHANGE EVENT #{self.event_count}\n"
            f"{'-'*30}\n"
            f"Time:              {self.sim_time:.2f} s\n"
            f"Position:          ({self.vehicle.x:.1f}, {self.vehicle.y:.1f})\n"
            f"Previous Path:     {self.active_path_id}\n"
            f"New Path:          {new_path_id}\n"
            f"Previous Lane:     {prev_lane}\n"
            f"New Lane:          {new_lane}\n"
            f"Trigger:           {reason}\n"
            f"Risk:              {self.current_risk}\n"
            f"TTC:               {self.min_ttc:.2f} s\n"
            f"Clearance:         {clearance:.1f} px\n"
            f"Speed:             {self.vehicle.v:.1f} px/s\n"
            f"Candidates:        {len(self.all_candidates)}\n"
            f"Replanning Latency:{latency:.1f} ms\n"
            f"{'-'*30}\n\n"
        )
        
        with open(self.log_file, 'a') as f:
            f.write(log_str)
            
        self.prev_path_id = self.active_path_id
        self.active_path_id = new_path_id

    def pure_pursuit(self):
        if not self.active_path:
            return 0.0, 0.0
            
        lookahead = 40.0
        
        closest_idx = 0
        min_dist = float('inf')
        for i, p in enumerate(self.active_path):
            dist = math.hypot(p[0] - self.vehicle.x, p[1] - self.vehicle.y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        target_idx = closest_idx
        for i in range(closest_idx, len(self.active_path)):
            p = self.active_path[i]
            dist = math.hypot(p[0] - self.vehicle.x, p[1] - self.vehicle.y)
            if dist > lookahead:
                target_idx = i
                break
                
        # If we reached the end of the path (which is the mathematical goal)
        if target_idx == len(self.active_path) - 1:
            if self.scenario_name == 'intersection':
                goal_dist = math.hypot(self.active_path[-1][0] - self.vehicle.x, self.active_path[-1][1] - self.vehicle.y)
            else:
                goal_dist = math.hypot(self.goal[0] - self.vehicle.x, self.goal[1] - self.vehicle.y)
                
            if goal_dist < 40.0:
                self.current_action = 'STOP'
                return -20.0, 0.0
                
        target = self.active_path[target_idx]
            
        dx = target[0] - self.vehicle.x
        dy = target[1] - self.vehicle.y
        target_yaw = math.atan2(dy, dx)
        
        yaw_diff = target_yaw - self.vehicle.yaw
        while yaw_diff > math.pi: yaw_diff -= 2*math.pi
        while yaw_diff < -math.pi: yaw_diff += 2*math.pi
        
        steering = yaw_diff * 0.5
        
        if self.current_action == 'EMERGENCY_BRAKE':
            accel = -100.0
        elif self.current_action == 'SLOW_DOWN':
            accel = -15.0
        elif self.current_action == 'STOP':
            accel = -20.0
        else:
            accel = 5.0 if self.vehicle.v < 40.0 else 0.0
            
        return accel, steering

    def check_invariants(self):
        """Mathematically verifies that the vehicle and paths obey physical bounds."""
        if self.scenario_name == 'intersection':
            return # Different bounds for full 2D area
            
        # Vehicle must be within the road (30 to 210, with 15px margin)
        assert 15.0 <= self.vehicle.x <= 225.0, f"Vehicle left drivable road! x={self.vehicle.x:.1f}"
        
        if self.active_path:
            # Active path must start exactly at vehicle's last eval position
            start_dist = math.hypot(self.active_path[0][0] - self.vehicle.x, self.active_path[0][1] - self.vehicle.y)
            assert start_dist < 20.0, f"Path disconnected from vehicle! start_dist={start_dist:.1f}"
            
            # Active path must end exactly at goal
            end_dist = math.hypot(self.active_path[-1][0] - self.goal[0], self.active_path[-1][1] - self.goal[1])
            assert end_dist < 5.0, f"Path does not terminate at Goal! end_dist={end_dist:.1f}"

    def step(self, dt):
        self.sim_time += dt
        
        # Update obstacles
        for obs in self.obstacles:
            obs['x'] += obs['vx'] * dt
            obs['y'] += obs['vy'] * dt
            
            obs['history'].append([obs['x'], obs['y'], self.sim_time])
            if len(obs['history']) > 10:
                obs['history'].pop(0)
                
            obs['predicted_trajectory'] = self.predictor.predict(obs['history'])
            
        ego_traj = self.get_ego_predicted_trajectory(dt)
        self.current_risk, self.min_ttc, min_clear = self.risk_assessor.assess_risk(self.vehicle.get_state(), ego_traj, self.obstacles)
        self.current_action = self.decision_engine.decide(self.current_risk)
        
        is_replanning = False
        if self.scenario_name != 'intersection':
            self.current_lane = min(self.lanes, key=lambda l: abs(l['x'] - self.vehicle.x))['id']
        else:
            self.current_lane = self.target_lane # Abstract routing
        
        # Generate paths exactly from vehicle state to global goal
        if self.sim_time - self.last_eval_time > 0.2:
            t_start = time.time()
            if self.scenario_name == 'intersection':
                cands = self.candidate_planner.generate_intersection_candidates(
                    self.vehicle.x, self.vehicle.y, self.vehicle.v, self.sim_time
                )
            else:
                cands = self.candidate_planner.generate_candidates(
                    self.vehicle.x, self.vehicle.y, self.vehicle.v, self.sim_time,
                    self.goal[0], self.goal[1], num_points=60
                )
                
            self.all_candidates = self.candidate_planner.evaluate_candidates(cands, self.obstacles)
            latency = (time.time() - t_start) * 1000.0
            
            safe_lanes = [str(c['lane_id']) for c in self.all_candidates if c['safe']]
            self.safe_alts_str = ", ".join(safe_lanes) if safe_lanes else "NONE"
            
            best_cand = min(self.all_candidates, key=lambda c: c['cost'])
            
            if not best_cand['safe']:
                self.current_action = 'EMERGENCY_BRAKE'
                self.replan_reason = "ALL PATHS BLOCKED"
            else:
                if best_cand['lane_id'] != self.target_lane:
                    # GENUINE REPLAN
                    # Freeze the exact geometry of the old active path to history
                    if self.active_path:
                        self.historical_paths.append(self.active_path.copy())
                        
                    self.replan_reason = "Dynamic Obstacle Avoidance"
                    self.log_path_change(self.target_lane, best_cand['lane_id'], "Obstacle Collision Risk", min_clear, latency)
                    self.target_lane = best_cand['lane_id']
                    if self.scenario_name == 'intersection':
                        # Update global visual goal to match chosen route end
                        self.goal = (best_cand['path'][-1][0], best_cand['path'][-1][1])
                        
                    is_replanning = True
                    self.replanned_markers.append((self.vehicle.x, self.vehicle.y, self.event_count))
                    
                # Always adopt the new candidate so it remains strictly attached to vehicle
                self.active_path = best_cand['path']
                
            self.last_eval_time = self.sim_time
                
        self.metrics.update((self.vehicle.x, self.vehicle.y), self.current_risk, is_replanning, min_clear)
        
        accel, steering = self.pure_pursuit()
        self.vehicle.update(accel, steering, dt)
        
        self.check_invariants()
        
    def draw_dashboard(self):
        panel_rect = pygame.Rect(self.width - 250, 0, 250, self.height)
        pygame.draw.rect(self.screen, (30, 32, 40), panel_rect)
        
        title = self.large_font.render("SYSTEM STATUS", True, TEXT_COLOR)
        self.screen.blit(title, (self.width - 230, 20))
        
        y = 60
        def draw_stat(label, value, color=TEXT_COLOR):
            nonlocal y
            text = self.font.render(f"{label}: {value}", True, color)
            self.screen.blit(text, (self.width - 230, y))
            y += 25
            
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
        
        y += 15
        
        if self.scenario_name == 'intersection':
            crossing = sum(1 for obs in self.obstacles if abs(obs['x'] - self.vehicle.x) < 200 and abs(obs['y'] - self.vehicle.y) < 200)
            draw_stat("Current Route", self.target_lane, PATH_COLOR)
            draw_stat("Nearby Agents", len(self.obstacles))
            draw_stat("Crossing Agents", crossing, (255,100,0) if crossing > 0 else (0,255,0))
            draw_stat("Safe Routes", len(self.safe_alts_str.split(', ')))
        else:
            draw_stat("Active Path", self.active_path_id, PATH_COLOR)
            draw_stat("Prev Path", self.prev_path_id, OLD_PATH_COLOR)
            draw_stat("Current Lane", self.current_lane)
            draw_stat("Target Lane", self.target_lane)
            draw_stat("Safe Alts", self.safe_alts_str, (0, 255, 100) if self.safe_alts_str != "NONE" else (255, 0, 0))
            
        draw_stat("Reason", self.replan_reason)
        
        y += 15
        draw_stat("Replans", self.event_count)
        draw_stat("Collisions", self.metrics.metrics['collision_count'])
        if self.active_path:
            goal_dist = math.hypot(self.active_path[-1][0] - self.vehicle.x, self.active_path[-1][1] - self.vehicle.y)
        else:
            goal_dist = 0.0
        draw_stat("Dist to Goal", f"{goal_dist:.1f} px")
        
        if self.debug:
            y += 30
            draw_stat("DEBUG MODE", "ACTIVE", (255,200,0))
            draw_stat("Vehicle X", f"{self.vehicle.x:.1f}")
            draw_stat("Vehicle Y", f"{self.vehicle.y:.1f}")
            draw_stat("Goal X", f"{self.goal[0]:.1f}")
            draw_stat("Goal Y", f"{self.goal[1]:.1f}")

    def render(self):
        self.screen.fill(BG_COLOR)
        
        # 1. Road Geometry
        if self.scenario_name == 'intersection':
            # Vertical Road
            pygame.draw.rect(self.screen, ROAD_COLOR, (400, 0, 200, self.height))
            # Horizontal Road
            pygame.draw.rect(self.screen, ROAD_COLOR, (0, 300, self.width - 250, 200))
            
            # Intersection Center Square
            pygame.draw.rect(self.screen, ROAD_COLOR, (400, 300, 200, 200))
            
            # Draw Lane Dividers for Intersection
            # Vertical dividers (N/S)
            for y_line in range(0, self.height, 40):
                if not (300 < y_line < 500): # Don't draw dashed inside junction
                    pygame.draw.line(self.screen, LANE_COLOR, (500, y_line), (500, y_line + 20), 2)
            # Horizontal dividers (E/W)
            for x_line in range(0, self.width - 250, 40):
                if not (400 < x_line < 600):
                    pygame.draw.line(self.screen, LANE_COLOR, (x_line, 400), (x_line + 20, 400), 2)
        else:
            pygame.draw.rect(self.screen, ROAD_COLOR, (30, 0, 180, self.height))
            for i in range(1, 3):
                lx = 30 + (i * 60)
                for y_line in range(0, self.height, 40):
                    pygame.draw.line(self.screen, LANE_COLOR, (lx, y_line), (lx, y_line + 20), 2)
            
        # 2. Paths
        if self.debug:
            for cand in self.all_candidates:
                if len(cand['path']) > 1:
                    pts = [(p[0], p[1]) for p in cand['path']]
                    color = CANDIDATE_SAFE if cand['safe'] else CANDIDATE_BLOCKED
                    pygame.draw.lines(self.screen, color, False, pts, 1)
                
        # Draw Historical Paths (Dashed)
        for hist_path in self.historical_paths:
            if len(hist_path) > 1:
                for i in range(0, len(hist_path) - 1, 2):
                    p1 = (hist_path[i][0], hist_path[i][1])
                    p2 = (hist_path[i+1][0], hist_path[i+1][1])
                    pygame.draw.line(self.screen, OLD_PATH_COLOR, p1, p2, 2)
            
        # Draw active path (Solid, always connected to vehicle)
        if len(self.active_path) > 1:
            points = [(p[0], p[1]) for p in self.active_path]
            pygame.draw.lines(self.screen, PATH_COLOR, False, points, 4)
            if self.debug:
                for px, py, _ in self.active_path[::5]:
                    pygame.draw.circle(self.screen, PATH_COLOR, (int(px), int(py)), 3)
            
        # 3. Replanned Markers
        for mx, my, event_id in self.replanned_markers:
            pygame.draw.circle(self.screen, MARKER_COLOR, (int(mx), int(my)), 6)
            marker_text = self.font.render(f"REPLAN #{event_id}", True, MARKER_COLOR)
            y_offset = (event_id % 3) * -15
            self.screen.blit(marker_text, (int(mx) + 12, int(my) - 10 + y_offset))
            if self.debug:
                dbg_txt = self.debug_font.render(f"({mx:.1f}, {my:.1f})", True, MARKER_COLOR)
                self.screen.blit(dbg_txt, (int(mx) + 12, int(my) + 5 + y_offset))
            
        # 4. Global Fixed Goal
        pygame.draw.circle(self.screen, (0, 255, 0), (int(self.goal[0]), int(self.goal[1])), 10, 2)
        if self.debug:
            dbg_txt = self.debug_font.render(f"GOAL ({self.goal[0]}, {self.goal[1]})", True, (0, 255, 0))
            self.screen.blit(dbg_txt, (int(self.goal[0]) + 15, int(self.goal[1])))
            
        # 5. Obstacles & Predictions
        for obs in self.obstacles:
            color = CAR_COLOR
            if obs['type'] == 'pedestrian': color = PED_COLOR
            elif obs['type'] == 'animal': color = ANIMAL_COLOR
            
            ox, oy = int(obs['x']), int(obs['y'])
            pygame.draw.circle(self.screen, color, (ox, oy), 15)
            
            # Avoid overlapping text by putting it below
            label = self.font.render(obs['id'], True, (255,255,255))
            self.screen.blit(label, (ox - 30, oy + 20))
            
            if len(obs['predicted_trajectory']) > 1:
                pts = [(pt[0], pt[1]) for pt in obs['predicted_trajectory']]
                pygame.draw.lines(self.screen, PRED_COLOR, False, pts, 2)
            
        # 6. Vehicle
        vx, vy = int(self.vehicle.x), int(self.vehicle.y)
        end_x = vx + int(25 * math.cos(self.vehicle.yaw))
        end_y = vy + int(25 * math.sin(self.vehicle.yaw))
        
        pygame.draw.circle(self.screen, EGO_COLOR, (vx, vy), 12)
        pygame.draw.line(self.screen, (255, 255, 255), (vx, vy), (end_x, end_y), 4)
        
        self.draw_dashboard()
        pygame.display.flip()
        
    def run(self):
        running = True
        dt = 0.05
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_d:
                        self.debug = not self.debug
                        print(f"Debug Mode: {'ON' if self.debug else 'OFF'}")
                    
            self.step(dt)
            self.render()
            
            # Goal reached check
            if self.active_path:
                goal_dist = math.hypot(self.active_path[-1][0] - self.vehicle.x, self.active_path[-1][1] - self.vehicle.y)
                if goal_dist < 40.0:
                    print(f"Goal Reached! Replans: {self.event_count}")
                    self.metrics.finish_scenario(success=True)
                    running = False
                
            self.clock.tick(30)
            
        metrics_path = os.path.join(os.path.dirname(__file__), '..', 'results', 'metrics', f'{self.scenario_name}_metrics.json')
        os.makedirs(os.path.dirname(metrics_path), exist_ok=True)
        self.metrics.save(metrics_path)
        print(f"Metrics saved to {metrics_path}")
        pygame.quit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', type=str, default='cattle', choices=['unmarked', 'intersection', 'market', 'cattle'])
    parser.add_argument('--debug', action='store_true', help="Enable visual debug assertions")
    args = parser.parse_args()
    
    sim = Simulation(args.scenario, debug=args.debug)
    sim.run()
