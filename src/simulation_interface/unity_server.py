import asyncio
import websockets
import json
import logging
import sys
import os

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.simulation_interface.message_models import UnityStateMessage, PythonCommandMessage, TrajectoryPoint
from src.prediction.trajectory_predictor import TrajectoryPredictor
from src.safety.risk import RiskAssessor
from src.decision.decision_engine import DecisionEngine
from src.planning.candidate_planner import CandidatePlanner

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger("UnityServer")

class UnityBrainServer:
    def __init__(self, host="0.0.0.0", port=8765):
        self.host = host
        self.port = port
        
        # Initialize the AI Brain modules
        self.predictor = TrajectoryPredictor(horizon_seconds=2.0, timestep=0.1)
        self.risk_assessor = RiskAssessor()
        self.decision_engine = DecisionEngine()
        
        # Provide default lanes. In a full system, Unity might send road boundaries.
        self.lanes = [{'id': 1, 'x': 60.0}, {'id': 2, 'x': 120.0}, {'id': 3, 'x': 180.0}]
        self.planner = CandidatePlanner(self.lanes)
        
        self.obstacle_histories = {}
        self.last_goal = (120.0, 50.0) # Abstract global goal

    def process_state(self, msg: UnityStateMessage) -> PythonCommandMessage:
        """Core pipeline: Perception (Unity) -> Prediction -> Risk -> Decision -> Planning"""
        
        # 1. Formatting Objects & Tracking History
        obstacles = []
        for obj in msg.objects:
            if obj.id not in self.obstacle_histories:
                self.obstacle_histories[obj.id] = []
                
            self.obstacle_histories[obj.id].append([obj.x, obj.y, msg.timestamp])
            # Keep last 10 points
            if len(self.obstacle_histories[obj.id]) > 10:
                self.obstacle_histories[obj.id].pop(0)
                
            # Predict trajectory
            pred_traj = self.predictor.predict(self.obstacle_histories[obj.id])
            
            obstacles.append({
                'id': obj.id,
                'type': obj.object_class,
                'x': obj.x,
                'y': obj.y,
                'vx': obj.vx,
                'vy': obj.vy,
                'history': self.obstacle_histories[obj.id],
                'predicted_trajectory': pred_traj
            })

        # 2. Risk Assessment
        ego_state = {
            'x': msg.vehicle.x,
            'y': msg.vehicle.y,
            'yaw': msg.vehicle.heading,
            'v': msg.vehicle.speed
        }
        
        # Simplistic ego trajectory projection for risk assessment
        ego_traj = []
        for i in range(10):
            ego_traj.append([msg.vehicle.x, msg.vehicle.y - (msg.vehicle.speed * i * 0.1), msg.timestamp + (i * 0.1)])
            
        risk_level, min_ttc, min_clear = self.risk_assessor.assess_risk(ego_state, ego_traj, obstacles)
        
        # 3. Decision Engine
        action = self.decision_engine.decide(risk_level)
        target_speed = msg.vehicle.speed
        if action == 'EMERGENCY_BRAKE': target_speed = max(0, target_speed - 100 * 0.1)
        elif action == 'SLOW_DOWN': target_speed = max(0, target_speed - 15 * 0.1)
        elif action == 'CRUISE': target_speed = min(40.0, target_speed + 5 * 0.1)
        elif action == 'STOP': target_speed = 0.0
        
        # 4. Path Planning
        cands = self.planner.generate_candidates(msg.vehicle.x, msg.vehicle.y, msg.vehicle.speed, msg.timestamp, self.last_goal[0], self.last_goal[1])
        evaluated = self.planner.evaluate_candidates(cands, obstacles)
        
        best_cand = min(evaluated, key=lambda c: c['cost'])
        
        trajectory_out = []
        for p in best_cand['path']:
            trajectory_out.append(TrajectoryPoint(x=p[0], y=p[1], time=p[2], speed=target_speed))
            
        # 5. Build Response
        return PythonCommandMessage(
            timestamp=msg.timestamp,
            risk=risk_level,
            action=action,
            target_speed=target_speed,
            trajectory=trajectory_out
        )

    async def handler(self, websocket):
        logger.info(f"Unity client connected from {websocket.remote_address}")
        try:
            async for message in websocket:
                try:
                    # Parse JSON strictly via Pydantic
                    data = json.loads(message)
                    state_msg = UnityStateMessage(**data)
                    
                    # Run Pipeline
                    response_msg = self.process_state(state_msg)
                    
                    # Send JSON back
                    await websocket.send(response_msg.model_dump_json())
                    
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
                    # Send fallback error payload
                    err_payload = {"error": str(e), "action": "EMERGENCY_BRAKE"}
                    await websocket.send(json.dumps(err_payload))
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("Unity client disconnected naturally.")
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            logger.info("Connection handler terminated.")

    async def serve(self):
        logger.info(f"Starting Unity Python Brain Server on ws://{self.host}:{self.port}")
        async with websockets.serve(self.handler, self.host, self.port):
            await asyncio.Future()  # run forever

if __name__ == "__main__":
    server = UnityBrainServer()
    try:
        asyncio.run(server.serve())
    except KeyboardInterrupt:
        logger.info("Server manually stopped.")
