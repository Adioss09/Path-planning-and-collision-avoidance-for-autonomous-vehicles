import asyncio
import websockets
import json
import logging
import sys
import os
import math

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.simulation_interface.message_models import UnityStateMessage, PythonCommandMessage
from src.simulation_interface.unity_adapter import UnityAdapter
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
        self.adapter = UnityAdapter()
        
        # Provide default abstract lanes for Unity mapping (Unity road is centered at X=0)
        self.lanes = [{'id': 1, 'x': -3.5}, {'id': 2, 'x': 0.0}, {'id': 3, 'x': 3.5}]
        self.planner = CandidatePlanner(self.lanes)
        
        self.obstacle_histories = {}
        self.last_goal = (0.0, 200.0)

    def process_state(self, msg: UnityStateMessage) -> PythonCommandMessage:
        # 1. Coordinate Transformation
        ego_state, raw_obstacles = self.adapter.unity_to_python_state(msg)
        
        # 2. Tracking & Prediction
        obstacles = []
        for obj in raw_obstacles:
            obj_id = obj['id']
            if obj_id not in self.obstacle_histories:
                self.obstacle_histories[obj_id] = []
                
            self.obstacle_histories[obj_id].append([obj['x'], obj['y'], msg.timestamp])
            if len(self.obstacle_histories[obj_id]) > 10:
                self.obstacle_histories[obj_id].pop(0)
                
            pred_traj = self.predictor.predict(self.obstacle_histories[obj_id])
            obj['history'] = self.obstacle_histories[obj_id]
            obj['predicted_trajectory'] = pred_traj
            obstacles.append(obj)

        # 3. Risk Assessment
        ego_traj = []
        for i in range(10):
            ego_traj.append([ego_state['x'], ego_state['y'] + (ego_state['v'] * i * 0.1), msg.timestamp + (i * 0.1)])
            
        risk_level, min_ttc, min_clear = self.risk_assessor.assess_risk(ego_state, ego_traj, obstacles)
        
        # 4. Decision Engine
        action = self.decision_engine.decide(risk_level)
        target_speed = ego_state['v']
        if action == 'EMERGENCY_BRAKE': target_speed = max(0, target_speed - 100 * 0.1)
        elif action == 'SLOW_DOWN': target_speed = max(0, target_speed - 15 * 0.1)
        elif action == 'CRUISE': target_speed = min(40.0, target_speed + 5 * 0.1)
        elif action == 'STOP': target_speed = 0.0
        
        # 5. Path Planning
        scenario_id = msg.scenario.id if msg.scenario else "cattle"
        
        if scenario_id == "urban_intersection":
            cands = self.planner.generate_intersection_candidates(ego_state['x'], ego_state['y'], ego_state['v'], msg.timestamp)
        else:
            cands = self.planner.generate_candidates(ego_state['x'], ego_state['y'], ego_state['v'], msg.timestamp, self.last_goal[0], self.last_goal[1])
            
        evaluated = self.planner.evaluate_candidates(cands, obstacles)
        best_cand = min(evaluated, key=lambda c: c['cost'])
        
        # 6. Transform back to Unity Payload
        return self.adapter.python_to_unity_command(
            timestamp=msg.timestamp,
            risk=risk_level,
            action=action,
            target_speed=target_speed,
            python_trajectory=best_cand['path']
        )

    async def handler(self, websocket):
        logger.info(f"Unity client connected from {websocket.remote_address}")
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    state_msg = UnityStateMessage(**data)
                    response_msg = self.process_state(state_msg)
                    await websocket.send(response_msg.model_dump_json())
                    
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    logger.error(f"Error processing message: {e}")
                    err_payload = {"error": str(e), "decision": {"action": "EMERGENCY_BRAKE", "risk": "CRITICAL", "target_speed": 0.0}}
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
            await asyncio.Future()

if __name__ == "__main__":
    server = UnityBrainServer()
    try:
        asyncio.run(server.serve())
    except KeyboardInterrupt:
        logger.info("Server manually stopped.")
