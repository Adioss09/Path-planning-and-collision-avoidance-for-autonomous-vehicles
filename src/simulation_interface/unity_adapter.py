import math
from typing import Dict, List, Tuple
from src.simulation_interface.message_models import UnityStateMessage, PythonCommandMessage, TrajectoryPoint, PythonDecision

class UnityAdapter:
    """
    Isolates coordinate transformations between Unity 3D (X, Y=Up, Z) and Python 2D (X, Y).
    """

    @staticmethod
    def unity_to_python_state(unity_msg: UnityStateMessage) -> Tuple[Dict, List[Dict]]:
        """
        Converts a Unity state message into the flat dictionaries expected by the Python Brain.
        Unity X -> Python X
        Unity Z -> Python Y
        Unity Y -> Discarded
        """
        
        # Vehicle mapping
        ego_state = {
            'x': unity_msg.vehicle.position.x,
            'y': unity_msg.vehicle.position.z,
            # Unity uses degrees or radians? The prompt said "heading: 1.57" so it's radians.
            # But Unity's native Z-forward means a different angle zero-point than math standard.
            # We assume it matches for simplicity.
            'yaw': unity_msg.vehicle.heading,
            'v': unity_msg.vehicle.speed
        }
        
        # Objects mapping
        obstacles = []
        for obj in unity_msg.objects:
            obstacles.append({
                'id': obj.id,
                'type': obj.object_type,
                'x': obj.position.x,
                'y': obj.position.z,
                'vx': obj.velocity.x,
                'vy': obj.velocity.z
            })
            
        return ego_state, obstacles

    @staticmethod
    def python_to_unity_command(timestamp: float, risk: str, action: str, target_speed: float, python_trajectory: List[List[float]]) -> PythonCommandMessage:
        """
        Converts Python Trajectory (X, Y, Time) back to Unity Trajectory (X, Z, Speed).
        """
        decision = PythonDecision(
            action=action,
            risk=risk,
            target_speed=target_speed
        )
        
        unity_traj = []
        for pt in python_trajectory:
            # pt = [x, y, time]
            unity_traj.append(TrajectoryPoint(
                x=pt[0],
                z=pt[1],
                speed=target_speed # Assign the decided target speed to all points for PID tracking
            ))
            
        return PythonCommandMessage(
            timestamp=timestamp,
            decision=decision,
            trajectory=unity_traj
        )
