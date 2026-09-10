from pydantic import BaseModel, Field
from typing import List, Optional

# ==========================================
# UNITY -> PYTHON (INCOMING STATE)
# ==========================================

class Vector3(BaseModel):
    x: float
    y: float
    z: float

class UnityVehicleState(BaseModel):
    id: str = "ego"
    position: Vector3
    heading: float
    speed: float

class UnityObjectVelocity(BaseModel):
    x: float
    z: float

class UnityObject(BaseModel):
    id: str
    object_type: str = Field(alias="type")  # mapped to 'type' in JSON
    position: Vector3
    velocity: UnityObjectVelocity

class UnityScenario(BaseModel):
    id: str
    time: float

class UnityStateMessage(BaseModel):
    timestamp: float
    scenario: Optional[UnityScenario] = None
    vehicle: UnityVehicleState
    objects: List[UnityObject]

# ==========================================
# PYTHON -> UNITY (OUTGOING COMMANDS)
# ==========================================

class TrajectoryPoint(BaseModel):
    x: float
    z: float
    speed: float

class PythonDecision(BaseModel):
    action: str
    risk: str
    target_speed: float

class PythonCommandMessage(BaseModel):
    timestamp: float
    decision: PythonDecision
    trajectory: List[TrajectoryPoint]
