from pydantic import BaseModel, Field
from typing import List, Optional

# ==========================================
# UNITY -> PYTHON (INCOMING STATE)
# ==========================================

class UnityVehicleState(BaseModel):
    x: float
    y: float
    heading: float
    speed: float

class UnityObject(BaseModel):
    id: str
    object_class: str = Field(alias="class")  # mapped to 'class' in JSON
    x: float
    y: float
    vx: float
    vy: float

class UnityStateMessage(BaseModel):
    timestamp: float
    scenario_id: Optional[str] = None
    vehicle: UnityVehicleState
    objects: List[UnityObject]

# ==========================================
# PYTHON -> UNITY (OUTGOING COMMANDS)
# ==========================================

class TrajectoryPoint(BaseModel):
    x: float
    y: float
    speed: float
    time: float

class PythonCommandMessage(BaseModel):
    timestamp: float
    risk: str
    action: str
    target_speed: float
    trajectory: List[TrajectoryPoint]
