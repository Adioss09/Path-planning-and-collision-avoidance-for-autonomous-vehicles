# Unity Integration Guide: SIH26037 Autonomous Brain

This document explains exactly how to integrate the Unity simulation environment with the Python Autonomous Navigation "Brain". 

**Important Design Principle:** Unity is strictly responsible for rendering the environment, simulating sensors (or exporting bounding boxes), and moving the ego vehicle smoothly along the provided trajectory. **Unity does NOT make driving decisions or calculate paths.** All tracking, trajectory prediction, collision-risk assessment, and path planning occurs dynamically inside the Python server.

## 1. Connection Details

- **Protocol:** WebSocket (ws)
- **Address:** `ws://localhost:8765` (or the IP of the machine running Python)
- **Format:** JSON strings.
- **Update Frequency:** Unity should ideally send state updates at ~10 Hz (every 0.1s). The Python server instantly responds to each message with a corresponding action and trajectory.

## 2. Coordinate System Assumptions

- Python operates in a 2D Top-Down Cartesian system `(X, Y)`.
- If Unity uses standard 3D `(X, Y, Z)` where `Y` is Up, you must map your coordinates before sending them to Python:
  - Python `X` = Unity `X`
  - Python `Y` = Unity `Z`
- **Units:** Meters and Meters/Second (m/s).
- **Heading/Yaw:** Radians.

---

## 3. JSON Protocol Schemas

### A. Unity → Python (Incoming State)

Every frame/tick, Unity must collect the Ego vehicle state and a list of all dynamic objects, and send them to Python.

```json
{
  "timestamp": 123.45,
  "scenario_id": "cattle",
  "vehicle": {
    "x": 120.0,
    "y": 500.0,
    "heading": -1.57,
    "speed": 15.0
  },
  "objects": [
    {
      "id": "CATTLE_01",
      "class": "animal",
      "x": 100.0,
      "y": 450.0,
      "vx": 2.5,
      "vy": 0.0
    },
    {
      "id": "CAR_02",
      "class": "car",
      "x": 60.0,
      "y": 400.0,
      "vx": 0.0,
      "vy": -10.0
    }
  ]
}
```

*Note: `class` can be one of: `car`, `bus`, `truck`, `auto`, `bike`, `pedestrian`, `animal`, `cart`.*

### B. Python → Unity (Outgoing Commands)

Immediately upon receiving the state, Python calculates the Risk, decides an Action, generates an adaptive collision-free path, and sends the following response:

```json
{
  "timestamp": 123.45,
  "risk": "HIGH",
  "action": "SLOW_DOWN",
  "target_speed": 5.0,
  "trajectory": [
    {"x": 120.0, "y": 498.0, "speed": 14.5, "time": 123.55},
    {"x": 120.0, "y": 496.0, "speed": 14.0, "time": 123.65},
    {"x": 110.0, "y": 480.0, "speed": 10.0, "time": 124.00}
  ]
}
```

#### Field Explanations:
- **`risk`**: The assessed collision risk (`LOW`, `MEDIUM`, `HIGH`, `CRITICAL`). Can be used to change UI colors in Unity.
- **`action`**: The decided state (`CRUISE`, `SLOW_DOWN`, `EMERGENCY_BRAKE`, `STOP`).
- **`target_speed`**: The desired instantaneous speed for the Ego vehicle.
- **`trajectory`**: An ordered list of waypoints. Unity should use a smooth controller (like Pure Pursuit or a PID controller) to smoothly drive the Ego vehicle through these `(x, y)` coordinates.

---

## 4. Unity C# Client Implementation (Pseudocode)

Below is an example of how the Unity developer should implement the WebSocket client. We recommend using `NativeWebSocket` or `WebSocketSharp`.

```csharp
using UnityEngine;
using WebSocketSharp;
using Newtonsoft.Json;

public class UnityBrainClient : MonoBehaviour
{
    private WebSocket ws;
    public Transform egoVehicle;
    
    void Start()
    {
        ws = new WebSocket("ws://localhost:8765");
        
        ws.OnMessage += (sender, e) =>
        {
            // 1. Receive Brain Commands
            BrainResponse response = JsonConvert.DeserializeObject<BrainResponse>(e.Data);
            
            // 2. Pass trajectory to your Unity Vehicle Controller
            VehicleController.Instance.SetActiveTrajectory(response.trajectory, response.target_speed);
            
            // 3. Update UI
            DashboardUI.Instance.UpdateRisk(response.risk, response.action);
        };
        
        ws.Connect();
        
        // Start sending state at 10Hz
        InvokeRepeating("SendStateToBrain", 0.1f, 0.1f);
    }

    void SendStateToBrain()
    {
        if (ws.ReadyState != WebSocketState.Open) return;

        // Collect State
        UnityStateMessage msg = new UnityStateMessage
        {
            timestamp = Time.time,
            scenario_id = "market",
            vehicle = new VehicleState {
                x = egoVehicle.position.x,
                y = egoVehicle.position.z, // Mapping Z to Y
                heading = egoVehicle.eulerAngles.y * Mathf.Deg2Rad,
                speed = egoVehicle.GetComponent<Rigidbody>().velocity.magnitude
            },
            objects = PerceptionManager.Instance.GetDynamicObjects()
        };

        // Send JSON
        string json = JsonConvert.SerializeObject(msg);
        ws.Send(json);
    }
    
    void OnDestroy()
    {
        if (ws != null) ws.Close();
    }
}
```

## 5. Error Handling

If Python detects an internal crash, or if a message format is invalid, it will immediately respond with a fallback payload:
```json
{
  "error": "Message validation failed",
  "action": "EMERGENCY_BRAKE"
}
```
Unity must parse this, log the error, and physically halt the Ego vehicle immediately.

## 6. Testing

You can test your Unity connection against the local server without running the full visual Python simulation:
1. Ensure the Python environment is activated (`./setup_and_run.sh`).
2. Run the server: `python src/simulation_interface/unity_server.py`.
3. Press Play in Unity. You should see connection logs in the Python terminal, followed by continuous JSON data exchange.
