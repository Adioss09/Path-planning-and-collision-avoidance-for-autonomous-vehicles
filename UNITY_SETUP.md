# Unity Setup & Integration Guide: SIH26037 Autonomous Brain

This guide explains exactly how to set up the Unity simulation environment and integrate it with the Python Autonomous Navigation "Brain".

## 1. Important Design Principle
**Do not implement autonomous driving logic inside Unity.** Unity is purely responsible for rendering the environment, moving the Ego vehicle along the Python-generated trajectory, and sending object states back to Python. The actual tracking, path planning, and collision prediction happen continuously inside the Python server.

## 2. Unity Project Setup

1. **Install Unity**: Download Unity Hub and install Unity 2022.3 LTS (or later).
2. **Create Project**: Create a new 3D Core Project named `SIH26037_Simulation`.
3. **Import WebSocket Library**: You need a WebSocket client for C#. The most reliable for Unity is [NativeWebSocket](https://github.com/endel/NativeWebSocket). Download and import the package.
4. **Copy Scripts**: Copy the contents of the `UNITY/UnityProject/Assets/Scripts/` folder from this repository into your Unity project's `Assets/Scripts/` folder.

## 3. Scene Setup

### The Environment
- Create a basic 3D plane for the road.
- Populate the scene with simple Cubes or free assets representing cars, cattle, and pedestrians.

### Dynamic Agents
- Attach the `DynamicAgent.cs` script to **every** moving obstacle (pedestrians, cars, cattle).
- Ensure the `Agent Type` is set correctly (e.g., `cattle`, `car`, `auto`). Unity will automatically calculate their velocity and report it to Python.

### The Ego Vehicle
- Create a distinct object (e.g., a blue Cube) for the autonomous Ego vehicle.
- Attach `VehicleController.cs`. This component uses a Pure Pursuit steering algorithm to smoothly follow the exact sequence of waypoints provided by Python.
- Attach `UnityNetworkClient.cs` to the vehicle (or an empty NetworkManager object). Drag the Ego vehicle into the public `VehicleController` slot.
- Attach `ScenarioManager.cs` to manage different SIH scenarios (e.g., `cattle_crossing`, `urban_intersection`).

## 4. How to Run the Closed-Loop Simulation

The system requires continuous two-way communication.

### Step 1: Start the Python Backend
Open a terminal, activate your python environment, and start the WebSocket server:
```bash
source venv/bin/activate
python src/simulation_interface/unity_server.py
```
*You should see a log saying the server is listening on ws://0.0.0.0:8765.*

### Step 2: Start Unity
Press **Play** in the Unity Editor.

### Step 3: Observe the Closed Loop
1. Unity sends the environment state (Ego position, Cattle position/velocity) to Python 10 times a second.
2. The Python Brain calculates TTC (Time-To-Collision) and Risk.
3. When the Cattle crosses the road, Python's Decision Engine switches to `REPLAN`.
4. Python generates a smooth, curved bezier path around the cattle and sends it to Unity.
5. Unity's `VehicleController` instantaneously steers the Ego vehicle along the new safe trajectory!

## 5. Offline Demonstration Mode
If you need to test the Unity graphics without running the Python server, select the `UnityNetworkClient` component and check the `DEMO_MODE` box. The vehicle will ignore the network and fall back to local offline movement.

## 6. Coordinate System Transformations
Unity uses a 3D left-handed coordinate system `(X, Y=Up, Z=Forward)`.
Our Python backend uses a 2D mathematical coordinate system `(X, Y)`.

The `unity_adapter.py` script automatically handles the conversion:
- Unity `X` → Python `X`
- Unity `Z` → Python `Y`
- Unity `Y` (Vertical) is discarded by Python.
When Python returns a trajectory, the Adapter maps the Python `Y` coordinates back into Unity `Z` coordinates so the C# controller receives perfectly aligned waypoints. You do **not** need to manually convert coordinates in Unity.
