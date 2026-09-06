# SIH26037 — Adaptive Path Planning & Collision Avoidance
## Master Project Specification & Autonomous Build Instructions

---

# 0. ROLE

You are an expert autonomous software engineer, ML engineer, computer-vision engineer, robotics engineer, and hackathon developer.

You are responsible for independently creating a complete proof-of-concept implementation for **Smart India Hackathon problem SIH26037 — Adaptive Path Planning & Collision Avoidance**.

Treat this document as the **single source of truth** for the project.

Do not merely create a conceptual demo or static UI.

Build a genuine, runnable proof-of-concept that demonstrates the core technical pipeline:

**Perception → Scene Understanding → Tracking → Prediction → Risk Assessment → Decision Making → Adaptive Path Planning → Vehicle Motion → Replanning**

The project must be designed so that it can later be extended into the full MATLAB/RoadRunner/Simulink solution described by the SIH problem statement.

---

# 1. PROBLEM STATEMENT

## SIH26037 — Adaptive Path Planning & Collision Avoidance

### Background

Most autonomous driving systems are developed for roads with clear lane markings, standard signage, predictable traffic flow, and controlled intersections.

Indian roads are often very different.

Vehicles of many types share the same space, including:

- Cars
- Buses
- Trucks
- Auto-rickshaws
- Two-wheelers
- Bicycles
- Pedestrians
- Pushcarts
- Animals

Road users may:

- Change direction suddenly
- Merge without signalling
- Drive against traffic
- Cross at unmarked locations
- Overtake unpredictably

In many areas:

- Road edges are unclear
- Lane markings are missing
- Potholes are common
- Lane discipline is limited
- Traffic is heterogeneous
- Road geometry is poorly structured

These conditions make traditional path-planning methods that depend on structured road geometry and predictable motion difficult to use.

The goal is to design and simulate an adaptive path-planning system for an autonomous vehicle operating under these conditions.

---

# 2. REQUIRED SYSTEM BEHAVIOR

The system should:

1. Perceive the surrounding environment.
2. Detect diverse road users and obstacles.
3. Understand the drivable road area.
4. Track relevant dynamic objects across frames.
5. Estimate/predict short-term movement.
6. Determine whether the current trajectory is safe.
7. Calculate collision risk.
8. Make a driving decision.
9. Generate a collision-free path.
10. Replan the path whenever the environment changes.
11. Demonstrate safe vehicle motion.
12. Work particularly well for unstructured Indian traffic conditions.

The core principle is:

> The vehicle must not simply react to where an object is now. It must reason about where the object is likely to be and adapt its trajectory accordingly.

---

# 3. DATASETS AVAILABLE

The project directory contains downloaded datasets from the official **India Driving Dataset (IDD)**.

The available datasets include:

- IDD Segmentation Part 1
- IDD Segmentation Part 2
- IDD Detection Dataset

The datasets are large.

DO NOT attempt to blindly train on the complete datasets.

The project is being developed on a MacBook and is currently a **proof-of-concept for an SIH presentation**.

Therefore:

- Inspect the datasets first.
- Determine their exact structure and annotation format.
- Determine the available classes.
- Determine the number of images.
- Determine class distribution.
- Automatically create a manageable curated subset.
- Do not duplicate unnecessary data.
- Do not modify the original datasets.
- Keep original data untouched.

---

# 4. DATASET DISCOVERY — IMPORTANT

Before implementing the ML pipeline:

### Automatically inspect the project directory.

Find:

- Segmentation Part 1
- Segmentation Part 2
- Detection dataset
- Images
- Annotation files
- JSON/XML/TXT/mask files
- Class definitions
- Train/validation/test splits, if already provided

Do not assume folder names.

Do not assume annotation formats.

Do not assume class IDs.

First inspect the actual files.

Create a script:

`src/data/inspect_dataset.py`

that reports:

- Dataset structure
- Number of images
- Number of annotations
- Image dimensions
- Annotation format
- Class names
- Class frequencies
- Missing annotations
- Corrupted files
- Duplicate files where practical

Save the inspection report to:

`results/dataset_report.json`

and optionally:

`results/dataset_report.md`

---

# 5. DATA SUBSET STRATEGY

The objective is NOT maximum dataset size.

The objective is:

> Maximum useful diversity for a computationally manageable PoC.

Create a curated subset from the downloaded datasets.

Target approximately:

### Detection
3,000–5,000 images initially.

### Segmentation
2,000–4,000 images initially.

Adjust automatically based on:

- Dataset size
- Class distribution
- Available disk space
- Training feasibility
- Hardware constraints

Do not unnecessarily copy full-resolution duplicates.

Prefer symlinks or lightweight indexing where appropriate.

---

# 6. IMPORTANT CLASSES

Identify the actual IDD class names and map them to the following conceptual categories where possible:

### Dynamic road users

- Car
- Bus
- Truck
- Auto-rickshaw
- Two-wheeler
- Bicycle
- Pedestrian
- Animal

### Static obstacles

- Pushcart
- Roadside obstacles
- Construction objects
- Other relevant obstacle classes

### Environment

- Road
- Drivable area
- Sidewalk
- Vegetation
- Building
- Other relevant semantic classes

Do NOT invent classes that do not exist in the dataset.

Where a requested class does not exist directly, document the closest available category.

Create:

`configs/classes.yaml`

containing the final class mapping.

---

# 7. DATA PREPROCESSING

Create a reproducible preprocessing pipeline.

Required functionality:

1. Validate images.
2. Validate annotations.
3. Remove invalid pairs.
4. Resize images appropriately.
5. Normalize data as required by the selected model.
6. Convert annotations into the format required by the model.
7. Generate train/validation/test splits.
8. Preserve class balance as much as reasonably possible.
9. Produce dataset statistics.

Create:

`src/data/prepare_dataset.py`

The preprocessing pipeline must be reproducible.

Running it again should not create inconsistent splits.

---

# 8. MODEL STRATEGY

Do NOT train a huge model from scratch.

The system must be optimized for development on a MacBook.

Prefer:

- Lightweight pretrained models
- Transfer learning
- Small input resolution where appropriate
- Efficient inference
- CPU/MPS-compatible implementations

The exact architecture should be selected after inspecting the data.

Potential detection choices include:

- YOLO-family lightweight model
- YOLOv8n / YOLO11n or equivalent lightweight detector
- Another lightweight pretrained detector if technically more suitable

Potential segmentation choices include:

- SegFormer-B0
- DeepLabV3-MobileNet
- Fast-SCNN
- Another lightweight semantic segmentation model

Do not choose a model simply because it is popular.

Choose based on:

- MacBook compatibility
- Training time
- Accuracy
- Ease of implementation
- Inference speed
- Availability of pretrained weights
- Ease of deployment

Document the choice in:

`docs/MODEL_SELECTION.md`

---

# 9. OBJECT DETECTION MODULE

Implement:

`src/perception/detector.py`

The detector should produce:

```text
class
confidence
bounding box
center point
estimated position
```

Example:

```json
{
  "class": "auto_rickshaw",
  "confidence": 0.91,
  "bbox": [x1, y1, x2, y2],
  "center": [x, y]
}
```

The system must support inference on:

- Single images
- Image sequences
- Video

Create visualization utilities that render:

- Bounding boxes
- Class labels
- Confidence
- Object IDs when tracking is enabled

Save example results in:

`results/detection/`

---

# 10. ROAD / DRIVABLE-AREA SEGMENTATION

Implement:

`src/perception/segmenter.py`

The segmentation system should identify the usable road/drivable area.

The purpose is NOT simply to produce a pretty segmentation mask.

The output must eventually help the planner determine:

> Where is the vehicle allowed/safely able to travel?

Use the segmentation output to estimate:

- Drivable region
- Road boundaries
- Free space
- Areas that should be avoided

Save example outputs in:

`results/segmentation/`

---

# 11. SCENE REPRESENTATION

Create a unified representation of the environment.

Implement:

`src/perception/scene.py`

The scene representation should combine:

### Detection

"What objects are present?"

### Segmentation

"Where can the vehicle travel?"

### Tracking

"How are objects moving?"

Example:

```text
Scene
├── Drivable area
├── Road boundaries
├── Dynamic agents
│   ├── Car
│   ├── Auto-rickshaw
│   ├── Pedestrian
│   └── Two-wheeler
└── Static obstacles
```

---

# 12. OBJECT TRACKING

Implement lightweight multi-object tracking.

Possible approaches:

- ByteTrack
- SORT
- DeepSORT
- Another lightweight tracker

Prefer the simplest reliable option.

The tracker should assign persistent IDs:

```text
Pedestrian #3
Auto-rickshaw #7
Two-wheeler #11
```

For each tracked object maintain:

- Position
- Previous position
- Velocity estimate
- Direction
- Track history
- Object class

Implement:

`src/prediction/tracker.py`

---

# 13. SHORT-TERM TRAJECTORY PREDICTION

The project requires prediction of short-term motion.

For the PoC, do NOT immediately build a complicated deep-learning trajectory-prediction model.

Start with a robust lightweight baseline such as:

### Constant Velocity Model

Given:

```text
x_t
v_t
```

estimate:

```text
x_(t+Δt) = x_t + v_t * Δt
```

Optionally support:

- Constant acceleration
- Moving-average velocity
- Direction smoothing

The prediction horizon should be configurable.

For example:

```yaml
prediction:
  horizon_seconds: 3
  timestep: 0.2
```

The architecture should allow a learned trajectory predictor to replace this baseline later.

Create:

`src/prediction/trajectory_predictor.py`

---

# 14. COLLISION RISK ASSESSMENT

Implement a collision-risk module.

Create:

`src/safety/risk.py`

Use concepts such as:

### Time To Collision (TTC)

Estimate whether the predicted trajectories of:

- Ego vehicle
- Dynamic object

will intersect.

Calculate:

- TTC
- Minimum predicted distance
- Collision probability/risk score
- Safety margin

Example:

```text
Risk Level:

LOW
MEDIUM
HIGH
CRITICAL
```

The thresholds must be configurable.

Example:

```yaml
safety:
  critical_ttc: 1.0
  high_ttc: 2.0
  medium_ttc: 4.0
  minimum_clearance_m: 1.5
```

These are initial values only.

Document assumptions.

---

# 15. DECISION ENGINE

Implement:

`src/decision/decision_engine.py`

The decision engine should determine the vehicle's action.

Possible actions:

```text
CRUISE
SLOW_DOWN
STOP
REPLAN
EMERGENCY_BRAKE
```

Basic logic:

### LOW RISK

Continue following the current trajectory.

### MEDIUM RISK

Reduce speed and monitor.

### HIGH RISK

Trigger adaptive replanning.

### CRITICAL RISK

Perform emergency braking or safest available evasive action.

The system must prioritize safety over shortest travel time.

---

# 16. ADAPTIVE PATH PLANNING

This is the central component of SIH26037.

Create:

`src/planning/`

The planner should support:

- Initial path generation
- Dynamic obstacle avoidance
- Replanning
- Goal reaching
- Drivable-area constraints
- Safety margins
- Path smoothness

Potential algorithms:

### Initial path

A* or another grid-based planner.

### Adaptive replanning

Prefer:

- D* Lite
- Anytime D*
- Incremental A*
- Hybrid A*
- RRT* where appropriate

Choose the algorithm that gives the best PoC balance.

The planner must be able to respond when:

```text
Environment changes
        ↓
Obstacle appears
        ↓
Current path becomes unsafe
        ↓
Recalculate safe path
        ↓
Vehicle follows new path
```

This is the most important behavior to demonstrate.

---

# 17. PATH COST FUNCTION

Do not optimize only for shortest distance.

Use a weighted cost function such as:

```text
Total Cost =
    Distance Cost
  + Collision Risk Cost
  + Obstacle Proximity Cost
  + Path Curvature Cost
  + Smoothness Cost
  + Replanning Cost
```

Conceptually:

```text
J = w_d D
  + w_r R
  + w_c C
  + w_s S
```

where:

- D = path distance
- R = collision risk
- C = curvature/change cost
- S = smoothness penalty

Weights should be configurable.

Safety must receive a significantly higher priority than minor distance savings.

---

# 18. PATH SMOOTHING

A raw grid path may contain sharp turns.

Implement a path smoothing stage.

Possible approaches:

- Cubic spline
- B-spline
- Bézier interpolation
- Savitzky-Golay smoothing where appropriate

Ensure that smoothing does not cause the trajectory to enter obstacle regions.

Create:

`src/planning/path_smoother.py`

---

# 19. VEHICLE MODEL

We do not need a full high-fidelity vehicle dynamics model for the current PoC.

Implement a simplified kinematic bicycle model.

State:

```text
x
y
yaw
velocity
```

Control:

```text
steering
acceleration
```

Use it to demonstrate that the vehicle can follow the generated path.

Create:

`src/vehicle/bicycle_model.py`

---

# 20. CLOSED-LOOP SIMULATION

The system must be closed loop.

Do NOT simply generate a path once and display it.

The simulation should follow:

```text
Environment
    ↓
Sensors / Data
    ↓
Perception
    ↓
Tracking
    ↓
Prediction
    ↓
Risk Assessment
    ↓
Decision
    ↓
Path Planning
    ↓
Vehicle Control
    ↓
Vehicle Motion
    ↓
Updated Environment
    ↓
Perception again
```

The vehicle should periodically reevaluate the environment.

---

# 21. SENSOR ABSTRACTION

The SIH problem specifically mentions:

- Camera
- LiDAR
- Radar

For the current PoC, actual physical sensors are NOT required.

However, design the software architecture so that sensor inputs can later be integrated.

Create:

`src/sensors/`

with an abstraction such as:

```text
CameraSensor
LidarSensor
RadarSensor
SensorFusion
```

For the current implementation:

### Camera

Use IDD images/video.

### LiDAR

Simulate depth/distance information from available scene geometry or object positions.

### Radar

Simulate relative velocity for dynamic objects using tracked positions.

Clearly label simulated sensor information as simulated.

Do not falsely claim that the current PoC uses real LiDAR/radar measurements.

---

# 22. SENSOR FUSION

Create a simple fusion layer.

Conceptually:

```text
Camera
   +
LiDAR
   +
Radar
   ↓
Unified Object State
```

The PoC may use simulated measurements.

The architecture must allow future replacement with real sensor data.

---

# 23. FIVE REQUIRED SIH SCENARIOS

The final PoC must support at least five scenarios corresponding to the SIH requirement.

---

## Scenario 1 — Unmarked Village Road

Characteristics:

- No lane markings
- Unclear road boundaries
- Irregular road geometry
- Slow traffic
- Possible pedestrian/two-wheeler movement

System requirement:

- Use drivable-area understanding rather than relying solely on lanes.
- Maintain a safe path.

---

## Scenario 2 — Busy Urban Intersection Without Signals

Characteristics:

- Multiple directions of traffic
- No traffic signal
- Informal merging
- Pedestrians
- Two-wheelers
- Auto-rickshaws

System requirement:

- Predict surrounding movement.
- Assess collision risk.
- Adapt the vehicle trajectory.

---

## Scenario 3 — Highway Merge With Slow-Moving Vehicles

Characteristics:

- High-speed ego vehicle
- Slow-moving vehicle
- Vehicle merging into traffic
- Significant speed differences

System requirement:

- Predict relative movement.
- Calculate TTC.
- Slow down or replan appropriately.

---

## Scenario 4 — Dense Market Area

Characteristics:

- High traffic density
- Pedestrians
- Auto-rickshaws
- Two-wheelers
- Pushcarts
- Irregular movement
- Narrow usable road area

System requirement:

- Prioritize collision avoidance.
- Reduce speed.
- Continuously replan.

---

## Scenario 5 — Sudden Cattle Crossing

Characteristics:

- Vehicle travelling normally
- Animal suddenly enters road
- Potential collision trajectory

System requirement:

```text
Animal detected
      ↓
Trajectory predicted
      ↓
Collision risk HIGH
      ↓
Emergency decision
      ↓
Safe braking / evasive path
```

This scenario should produce one of the strongest visual demonstrations.

---

# 24. SIMULATION ENGINE

The current PoC may use a Python-based simulation if this is substantially faster to implement than MATLAB/RoadRunner.

However, structure the system so that the final architecture maps cleanly to:

```text
RoadRunner
     ↓
Automated Driving Toolbox
     ↓
Sensor Fusion
     ↓
Stateflow
     ↓
Navigation Toolbox
     ↓
Vehicle Dynamics / Simulink
```

Do NOT claim that the Python PoC itself is RoadRunner/Simulink.

Instead document it as:

> "A software proof-of-concept architecture designed for subsequent MATLAB/RoadRunner/Simulink integration."

---

# 25. SIMULATION VISUALIZATION

Create a visually impressive top-down simulation.

The visualization should show:

### Vehicle

Blue/clearly distinguishable ego vehicle.

### Obstacles

Different icons/shapes/colors for:

- Cars
- Auto-rickshaws
- Pedestrians
- Two-wheelers
- Animals
- Static obstacles

### Paths

Show:

- Original planned path
- Current active path
- Replanned path
- Predicted trajectories

### Safety

Display:

- Collision risk
- TTC
- Minimum clearance
- Current action

Example dashboard:

```text
┌──────────────────────────────────┐
│ ADAPTIVE AUTONOMOUS NAVIGATION   │
├──────────────────────────────────┤
│ Speed:             28 km/h       │
│ TTC:                1.8 s        │
│ Risk:              HIGH          │
│ Action:            REPLANNING    │
│                                  │
│ Objects:            7            │
│ Dynamic Objects:    4            │
│                                  │
│ Path Status:        SAFE ✓       │
└──────────────────────────────────┘
```

---

# 26. USER INTERFACE

Create a simple professional dashboard.

It should contain:

### Main simulation

Large visualization area.

### Control panel

Allow:

- Start simulation
- Pause
- Reset
- Select scenario
- Spawn obstacle
- Toggle prediction
- Toggle risk visualization
- Toggle planned/replanned paths

### Metrics panel

Show:

- Current speed
- TTC
- Risk level
- Replanning count
- Replanning latency
- Path length
- Minimum clearance
- Scenario completion

The interface should look like an engineering/autonomous-driving prototype rather than a generic website.

---

# 27. SCENARIO EVENTS

The simulation must allow dynamic events.

Examples:

```text
t = 0s
Normal driving

t = 4s
Auto-rickshaw begins merging

t = 5s
Collision risk increases

t = 5.2s
Decision engine detects HIGH risk

t = 5.3s
Replanning triggered

t = 5.5s
New path generated

t = 6s
Vehicle follows new path
```

The timing should be configurable.

---

# 28. PERFORMANCE METRICS

Implement measurement of:

### 1. Scenario completion rate

Percentage of scenarios completed without collision.

### 2. Collision count

Number of collisions.

### 3. Replanning latency

Time between:

```text
Unsafe condition detected
        ↓
New safe path available
```

### 4. Path smoothness

Use curvature or steering-change metrics.

### 5. Minimum obstacle clearance

Minimum distance between vehicle and obstacles.

### 6. Travel time

Time required to reach destination.

### 7. Number of replans

Number of adaptive path changes.

### 8. Emergency interventions

Number of emergency braking events.

Create:

`src/evaluation/metrics.py`

and save results to:

`results/metrics/`

---

# 29. EXPERIMENTAL COMPARISON

Where practical, compare:

### Baseline

Simple static shortest-path planner.

versus

### Proposed

Risk-aware adaptive planner.

Demonstrate that the baseline can fail or require stopping when an obstacle appears, while the adaptive system can replan.

Example:

```text
                 Baseline       Proposed
Collision           1              0
Replans             0              3
TTC response        Late           Early
Path smoothness     Medium         High
Completion          80%            100%
```

Do NOT fabricate numbers.

Only show values actually measured by the implementation.

---

# 30. PROJECT STRUCTURE

Create a clean structure similar to:

```text
SIH26037/
│
├── README.md
├── SIH26037_MASTER_SPEC.md
├── requirements.txt
├── environment.yml
│
├── configs/
│   ├── classes.yaml
│   ├── training.yaml
│   ├── planner.yaml
│   ├── safety.yaml
│   └── scenarios.yaml
│
├── data/
│   ├── raw/
│   ├── processed/
│   └── subset/
│
├── models/
│   ├── detection/
│   ├── segmentation/
│   └── prediction/
│
├── src/
│   ├── data/
│   ├── perception/
│   ├── sensors/
│   ├── tracking/
│   ├── prediction/
│   ├── safety/
│   ├── decision/
│   ├── planning/
│   ├── vehicle/
│   ├── simulation/
│   ├── evaluation/
│   └── utils/
│
├── scripts/
│   ├── prepare_data.py
│   ├── train_detector.py
│   ├── train_segmenter.py
│   ├── run_inference.py
│   └── run_simulation.py
│
├── results/
│   ├── detection/
│   ├── segmentation/
│   ├── trajectories/
│   ├── scenarios/
│   ├── metrics/
│   └── figures/
│
├── docs/
│   ├── ARCHITECTURE.md
│   ├── MODEL_SELECTION.md
│   ├── DATASET.md
│   ├── ALGORITHMS.md
│   └── RESULTS.md
│
└── demo/
    └── ...
```

Adapt the structure if necessary.

Do not create unnecessary files.

---

# 31. COMMAND-LINE INTERFACE

The project must be runnable through simple commands.

Examples:

```bash
python scripts/prepare_data.py
```

```bash
python scripts/train_detector.py
```

```bash
python scripts/train_segmenter.py
```

```bash
python scripts/run_inference.py --image <path>
```

```bash
python scripts/run_simulation.py --scenario market
```

```bash
python scripts/run_simulation.py --scenario cattle
```

If a GUI is implemented, provide:

```bash
python scripts/run_demo.py
```

Create clear instructions in `README.md`.

---

# 32. HARDWARE CONSTRAINT

The development machine is a MacBook.

Therefore:

- Detect Apple Silicon/MPS where available.
- Support CPU fallback.
- Avoid CUDA-only assumptions.
- Do not require an NVIDIA GPU.
- Use small models.
- Use mixed precision only where supported safely.
- Keep batch sizes configurable.
- Allow reduced-resolution training.
- Provide a quick-training configuration.

Example:

```yaml
hardware:
  device: auto
  batch_size: 8
  workers: 2
```

The code should automatically select:

```text
MPS → CUDA → CPU
```

according to availability.

---

# 33. TRAINING REQUIREMENTS

Training must be reproducible.

Save:

- Model weights
- Configuration
- Training logs
- Validation metrics
- Best checkpoint
- Final checkpoint

Do not train indefinitely.

Provide:

### Quick mode

Designed for PoC development.

### Full mode

Uses the larger curated subset.

Example:

```bash
python scripts/train_detector.py --mode quick
```

and:

```bash
python scripts/train_detector.py --mode full
```

---

# 34. FAILURE HANDLING

The software must handle:

- Missing files
- Corrupted images
- Missing annotations
- Unsupported image formats
- Empty detections
- No predicted trajectory
- Planner failure
- No safe path
- Sensor failure simulation

If no safe path exists:

```text
STOP / EMERGENCY BRAKE
```

Never force the vehicle through an unsafe path.

---

# 35. SAFETY PRIORITY

The planning objective must prioritize:

1. Collision avoidance
2. Vehicle stability
3. Minimum clearance
4. Path feasibility
5. Smoothness
6. Travel time
7. Path length

Never prioritize shortest path over safety.

---

# 36. VISUAL OUTPUTS REQUIRED

Automatically generate polished figures for the final SIH presentation.

At minimum:

### Figure 1

Original IDD image.

### Figure 2

Detection output.

### Figure 3

Segmentation output.

### Figure 4

Tracking + predicted trajectories.

### Figure 5

Collision-risk visualization.

### Figure 6

Initial path vs replanned path.

### Figure 7

Five scenario screenshots.

### Figure 8

Performance metrics.

Save everything under:

`results/figures/`

Use readable labels and professional formatting.

---

# 37. ARCHITECTURE DIAGRAM

Generate an architecture diagram representing:

```text
                 IDD / SIMULATION
                        │
                        ▼
              ┌─────────────────┐
              │    PERCEPTION   │
              │ Camera/LiDAR/   │
              │ Radar           │
              └────────┬────────┘
                       ▼
                 SENSOR FUSION
                       │
                       ▼
              OBJECT DETECTION
                       │
                       ▼
                    TRACKING
                       │
                       ▼
             TRAJECTORY PREDICTION
                       │
                       ▼
               RISK ASSESSMENT
                       │
                       ▼
                DECISION ENGINE
                       │
                       ▼
              ADAPTIVE PLANNER
                       │
                       ▼
              PATH SMOOTHING
                       │
                       ▼
               VEHICLE MODEL
                       │
                       ▼
                 ENVIRONMENT
                       │
                       └────── feedback
```

Save a visual version under:

`results/figures/system_architecture.png`

---

# 38. DOCUMENTATION

Create concise technical documentation explaining:

### README.md

- Project overview
- Installation
- Dataset preparation
- Training
- Inference
- Running simulation
- Running scenarios
- Results

### DATASET.md

- Dataset source
- Dataset structure
- Selected subset
- Classes
- Preprocessing
- Dataset limitations

### MODEL_SELECTION.md

Explain:

- Detection model selected
- Segmentation model selected
- Tracking algorithm
- Prediction method
- Why each was selected

### ALGORITHMS.md

Explain:

- Detection
- Segmentation
- Tracking
- Prediction
- TTC
- Risk assessment
- Decision engine
- Path planning
- Path smoothing
- Vehicle model

### ARCHITECTURE.md

Explain the complete system flow.

### RESULTS.md

Automatically populate with measured results after experiments.

---

# 39. IMPORTANT HONESTY REQUIREMENT

Never claim that something was implemented if it was not.

Clearly distinguish between:

### Implemented PoC

and:

### Proposed future integration

For example:

```text
CURRENT PoC:
Python + IDD + lightweight ML + custom simulation

FUTURE FULL SYSTEM:
RoadRunner + MATLAB + Automated Driving Toolbox
+ Navigation Toolbox + Stateflow + Simulink
+ real Camera/LiDAR/Radar
```

Do not claim real LiDAR/radar measurements if they are simulated.

Do not fabricate accuracy metrics.

Do not fabricate scenario results.

---

# 40. FINAL DEMONSTRATION FLOW

The final demo should follow this sequence:

```text
1. Select scenario
        ↓
2. Vehicle starts
        ↓
3. Perception detects road users
        ↓
4. Objects are tracked
        ↓
5. Future movement is predicted
        ↓
6. Collision risk is calculated
        ↓
7. Vehicle continues safely
        ↓
8. Dynamic obstacle appears
        ↓
9. Collision risk increases
        ↓
10. Decision engine triggers REPLAN
        ↓
11. New safe path generated
        ↓
12. Vehicle follows new path
        ↓
13. Vehicle reaches destination
```

This should be visually obvious to a judge.

---

# 41. PRIORITY ORDER

Because this is currently a 2-day PoC, prioritize development in exactly this order:

## Priority 1 — MUST WORK

- Dataset inspection
- Detection
- Basic segmentation
- Simulation environment
- Vehicle movement
- Obstacle representation
- Initial path
- Dynamic obstacle
- Collision detection
- Adaptive replanning
- Visualization

## Priority 2 — SHOULD WORK

- Tracking
- Short-term prediction
- TTC
- Decision engine
- Five scenarios
- Metrics

## Priority 3 — NICE TO HAVE

- Advanced trajectory prediction
- Sensor fusion simulation
- Sophisticated vehicle dynamics
- Advanced learned planner
- High-fidelity rendering
- MATLAB/RoadRunner bridge

If time becomes limited, never sacrifice Priority 1 for Priority 3.

---

# 42. IMPLEMENTATION STRATEGY

Do not attempt to build everything at once.

Implement incrementally:

### Step 1

Inspect datasets.

### Step 2

Create curated subsets.

### Step 3

Train/test detection.

### Step 4

Train/test segmentation.

### Step 5

Build basic simulation.

### Step 6

Implement static obstacle avoidance.

### Step 7

Add dynamic obstacles.

### Step 8

Add tracking.

### Step 9

Add trajectory prediction.

### Step 10

Add TTC/risk assessment.

### Step 11

Add decision engine.

### Step 12

Add adaptive replanning.

### Step 13

Add vehicle model.

### Step 14

Integrate all components.

### Step 15

Implement five scenarios.

### Step 16

Generate metrics.

### Step 17

Generate presentation figures.

### Step 18

Polish demo.

At every stage, run a test before proceeding.

---

# 43. TESTING

Create automated/basic tests for:

- Dataset loader
- Annotation parser
- Detector output
- Segmenter output
- Tracker
- Trajectory predictor
- TTC calculation
- Risk classifier
- Planner
- Path collision checking
- Vehicle model
- Scenario simulation

Create:

`tests/`

The project should not silently fail.

---

# 44. DEFINITION OF DONE

The project is considered successfully implemented when:

### Data

- IDD data has been inspected.
- Curated subset has been created.
- Dataset preprocessing works.

### Perception

- Object detection works.
- Road segmentation works.

### Prediction

- Objects can be tracked.
- Short-term trajectories can be predicted.

### Safety

- TTC/risk can be calculated.
- Risk levels influence decisions.

### Planning

- Initial path can be generated.
- Dynamic obstacles can invalidate the path.
- Planner can generate a new path.
- New path is collision-free.

### Vehicle

- Vehicle can follow the generated trajectory.

### Simulation

- Environment changes dynamically.
- System responds to changes.
- Five SIH scenarios can be demonstrated.

### Evaluation

- Metrics are automatically calculated.
- Results are saved.

### Presentation

- Figures are generated.
- Architecture is documented.
- Demo can be run easily.

---

# 45. DO NOT DO THESE THINGS

Do NOT:

- Train on the entire dataset unnecessarily.
- Train models from scratch unless absolutely necessary.
- Build a generic obstacle avoidance demo disconnected from Indian road conditions.
- Use only synthetic images when IDD data is available.
- Pretend simulated sensors are physical sensors.
- Fabricate metrics.
- Hard-code results.
- Hard-code a single path for every scenario.
- Make the project dependent on CUDA.
- Create an unnecessarily complex frontend before the core planner works.
- Spend most of the development time on UI.
- Implement advanced AI simply for the sake of saying "AI".
- Ignore safety when optimizing path length.

---

# 46. CORE PROJECT MESSAGE

The final implementation must communicate this idea clearly:

> **Traditional autonomous navigation assumes structured roads and predictable traffic. Our system is designed for the uncertainty of Indian roads by combining scene understanding, heterogeneous-agent tracking, short-term motion prediction, risk-aware decision making, and continuous adaptive path replanning.**

The most important innovation is:

> **Predictive, risk-aware adaptive navigation for unstructured mixed traffic rather than simple reactive obstacle avoidance.**

---

# 47. FINAL OUTPUTS

At the end of implementation, ensure the repository contains:

```text
✓ Trained detection model
✓ Trained segmentation model
✓ Dataset subset
✓ Dataset preprocessing scripts
✓ Detection inference
✓ Segmentation inference
✓ Tracking
✓ Trajectory prediction
✓ TTC/risk assessment
✓ Decision engine
✓ Adaptive planner
✓ Path smoothing
✓ Vehicle model
✓ Closed-loop simulation
✓ Five scenarios
✓ Metrics
✓ Visualizations
✓ Architecture diagram
✓ README
✓ Technical documentation
✓ Demo launcher
```

---

# 48. FIRST ACTION

DO NOT immediately start coding.

First:

1. Inspect the entire provided project directory.
2. Locate all IDD datasets.
3. Determine their exact structure.
4. Produce a dataset report.
5. Determine the actual classes.
6. Determine annotation formats.
7. Determine dataset sizes.
8. Propose the smallest sensible subset for the PoC.
9. Check available hardware.
10. Determine whether Apple Silicon MPS is available.
11. Select the ML models based on the actual data.
12. Create a short implementation plan.

Then begin implementation.

Do not ask unnecessary questions if the answer can be determined by inspecting the provided files.

Only ask for user input when a decision genuinely cannot be determined from the repository or this specification.

---

# 49. DEVELOPMENT PRINCIPLE

The goal is not to create the largest or most sophisticated autonomous-driving system.

The goal is to create the **most convincing technically valid SIH26037 proof-of-concept possible within limited hardware and time**.

Every implementation decision should be evaluated using:

```text
SIH Relevance
      ×
Technical Credibility
      ×
Demo Value
      ×
Feasibility
```

Prioritize features that maximize all four.

The final system should be understandable to a judge within a few minutes while still being technically defensible under detailed questioning.

---

# END OF SPECIFICATION