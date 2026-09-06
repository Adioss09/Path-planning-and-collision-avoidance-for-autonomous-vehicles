# System Architecture

This proof-of-concept represents a pipeline designed for subsequent MATLAB/RoadRunner/Simulink integration.

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

The current PoC implements this entirely in Python, utilizing Pygame for the closed-loop simulation environment.
