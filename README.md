# SIH26037 - Adaptive Path Planning & Collision Avoidance

This is a proof-of-concept (PoC) implementation for **SIH26037**. Traditional autonomous navigation systems are designed for structured roads. This system is designed for the uncertainty of Indian roads by combining scene understanding, heterogeneous-agent tracking, short-term motion prediction, risk-aware decision making, and continuous adaptive path replanning.

## Features
- **Predictive, Risk-Aware Adaptive Navigation** rather than simple reactive avoidance.
- **Custom 2D Simulation Engine** using Pygame to visualize 5 required SIH scenarios.
- **Lightweight Perception Pipeline** utilizing YOLOv8n and MobileNet.
- **A* Adaptive Replanning** that safely navigates around dynamic obstacles.
- **Metrics Tracking** for measuring Time-to-Collision (TTC) and minimum clearance.

## Installation

1. Create a Python 3.10+ environment.
2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Running the System

### 1. Dataset Preparation
If you have the India Driving Dataset (IDD) archives in the root directory (`idd-detection.tar.gz`, `idd-segmentation.tar.gz`), run:
```bash
python src/data/inspect_dataset.py
python scripts/prepare_data.py
```
*This extracts a manageable subset of images and annotations for local development.*

### 2. Run Simulation
To see the adaptive path planning in action, launch the simulation dashboard:
```bash
python scripts/run_simulation.py --scenario market
```
Available Scenarios: `unmarked`, `intersection`, `market`, `cattle`

## Documentation
- [Architecture](docs/ARCHITECTURE.md)
- [Model Selection](docs/MODEL_SELECTION.md)
- [Dataset](docs/DATASET.md)
- [Algorithms](docs/ALGORITHMS.md)
- [Results](docs/RESULTS.md)
