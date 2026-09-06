# Model Selection

Because this PoC is being developed on a MacBook (Apple Silicon M2), we prioritized lightweight, fast models that can leverage the CPU/MPS without requiring an NVIDIA CUDA GPU.

### 1. Object Detection
**Selected Model:** `YOLOv8n` (Ultralytics)
**Reason:** Extremely fast inference speed, minimal dependencies, and pre-trained on COCO which contains classes relevant to Indian roads (person, car, truck, bus, motorcycle, bicycle, animal/cow/dog).

### 2. Road Segmentation
**Selected Model:** `LRASPP MobileNetV3 Large` (Torchvision) / OpenCV Heuristics
**Reason:** Full DeepLabV3 models with ResNet backbones are too heavy for quick iterative development on a laptop. We utilize LRASPP MobileNet for lightweight inference, and provide a heuristic-based fallback for simulating perfect drivable-area extraction in the top-down simulator.

### 3. Tracking
**Selected Model:** Centroid-based Euclidean Tracker
**Reason:** We need persistent IDs to compute velocity vectors. A heavy tracker like DeepSORT requires extracting ReID features for every object, which slows down the loop. A simple centroid tracker is perfectly robust for the 2D simulation scenarios.
