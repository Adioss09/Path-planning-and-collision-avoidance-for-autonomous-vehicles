# Algorithms

### Trajectory Prediction
For the proof of concept, we use a **Constant Velocity Model**. The tracker provides historical positions, which we use to compute an average velocity vector `(vx, vy)`. We then extrapolate the position `horizon_seconds` into the future. This simple baseline demonstrates predictive capability without the overhead of heavy sequential neural networks.

### Time-To-Collision (TTC) & Risk
TTC is calculated geometrically. We evaluate the relative position vector and relative velocity vector. If the closing speed is positive (vehicles are approaching), `TTC = distance / closing_speed`. 
The Risk Assessor maps TTC to a categorical risk ('CRITICAL', 'HIGH', 'MEDIUM', 'LOW') using configurable thresholds.

### Adaptive Path Planning
We utilize **A*** (A-Star) search over a 2D occupancy grid (Costmap).
The cost function incorporates:
- Distance
- Obstacle Proximity (Inflated costs near obstacles)
- Collision Risk Weight (Static obstacles get a high penalty, dynamic predicted collisions trigger replanning)
The result is passed to a B-Spline **Path Smoother** to eliminate sharp, non-kinematic turns.

### Vehicle Dynamics
A **Kinematic Bicycle Model** is implemented to step the vehicle state forward given Acceleration and Steering commands. A simple **Pure Pursuit** controller is used to follow the smoothed path.
