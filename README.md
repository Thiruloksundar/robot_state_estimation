# Robot Localization: Kalman Filter vs Particle Filter

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![PyBullet](https://img.shields.io/badge/PyBullet-3.2.5-green.svg)](https://pybullet.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comprehensive comparative study of Kalman Filter (KF) and Particle Filter (PF) for robot localization in realistic indoor environments. This project demonstrates when each filter excels and identifies catastrophic failure cases where Kalman Filters produce physically impossible estimates.

**Authors:** Thirulok Sundar Mohan Rasu, Ravindran Vasoor Saravanan  
**Institution:** University of Michigan

---

## 🎯 Project Overview

This project addresses a fundamental question in probabilistic robotics: **When do Kalman Filters fail catastrophically, and why are Particle Filters essential for real-world deployment?**

We implemented both filters on a PR2 robot navigating through:
- **4-room house environment** with furniture obstacles
- **Symmetric hallway environment** designed to break KF assumptions

### Key Findings

| Scenario | Noise Type | KF Performance | PF Performance | Winner |
|----------|-----------|----------------|----------------|--------|
| 1. Low Gaussian | σ=0.05m | ✓ 0.063m error | ✗ 1.33m error | **KF** |
| 2. Moderate Gaussian | σ=0.10m | ✓ 0.084m error | ✗ 0.124m error | **KF** |
| 3. High Gaussian | σ=0.15m | ✓ 0.109m error | ✗ 0.135m error | **KF** |
| 4. Heavy-Tailed (35% outliers) | Non-Gaussian | ✗ 0.337m error | ✓ 0.094m error | **PF** |
| 5. Symmetric Hallway | Bimodal | ✗ **2.85m error (in wall!)** | ✓ 0.074m error | **PF** |

**Critical Result:** In Scenario 5, the Kalman Filter's mean estimate remained inside the central wall 24.7% of the time, demonstrating fundamental limitations with multimodal distributions.

---

## 📁 Project Structure

```
robot_state_estimation/robot_localization/
├── demo.py                              # Main demonstration script
├── install.sh                           # Package installation script
│
├── environment.py                       # 4-room house environment (2x scaled)
├── symmetric_environment.py             # Symmetric hallway for KF failure case
│
├── robot.py                             # PR2 robot interface
├── sensor_improved.py                   # Multiple noise model sensor
├── ambiguous_sensor.py                  # Bimodal sensor for symmetric env
│
├── kalman_filter_localization.py        # Kalman Filter implementation
├── particle_filter_localization.py      # Particle Filter implementation
│
├── path_planner.py                      # Path planning for house environment
├── symmetric_path.py                    # Path planning for symmetric environment
│
├── noise_configurations.py              # Scenario configurations
└── obstacle_checker.py                  # Collision detection system
```

---

## 🚀 Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/Thiruloksundar/robot_state_estimation.git
cd robot_state_estimation/robot_localization/

# Install dependencies
chmod +x install.sh
./install.sh
```

### Running the Demo

```bash
# Run all 5 scenarios (15-25 minutes)
python3 demo.py

```

### Output

The demo generates:
- `results_comparison_2x.png` - Detailed trajectory and error plots
- `accuracy_comparison_2x.png` - Summary bar charts
- `results_report_2x.txt` - Numerical results and statistics

---

## 🔬 Implementation Details

### System Architecture

1. **Environment Simulator** (PyBullet) - Realistic physics with collision detection
2. **PR2 Robot** - Waypoint-following control with kinematic motion model
3. **Sensor Models** - Gaussian, heavy-tailed, and ambiguous (bimodal) sensors
4. **Localization Filters** - KF and PF implementations
5. **Collision Detection** - Validates estimate physical feasibility

### Motion Model

Discrete-time kinematic model with process noise:

```
u_k = [Δx, Δy, Δθ]ᵀ + ε_motion
ε_motion ~ N(0, Σ_process)
```

- **Time step:** Δt = 0.05s (50 Hz)
- **Process noise:** σ_process ∈ {0.02, 0.04, 0.06}m depending on scenario
- **Collision avoidance:** Ray-casting with reactive lateral offsets (0.3m, 0.5m, 0.7m)

### Sensor Models

#### 1. Gaussian Sensor (Scenarios 1-3)
```
z_k = h(x_true) + ε_sensor
ε_sensor ~ N(0, R)
```
- **Sensor noise:** σ_sensor ∈ {0.05, 0.10, 0.15}m
- **Orientation noise:** σ_θ ∈ {0.03, 0.06, 0.10}rad

#### 2. Heavy-Tailed Sensor (Scenario 4)
Contaminated Gaussian mixture simulating outliers:
```
ε_mixed ~ (1-p)·N(0, σ²) + p·N(0, (10σ)²)
```
- **Outlier probability:** p = 0.35 (35% of measurements)
- **Outlier magnitude:** 10× normal noise (1-2m errors)

#### 3. Ambiguous Sensor (Scenario 5)
Bimodal measurements from symmetric corridors:
```
z_y = { corridor_A_center + N(0, 0.15²)  with prob 0.5
      { corridor_B_center + N(0, 0.15²)  with prob 0.5
```
- **Mode separation:** 6m between corridors
- Creates fundamental ambiguity KF cannot represent

### Kalman Filter

**State:** x = [x, y, v_x, v_y]ᵀ ∈ ℝ⁴ (position + velocity)

**Predict:**
```
x̂_{k+1|k} = F x̂_{k|k}
P_{k+1|k} = F P_{k|k} Fᵀ + Q
```

**Update:**
```
K = P_{k+1|k} Hᵀ (H P_{k+1|k} Hᵀ + R)⁻¹
x̂_{k+1|k+1} = x̂_{k+1|k} + K(z - H x̂_{k+1|k})
P_{k+1|k+1} = (I - KH) P_{k+1|k}
```

**State Transition Matrix:**
```
F = [1 0 Δt 0 ]
    [0 1 0  Δt]
    [0 0 1  0 ]
    [0 0 0  1 ]
```

**Observation Matrix:**
```
H = [1 0 0 0]
    [0 1 0 0]
```

### Particle Filter

**State:** x^i = [x, y, θ]ᵀ ∈ ℝ³ (position + orientation)

**Predict:**
```
x^i_{k+1} = x^i_k + u_k + ε^i
ε^i ~ N(0, Σ_control)
```

**Update:**
```
w^i_{k+1} ∝ w^i_k · exp(-‖z - h(x^i)‖²/(2σ²_sensor))
```

**Resample:** Systematic resampling to eliminate low-weight particles

**Estimate:** Weighted mean (circular statistics for orientation)

**Particle Counts:**
- Scenarios 1-3: N ∈ {1, 2, 3} (intentionally degenerate for KF comparison)
- Scenario 4: N = 100 (sufficient for outlier handling)
- Scenario 5: N = 300 (sufficient for bimodal distribution)

### Collision Detection

Enhanced system checking both obstacles and walls:

```python
def check_collision(env, point, margin=0.35):
    # Check furniture obstacles
    for obstacle in env.obstacles:
        if distance(point, obstacle) < obstacle.radius + margin:
            return True
    
    # Check walls (critical for Scenario 5)
    for wall in env.walls:
        if point inside wall.bounding_box(margin):
            return True
    
    return False
```

**Safety margin:** 0.35m (accounts for robot radius ≈ 0.3m)

---

## 📊 Experimental Results

### Scenario Configurations

| Scenario | Environment | Sensor σ (m) | Process σ (m) | Noise Model | PF Particles |
|----------|-------------|--------------|---------------|-------------|--------------|
| 1 | House | 0.05 | 0.02 | Gaussian | 1 |
| 2 | House | 0.10 | 0.04 | Gaussian | 2 |
| 3 | House | 0.15 | 0.06 | Gaussian | 3 |
| 4 | House | 0.10 | 0.04 | Heavy-tailed (35% outliers) | 100 |
| 5 | Symmetric | 0.15 | 0.03 | Ambiguous (bimodal) | 300 |

### Detailed Results

#### Scenario 1: Low Gaussian Noise (KF Optimal)

**Configuration:** Ideal conditions with minimal noise

| Metric | Kalman Filter | Particle Filter |
|--------|---------------|-----------------|
| Mean Error | **0.0630m** | 1.3268m |
| Median Error | 0.0498m | 1.1842m |
| Std Deviation | 0.0460m | 0.8156m |
| Max Error | 0.2012m | 3.8437m |
| Collision Rate | 5.01% | 7.73% |

**Interpretation:** KF achieves near-optimal performance (6.3cm mean error). PF with N=1 particle degenerates to random walk, demonstrating that particle methods require adequate samples.

#### Scenario 2: Moderate Gaussian Noise (KF Maintains Advantage)

**Configuration:** Realistic WiFi-based localization noise

| Metric | Kalman Filter | Particle Filter |
|--------|---------------|-----------------|
| Mean Error | **0.0839m** | 0.1242m |
| Median Error | 0.0733m | 0.1089m |
| Std Deviation | 0.0508m | 0.0754m |
| Max Error | 0.3642m | 0.6183m |
| Collision Rate | 5.07% | 5.16% |

**Interpretation:** KF maintains 48% advantage despite 2× higher noise. Both filters maintain physically valid estimates.

#### Scenario 3: High Gaussian Noise (KF Robustness)

**Configuration:** Challenging indoor conditions

| Metric | Kalman Filter | Particle Filter |
|--------|---------------|-----------------|
| Mean Error | **0.1091m** | 0.1349m |
| Median Error | 0.1004m | 0.1235m |
| Std Deviation | 0.0601m | 0.0786m |
| Max Error | 0.4856m | 0.5928m |
| Collision Rate | 5.04% | 5.10% |

**Interpretation:** Even with σ=0.15m (3× scenario 1), well-tuned KF achieves 10.9cm accuracy. Performance degradation (+73%) is sublinear in noise increase (+200%), demonstrating optimal Bayesian filtering.

#### Scenario 4: Heavy-Tailed Outliers (KF Fails, PF Succeeds)

**Configuration:** 35% outlier contamination with 10× noise magnitude

| Metric | Kalman Filter | Particle Filter |
|--------|---------------|-----------------|
| Mean Error | 0.3370m | **0.0936m** |
| Median Error | 0.2738m | **0.0780m** |
| Std Deviation | 0.2537m | **0.0639m** |
| Max Error | 1.7045m | **0.5944m** |
| Collision Rate | 4.63% | 4.91% |

**Interpretation:** KF error increases 5.4× compared to Scenario 2 despite identical base noise parameters. Gaussian assumption breaks down—KF computes high Kalman gain and "trusts" outliers, jumping toward them (sometimes through walls). PF's likelihood-based resampling naturally downweights outliers, maintaining 9.4cm accuracy comparable to Gaussian scenarios.

#### Scenario 5: Symmetric Hallway (KF Catastrophic Failure) ⚠️

**Configuration:** Two identical parallel corridors with ambiguous sensor

| Metric | Kalman Filter | Particle Filter |
|--------|---------------|-----------------|
| Mean Error | 2.8529m | **0.0740m** |
| Median Error | 2.9292m | **0.0660m** |
| Std Deviation | 0.5920m | **0.0436m** |
| Max Error | 4.8875m | **0.1839m** |
| **Collision Rate** | **24.68%** | **20.85%** |

**Critical Finding:** KF mean estimate persistently located inside the 1.5m-thick central wall! Because measurements arrive equally from Corridor A (y=+3m, true location) and Corridor B (y=-3m, false symmetric location), KF computes mean as (3 + (-3))/2 = 0, placing estimate inside the wall at y=0.

**Interpretation:** This demonstrates the **fundamental limitation of unimodal Gaussian representations for multimodal posteriors**. The KF mathematically computes the optimal Gaussian approximation to a bimodal distribution, which places the mean between modes—directly inside the obstacle. This is not a tuning error; it's an inherent theoretical limitation. PF with N=300 particles naturally maintains two clusters (one per corridor) and uses motion information to resolve ambiguity, converging to correct location within 50-100 timesteps.

### Combined Performance Analysis

**KF Wins:** 3/5 scenarios (S1, S2, S3) under Gaussian assumptions  
**PF Wins:** 2/5 scenarios (S4, S5) under non-Gaussian conditions  
**KF Critical Failures:**
- Scenario 4: 3.6× worse than PF (heavy-tailed noise)
- Scenario 5: **38× worse than PF**, mean in wall (multimodal ambiguity)

---

## 🎓 Theoretical Insights

### When Kalman Filters Excel

✅ **Linear dynamics** with Gaussian noise  
✅ **Single-mode distributions** (no ambiguity)  
✅ **Computational efficiency** (O(n²) vs O(N) particles)  
✅ **Optimal** for the above conditions (provably minimum variance)

### When Kalman Filters Fail

❌ **Non-Gaussian noise** (heavy-tailed, outliers)  
❌ **Multimodal posteriors** (symmetric environments, kidnapped robot)  
❌ **Non-linear dynamics** (extended/unscented KF needed)  
❌ **Cannot represent** multiple hypotheses simultaneously

### Why Particle Filters Are Essential

✅ **Non-parametric** representation (arbitrary distributions)  
✅ **Multimodal capability** (multiple hypothesis tracking)  
✅ **Outlier robustness** (likelihood-based resampling)  
✅ **Theoretically complete** (converges to true posterior as N→∞)

### The Fundamental Trade-off

**Kalman Filter:**
- Assumes Gaussian → Computationally efficient
- Breaks catastrophically when assumptions violated

**Particle Filter:**
- No assumptions → Robust to real-world complexity
- Requires sufficient particles (computational cost)

**Key Insight:** For safety-critical applications (autonomous vehicles, surgical robots), PF's robustness outweighs computational cost. A 10cm error is acceptable; a 2.85m error inside a wall is catastrophic.

---

## 🛠️ Usage Examples

### Running Individual Scenarios

```python
from demo import run_single_scenario
from noise_configurations import OptimizedNoiseConfigurations

# Run only Scenario 5 (symmetric hallway failure case)
config = OptimizedNoiseConfigurations.scenario_5_symmetric_hallway_kf_catastrophic_failure()
results = run_single_scenario(config, scenario_num=5, use_gui=True, max_time=300)

# Access results
print(f"KF Mean Error: {results['kalman']['mean_error']:.4f}m")
print(f"PF Mean Error: {results['particle']['mean_error']:.4f}m")
print(f"KF Collision Rate: {results['kalman']['collision_rate']:.2%}")
```

### Custom Noise Configuration

```python
custom_config = {
    'name': 'Custom Test',
    'description': 'Your description',
    
    'sensor': {
        'pos_noise_std': 0.12,
        'theta_noise_std': 0.08,
        'noise_model': 'gaussian',
    },
    
    'action': {
        'process_std': 0.05,
        'control_std': (0.05, 0.05, 0.03)
    },
    
    'kalman': {
        'process_std': 0.05,
        'meas_std': 0.12
    },
    
    'particle': {
        'n_particles': 500,
        'pos_std': 0.12,
        'theta_std': 0.08
    }
}

results = run_single_scenario(custom_config, 6, use_gui=False)
```

### Visualizing Results

```python
import matplotlib.pyplot as plt
import numpy as np

# Plot error comparison
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(results['kalman']['errors'], 'b-', label='KF', alpha=0.7)
ax.plot(results['particle']['errors'], 'r-', label='PF', alpha=0.7)
ax.set_xlabel('Time Step')
ax.set_ylabel('Error (m)')
ax.set_title('Localization Error Over Time')
ax.legend()
ax.grid(True, alpha=0.3)
plt.show()
```

---

## 📝 Dependencies

### Required Packages

```
python >= 3.8
numpy >= 1.20.0
matplotlib >= 3.3.0
pybullet >= 3.2.5
pybullet-planning >= 0.5.0
```

### Installation

```bash
pip install numpy matplotlib pybullet pybullet-planning
```

Or use the provided install script:

```bash
./install.sh
```

---

## 🔍 Code Structure Deep Dive

### Filter Implementations

**Kalman Filter** (`kalman_filter_localization.py`):
- State: 4D [x, y, vx, vy]
- Constant velocity motion model
- Standard predict-update cycle
- Optimal for Gaussian systems

**Particle Filter** (`particle_filter_localization.py`):
- State: 3D [x, y, θ]
- Sequential Importance Resampling (SIR)
- Systematic resampling
- Circular statistics for orientation

### Environment Modules

**House Environment** (`environment.py`):
- 2× scaled 4-room layout (30m × 27m total)
- Room dimensions: 15m × 13.5m each
- Corridor width: 7.5m
- Doorways: 5m wide
- 22 obstacles (furniture) distributed across rooms

**Symmetric Environment** (`symmetric_environment.py`):
- Two parallel corridors: 4m wide × 20m long
- Central wall: 1.5m thick (the critical obstacle)
- Identical furniture in both corridors
- Designed to create measurement ambiguity

### Sensor Modules

**Improved Sensor** (`sensor_improved.py`):
- 4 noise models: Gaussian, heavy-tailed, bimodal, distance-dependent
- Configurable parameters for each model
- Statistics tracking (outlier count, measurement count)

**Ambiguous Sensor** (`ambiguous_sensor.py`):
- Bimodal measurement distribution
- 50% probability reports Corridor A, 50% Corridor B
- Creates fundamental ambiguity for Scenario 5

### Path Planning

**House Path Planner** (`path_planner.py`):
- Generates 80+ waypoint path through all 4 rooms
- Visits corners and explores each room thoroughly
- Smooth doorway transitions
- Maintains 0.5m clearance from obstacles

**Symmetric Path Planner** (`symmetric_path.py`):
- Simple linear path through Corridor A
- 40 waypoints from x=-8m to x=+8m
- Constant y-position at corridor center
- Designed to maximize KF failure exposure

---

## 📈 Performance Metrics

### Accuracy Metrics

- **Mean Error:** Average Euclidean distance between estimate and ground truth
- **Median Error:** Robust central tendency measure
- **Std Deviation:** Estimate consistency (lower = more stable)
- **Max Error:** Worst-case performance

### Safety Metrics

- **Collision Rate:** Percentage of timesteps where estimate is inside obstacle/wall
- **Collision Count:** Total number of physically invalid estimates
- Critical threshold: >10% indicates systematic failure

### Computational Metrics

- **Runtime:** Wall-clock time for scenario completion
- **Steps:** Number of simulation timesteps
- **Frequency:** 50 Hz (Δt = 0.05s)

---

## 🎯 Key Takeaways

1. **Kalman Filters are optimal** under ideal conditions (Gaussian noise, linear dynamics)
   - Scenarios 1-3: Mean errors 0.06-0.11m vs 0.12-1.33m for particle-limited PF

2. **Non-Gaussian noise breaks KF assumptions**
   - Scenario 4: 35% outliers → KF error 3.6× worse than PF
   - KF "trusts" outliers due to Gaussian assumption

3. **Multimodal distributions require non-parametric representations**
   - Scenario 5: KF mean in wall 24.7% of time (catastrophic failure)
   - PF naturally maintains multiple hypotheses

4. **Real-world deployment requires robustness**
   - Sensors always produce outliers (reflections, interference, occlusions)
   - Symmetric/aliased environments exist (hallways, offices, warehouses)
   - PF's computational cost is justified for safety-critical applications

5. **The theoretical necessity of Particle Filters**
   - Not just "better" than KF in some cases
   - **Technically necessary** for non-Gaussian posteriors
   - Fundamental limitation of parametric representations

---

## 📚 References

1. LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
   - Chapter 8: Feedback Motion Planning
   - Chapter 9: Sensor-Based Planning

2. Boyd, S., & Vandenberghe, L. (2004). *Convex Optimization*. Cambridge University Press.
   - Chapter 7: Statistical Estimation
   - Appendix C: Numerical Linear Algebra Background

3. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
   - Chapter 4: Nonparametric Filters (Particle Filters)
   - Chapter 3: Gaussian Filters (Kalman Filters)

---

## 🤝 Contributing

Contributions are welcome! Areas for extension:

1. **Additional Scenarios:**
   - Non-linear dynamics (curved paths, rotation)
   - Dynamic obstacles (moving furniture)
   - Sensor occlusions (LIDAR with obstacles)

2. **Filter Variants:**
   - Extended Kalman Filter (EKF)
   - Unscented Kalman Filter (UKF)
   - Adaptive Particle Filter (variable N)
   - Rao-Blackwellized Particle Filter

3. **Environments:**
   - Outdoor navigation (GPS coordinates)
   - 3D environments (aerial robots)
   - Multi-robot scenarios

4. **Performance Optimization:**
   - GPU-accelerated particle filter
   - Parallel resampling algorithms
   - Adaptive noise estimation

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👥 Authors

**Thirulok Sundar Mohan Rasu**  
Email: thirulok@umich.edu  
University of Michigan

**Ravindran Vasoor Saravanan**  
Email: vsravi@umich.edu  
University of Michigan

---

**⭐ Star this repository if you found it helpful!**
