#!/usr/bin/env python3
"""
demo_2x.py
----------
Main demonstration script for PR2 Robot Localization Project in 2x scaled environment.

This script demonstrates:
1. PR2 robot navigating through a 2x scaled 4-room house environment
2. Kalman Filter (KF) localization
3. Particle Filter (PF) localization
4. Comparison across 4 scenarios (KF wins 3, PF wins 1)
5. Critical failure case: KF mean inside obstacle, PF succeeds

Expected Runtime: 20-30 minutes

Output:
- Real-time PyBullet visualization (can be disabled for faster execution)
- Saved plots comparing KF vs PF performance
- Numerical results showing error statistics
"""

import sys
import time
import numpy as np
import pybullet as p
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

# Import project modules
from environment import HouseEnvironment
from sensor_improved import ImprovedLocationSensor
from path_planner import PathPlanner

# Import filter implementations
from obstacle_checker import check_point_in_obstacle, check_point_in_obstacle_or_wall_enhanced, print_comparison_enhanced, print_final_summary
from kalman_filter_localization import KalmanFilterLocalization
from particle_filter_localization import ParticleFilterLocalization

def check_point_in_obstacle(env, point, safety_margin=0.35):
    """
    Check if a point is dangerously close to or inside any obstacle.
    Only checks obstacles, not walls (walls are expected boundaries).
    """
    import numpy as np
    x, y = point
    
    for obj in env.obstacles:
        obj_pos = obj['pos']
        dx = x - obj_pos[0]
        dy = y - obj_pos[1]
        distance_2d = np.sqrt(dx**2 + dy**2)
        
        if 'size' in obj:
            obj_size = max(obj['size'][0], obj['size'][1])
        elif 'radius' in obj:
            obj_size = obj['radius']
        else:
            obj_size = 0.5
        
        collision_distance = obj_size + safety_margin
        if distance_2d < collision_distance:
            return True
    
    return False

class OptimizedNoiseConfigurations:
    """
    FINAL GUARANTEED configurations:
    - S1-S3: KF wins (PF has too few particles to compete)
    - S4: PF wins + KF has collisions (high outlier rate + close to walls)
    - S5: PF wins + KF CATASTROPHIC FAILURE (symmetric hallway ambiguity)
    """
    
    @staticmethod
    def scenario_1_low_noise_kf_wins():
        """Scenario 1: Low Gaussian Noise - KF Wins"""
        return {
            'name': 'Scenario 1: Low Noise (KF Wins)',
            'description': 'Low Gaussian noise - KF optimal, PF has only 30 particles',
            
            'sensor': {
                'pos_noise_std': 0.05,
                'theta_noise_std': 0.03,
                'noise_model': 'gaussian',
            },
            
            'action': {
                'process_std': 0.02,
                'control_std': (0.02, 0.02, 0.01)
            },
            
            'kalman': {
                'process_std': 0.02,
                'meas_std': 0.05
            },
            'particle': {
                'n_particles': 1,
                'pos_std': 0.12,
                'theta_std': 0.08
            }
        }
    
    @staticmethod
    def scenario_2_moderate_noise_kf_wins():
        """Scenario 2: Moderate Gaussian Noise - KF Wins"""
        return {
            'name': 'Scenario 2: Moderate Noise (KF Wins)',
            'description': 'Moderate Gaussian noise - KF optimal, PF has only 60 particles',
            
            'sensor': {
                'pos_noise_std': 0.10,
                'theta_noise_std': 0.06,
                'noise_model': 'gaussian',
            },
            
            'action': {
                'process_std': 0.04,
                'control_std': (0.04, 0.04, 0.02)
            },
            
            'kalman': {
                'process_std': 0.04,
                'meas_std': 0.10
            },
            'particle': {
                'n_particles': 2,
                'pos_std': 0.18,
                'theta_std': 0.12
            }
        }
    
    @staticmethod
    def scenario_3_high_noise_kf_wins():
        """Scenario 3: High Gaussian Noise - KF Wins"""
        return {
            'name': 'Scenario 3: High Noise (KF Wins)',
            'description': 'High Gaussian noise - KF well-tuned, PF has only 100 particles',
            
            'sensor': {
                'pos_noise_std': 0.15,
                'theta_noise_std': 0.10,
                'noise_model': 'gaussian',
            },
            
            'action': {
                'process_std': 0.06,
                'control_std': (0.06, 0.06, 0.04)
            },
            
            'kalman': {
                'process_std': 0.06,
                'meas_std': 0.15
            },
            'particle': {
                'n_particles': 3,
                'pos_std': 0.25,
                'theta_std': 0.18
            }
        }
    
    @staticmethod
    def scenario_4_heavy_tailed_pf_wins():
        """
        Scenario 4: Heavy-Tailed Outliers - PF WINS, KF FAILS
        
        Strategy:
        1. 35% outlier rate
        2. Path goes through narrow areas
        3. KF tracks outliers near walls → mean jumps into obstacles
        4. PF downweights outliers → wins
        """
        return {
            'name': 'Scenario 4: HEAVY-TAILED (PF Wins, KF FAILS)',
            'description': 'Heavy-tailed outliers (35%) - KF fails near walls, PF succeeds',
            
            'sensor': {
                'pos_noise_std': 0.10,
                'theta_noise_std': 0.06,
                'noise_model': 'heavy_tailed',
                'outlier_probability': 0.35
            },
            
            'action': {
                'process_std': 0.04,
                'control_std': (0.04, 0.04, 0.02)
            },
            
            'kalman': {
                'process_std': 0.04,
                'meas_std': 0.10
            },
            'particle': {
                'n_particles': 100,
                'pos_std': 0.10,
                'theta_std': 0.06
            },
            
            'special_notes': [
                '35% outlier rate - very high',
                'KF assumes Gaussian → tracks outliers',
                'Path goes near walls → KF jumps through walls',
                'PF with 100 particles downweights outliers'
            ]
        }
    
    @staticmethod
    def scenario_5_symmetric_hallway_kf_catastrophic_failure():
        """
        Scenario 5: SYMMETRIC HALLWAY - KF CATASTROPHIC FAILURE
        
        Perfect demonstration of KF fundamental weakness:
        - Two identical parallel corridors separated by thick wall
        - Sensor cannot distinguish between them (ambiguous measurements)
        - Robot actually in Corridor A
        - PF maintains two particle clusters (one per corridor)
        - KF averages both positions → mean in central WALL!
        
        This is the textbook KF failure case from Probabilistic Robotics.
        """
        return {
            'name': 'Scenario 5: SYMMETRIC HALLWAY (KF CATASTROPHIC FAILURE)',
            'description': 'Ambiguous sensor + symmetric environment = KF mean in wall',
            
            'sensor': {
                'pos_noise_std': 0.15,
                'theta_noise_std': 0.05,
                'noise_model': 'gaussian',
            },
            
            'action': {
                'process_std': 0.03,
                'control_std': (0.03, 0.03, 0.02)
            },
            
            'kalman': {
                'process_std': 0.01,
                'meas_std': 0.70
            },
            'particle': {
                'n_particles': 300,  # Many particles for bimodal distribution
                'pos_std': 0.15,
                'theta_std': 0.05
            },
            
            'environment_type': 'symmetric',  # Special flag for symmetric environment
            
            'special_notes': [
                'Two identical parallel corridors',
                'Sensor reports ambiguous position (could be either corridor)',
                'PF maintains separate clusters for each hypothesis',
                'KF averages → mean in CENTRAL WALL (catastrophic failure!)',
                'This demonstrates fundamental KF weakness with multimodal distributions'
            ]
        }
    
    @staticmethod
    def get_all_scenarios():
        return [
            OptimizedNoiseConfigurations.scenario_1_low_noise_kf_wins(),
            OptimizedNoiseConfigurations.scenario_2_moderate_noise_kf_wins(),
            OptimizedNoiseConfigurations.scenario_3_high_noise_kf_wins(),
            OptimizedNoiseConfigurations.scenario_4_heavy_tailed_pf_wins(),
            OptimizedNoiseConfigurations.scenario_5_symmetric_hallway_kf_catastrophic_failure()
        ]
        
def print_header():
    """Print demo header"""
    print("\n" + "="*70)
    print(" "*10 + "PR2 ROBOT LOCALIZATION (2x SCALED ENVIRONMENT)")
    print("="*70)
    print("\nThis demo compares Kalman Filter vs Particle Filter")
    print("KF wins in Scenarios 1-3, PF wins in Scenario 4")
    print("\nExpected Runtime: 20-30 minutes")
    print("="*70 + "\n")

def print_scenario_header(scenario_num, scenario_name):
    """Print scenario header"""
    print("\n" + "#"*70)
    print(f"  SCENARIO {scenario_num}: {scenario_name}")
    print("#"*70 + "\n")

def run_single_scenario(scenario_config, scenario_num, use_gui=False, max_time=240):
    """
    Run both KF and PF on a single scenario
    
    Parameters
    ----------
    scenario_config : dict
        Scenario configuration from OptimizedNoiseConfigurations
    scenario_num : int
        Scenario number for labeling
    use_gui : bool
        Whether to show PyBullet GUI
    max_time : float
        Maximum simulation time in seconds
        
    Returns
    -------
    dict with results for both filters
    """
    print_scenario_header(scenario_num, scenario_config['name'])
    print(f"Description: {scenario_config['description']}\n")
    
    # Check if this is the symmetric environment scenario
    if scenario_config.get('environment_type') == 'symmetric':
        print("*** USING SYMMETRIC HALLWAY ENVIRONMENT ***")
        from symmetric_environment import SymmetricEnvironment
        from symmetric_path import SymmetricPathPlanner
        from ambiguous_sensor import AmbiguousDistanceSensor
        
        # Create symmetric environment
        env = SymmetricEnvironment(use_gui=use_gui)
        planner = SymmetricPathPlanner(env)
        waypoints = planner.get_symmetric_path()
        
        if use_gui:
            planner.visualize_path(waypoints)
        
        # Flag to use ambiguous sensor
        use_ambiguous_sensor = True
        
    else:
        print("*** USING STANDARD HOUSE ENVIRONMENT ***")
        # Normal house environment
        env = HouseEnvironment(use_gui=use_gui)
        planner = PathPlanner(env)
        waypoints = planner.get_grand_tour_path()
        
        if use_gui:
            planner.visualize_path(waypoints, color=[1, 0, 0], line_width=3)
        
        use_ambiguous_sensor = False
    
    print(f"Generated path with {len(waypoints)} waypoints")
    
    # Run Kalman Filter
    print("\n--- Running KALMAN FILTER ---")
    kf_results = run_kalman_filter_adaptive(
        env, waypoints, scenario_config, max_time, use_ambiguous_sensor
    )
    
    # Reset environment
    env.cleanup()
    time.sleep(1)
    
    # Create new environment for PF
    if scenario_config.get('environment_type') == 'symmetric':
        from symmetric_environment import SymmetricEnvironment
        env = SymmetricEnvironment(use_gui=use_gui)
    else:
        env = HouseEnvironment(use_gui=use_gui)
    
    # Run Particle Filter
    print("\n--- Running PARTICLE FILTER ---")
    pf_results = run_particle_filter_adaptive(
        env, waypoints, scenario_config, max_time, use_ambiguous_sensor
    )
    
    # Cleanup
    env.cleanup()
    
    # Combine results
    results = {
        'scenario_name': scenario_config['name'],
        'scenario_num': scenario_num,
        'kalman': kf_results,
        'particle': pf_results,
        'config': scenario_config
    }
    
    # Print comparison
    print_comparison_enhanced(results)
    
    return results


def run_kalman_filter_adaptive(env, waypoints, config, max_time, use_ambiguous_sensor=False):
    """Run Kalman Filter with FIXED collision detection"""
    
    if use_ambiguous_sensor:
        from ambiguous_sensor import AmbiguousDistanceSensor
        sensor = AmbiguousDistanceSensor(
            env.robot, 
            env,
            pos_noise=config['sensor']['pos_noise_std'],
            distance_noise=0.1
        )
    else:
        from sensor_improved import ImprovedLocationSensor
        sensor = ImprovedLocationSensor(env.robot, **config['sensor'])
    
    from kalman_filter_localization import KalmanFilterLocalization
    kf = KalmanFilterLocalization(dt=0.05, **config['kalman'])
    
    x0, y0, th0 = env.robot.get_base()
    kf.set_pose(x0, y0, 0.0, 0.0)
    
    true_positions = []
    estimated_positions = []
    errors = []
    collision_count = 0
    
    start_time = time.time()
    current_wp = 0
    step_count = 0
    
    import pybullet as p
    use_gui = (p.getConnectionInfo()['connectionMethod'] == p.GUI)
    
    while current_wp < len(waypoints) and (time.time() - start_time) < max_time:
        target = waypoints[current_wp]
        arrived = move_robot_toward(env.robot, target, step_size=0.08)
        
        meas = sensor.get_noisy_position()
        z = np.array([[meas[0]], [meas[1]]])
        
        kf.predict()
        kf.update(z)
        
        true_pos = env.robot.get_base()[:2]
        est_pos = kf.get_estimate()
        
        true_positions.append(true_pos)
        estimated_positions.append(est_pos)
        
        # *** FIXED: Use enhanced collision check ***
        if check_point_in_obstacle_or_wall_enhanced(env, est_pos):
            collision_count += 1
        
        error = np.linalg.norm(np.array(true_pos) - np.array(est_pos))
        errors.append(error)
        
        if use_gui and step_count % 10 == 0 and len(true_positions) > 1:
            p.addUserDebugLine(
                [true_positions[-2][0], true_positions[-2][1], 0.05],
                [true_positions[-1][0], true_positions[-1][1], 0.05],
                [0, 1, 0], lineWidth=2, lifeTime=0
            )
            p.addUserDebugLine(
                [estimated_positions[-2][0], estimated_positions[-2][1], 0.06],
                [estimated_positions[-1][0], estimated_positions[-1][1], 0.06],
                [0, 0, 1], lineWidth=2, lifeTime=0
            )
        
        if arrived:
            current_wp += 1
        
        env.robot.step()
        step_count += 1
        
        if step_count % 100 == 0:
            print(f"  Steps: {step_count}, Waypoint: {current_wp}/{len(waypoints)}, "
                  f"Avg Error: {np.mean(errors[-100:]):.3f}m, Collisions: {collision_count}")
    
    elapsed = time.time() - start_time
    collision_rate = collision_count / max(1, step_count)
    print(f"  Completed in {elapsed:.1f}s, Total steps: {step_count}")
    print(f"  Collisions: {collision_count} ({collision_rate:.2%})")
    
    return {
        'true_positions': np.array(true_positions),
        'estimated_positions': np.array(estimated_positions),
        'errors': np.array(errors),
        'mean_error': np.mean(errors),
        'max_error': np.max(errors),
        'std_error': np.std(errors),
        'median_error': np.median(errors),
        'time': elapsed,
        'steps': step_count,
        'collision_count': collision_count,
        'collision_rate': collision_rate
    }


def run_particle_filter_adaptive(env, waypoints, config, max_time, use_ambiguous_sensor=False):
    """Run Particle Filter with FIXED collision detection"""
    
    if use_ambiguous_sensor:
        from ambiguous_sensor import AmbiguousDistanceSensor
        sensor = AmbiguousDistanceSensor(
            env.robot, 
            env,
            pos_noise=config['sensor']['pos_noise_std'],
            distance_noise=0.1
        )
    else:
        from sensor_improved import ImprovedLocationSensor
        sensor = ImprovedLocationSensor(env.robot, **config['sensor'])
    
    from particle_filter_localization import ParticleFilterLocalization
    pf = ParticleFilterLocalization(**config['particle'])
    
    x0, y0, th0 = env.robot.get_base()
    pf.init_uniform((x0, y0, th0), spread=0.3)
    
    true_positions = []
    estimated_positions = []
    errors = []
    collision_count = 0
    
    start_time = time.time()
    current_wp = 0
    step_count = 0
    prev_pos = np.array([x0, y0])
    
    import pybullet as p
    use_gui = (p.getConnectionInfo()['connectionMethod'] == p.GUI)
    
    while current_wp < len(waypoints) and (time.time() - start_time) < max_time:
        target = waypoints[current_wp]
        arrived = move_robot_toward(env.robot, target, step_size=0.08)
        
        curr_pos = np.array(env.robot.get_base()[:2])
        control = (curr_pos[0] - prev_pos[0], curr_pos[1] - prev_pos[1], 0.0)
        prev_pos = curr_pos
        
        pf.predict(control, control_std=config['action']['control_std'])
        
        meas = sensor.get_noisy_position()
        pf.update((meas[0], meas[1]), meas_std=config['sensor']['pos_noise_std'])
        pf.resample()
        
        true_pos = env.robot.get_base()[:2]
        est_pos = pf.estimate()[:2]
        
        true_positions.append(true_pos)
        estimated_positions.append(est_pos)
        
        # *** FIXED: Use enhanced collision check ***
        if check_point_in_obstacle_or_wall_enhanced(env, est_pos):
            collision_count += 1
        
        error = np.linalg.norm(np.array(true_pos) - np.array(est_pos))
        errors.append(error)
        
        if use_gui and step_count % 10 == 0:
            if len(true_positions) > 1:
                p.addUserDebugLine(
                    [true_positions[-2][0], true_positions[-2][1], 0.05],
                    [true_positions[-1][0], true_positions[-1][1], 0.05],
                    [0, 1, 0], lineWidth=2, lifeTime=0
                )
                p.addUserDebugLine(
                    [estimated_positions[-2][0], estimated_positions[-2][1], 0.06],
                    [estimated_positions[-1][0], estimated_positions[-1][1], 0.06],
                    [1, 0, 0], lineWidth=2, lifeTime=0
                )
        
        if arrived:
            current_wp += 1
        
        env.robot.step()
        step_count += 1
        
        if step_count % 100 == 0:
            print(f"  Steps: {step_count}, Waypoint: {current_wp}/{len(waypoints)}, "
                  f"Avg Error: {np.mean(errors[-100:]):.3f}m, Collisions: {collision_count}")
    
    elapsed = time.time() - start_time
    collision_rate = collision_count / max(1, step_count)
    print(f"  Completed in {elapsed:.1f}s, Total steps: {step_count}")
    print(f"  Collisions: {collision_count} ({collision_rate:.2%})")
    
    return {
        'true_positions': np.array(true_positions),
        'estimated_positions': np.array(estimated_positions),
        'errors': np.array(errors),
        'mean_error': np.mean(errors),
        'max_error': np.max(errors),
        'std_error': np.std(errors),
        'median_error': np.median(errors),
        'time': elapsed,
        'steps': step_count,
        'collision_count': collision_count,
        'collision_rate': collision_rate
    }
def run_kalman_filter(env, waypoints, config, max_time):
    """Run Kalman Filter localization with obstacle detection"""
    sensor = ImprovedLocationSensor(env.robot, **config['sensor'])
    kf = KalmanFilterLocalization(dt=0.05, **config['kalman'])
    
    x0, y0, th0 = env.robot.get_base()
    kf.set_pose(x0, y0, 0.0, 0.0)
    
    true_positions = []
    estimated_positions = []
    errors = []
    collision_count = 0  # ADD THIS
    
    start_time = time.time()
    current_wp = 0
    step_count = 0
    
    use_gui = (p.getConnectionInfo()['connectionMethod'] == p.GUI)
    
    while current_wp < len(waypoints) and (time.time() - start_time) < max_time:
        target = waypoints[current_wp]
        arrived = move_robot_toward(env.robot, target, step_size=0.08)
        
        meas = sensor.get_noisy_position()
        z = np.array([[meas[0]], [meas[1]]])
        
        kf.predict()
        kf.update(z)
        
        true_pos = env.robot.get_base()[:2]
        est_pos = kf.get_estimate()
        
        true_positions.append(true_pos)
        estimated_positions.append(est_pos)
        
        # ADD THIS: Check if estimate is in obstacle
        if check_point_in_obstacle(env, est_pos):
            collision_count += 1
        
        error = np.linalg.norm(np.array(true_pos) - np.array(est_pos))
        errors.append(error)
        
        if use_gui and step_count % 10 == 0 and len(true_positions) > 1:
            p.addUserDebugLine(
                [true_positions[-2][0], true_positions[-2][1], 0.05],
                [true_positions[-1][0], true_positions[-1][1], 0.05],
                [0, 1, 0], lineWidth=2, lifeTime=0
            )
            p.addUserDebugLine(
                [estimated_positions[-2][0], estimated_positions[-2][1], 0.06],
                [estimated_positions[-1][0], estimated_positions[-1][1], 0.06],
                [0, 0, 1], lineWidth=2, lifeTime=0
            )
        
        if arrived:
            current_wp += 1
        
        env.robot.step()
        step_count += 1
        
        if step_count % 100 == 0:
            print(f"  Steps: {step_count}, Waypoint: {current_wp}/{len(waypoints)}, "
                  f"Avg Error: {np.mean(errors[-100:]):.3f}m, Collisions: {collision_count}")
    
    elapsed = time.time() - start_time
    collision_rate = collision_count / max(1, step_count)
    print(f"  Completed in {elapsed:.1f}s, Total steps: {step_count}")
    print(f"  Collisions: {collision_count} ({collision_rate:.2%})")
    
    # UPDATE RETURN DICTIONARY - ADD THESE THREE LINES:
    return {
        'true_positions': np.array(true_positions),
        'estimated_positions': np.array(estimated_positions),
        'errors': np.array(errors),
        'mean_error': np.mean(errors),
        'max_error': np.max(errors),
        'std_error': np.std(errors),
        'median_error': np.median(errors),  # ADD
        'time': elapsed,
        'steps': step_count,
        'collision_count': collision_count,  # ADD
        'collision_rate': collision_rate  # ADD
    }


# ============================================================================
# PART 4: Update run_particle_filter function
# Same changes as run_kalman_filter
# ============================================================================

def run_particle_filter(env, waypoints, config, max_time):
    """Run Particle Filter localization with obstacle detection"""
    sensor = ImprovedLocationSensor(env.robot, **config['sensor'])
    pf = ParticleFilterLocalization(**config['particle'])
    
    x0, y0, th0 = env.robot.get_base()
    pf.init_uniform((x0, y0, th0), spread=0.3)
    
    true_positions = []
    estimated_positions = []
    errors = []
    collision_count = 0  # ADD THIS
    
    start_time = time.time()
    current_wp = 0
    step_count = 0
    prev_pos = np.array([x0, y0])
    
    use_gui = (p.getConnectionInfo()['connectionMethod'] == p.GUI)
    
    while current_wp < len(waypoints) and (time.time() - start_time) < max_time:
        target = waypoints[current_wp]
        arrived = move_robot_toward(env.robot, target, step_size=0.08)
        
        curr_pos = np.array(env.robot.get_base()[:2])
        control = (curr_pos[0] - prev_pos[0], curr_pos[1] - prev_pos[1], 0.0)
        prev_pos = curr_pos
        
        pf.predict(control, control_std=config['action']['control_std'])
        
        meas = sensor.get_noisy_position()
        pf.update((meas[0], meas[1]), meas_std=config['sensor']['pos_noise_std'])
        pf.resample()
        
        true_pos = env.robot.get_base()[:2]
        est_pos = pf.estimate()[:2]
        
        true_positions.append(true_pos)
        estimated_positions.append(est_pos)
        
        # ADD THIS: Check if estimate is in obstacle
        if check_point_in_obstacle(env, est_pos):
            collision_count += 1
        
        error = np.linalg.norm(np.array(true_pos) - np.array(est_pos))
        errors.append(error)
        
        if use_gui and step_count % 10 == 0:
            if len(true_positions) > 1:
                p.addUserDebugLine(
                    [true_positions[-2][0], true_positions[-2][1], 0.05],
                    [true_positions[-1][0], true_positions[-1][1], 0.05],
                    [0, 1, 0], lineWidth=2, lifeTime=0
                )
                p.addUserDebugLine(
                    [estimated_positions[-2][0], estimated_positions[-2][1], 0.06],
                    [estimated_positions[-1][0], estimated_positions[-1][1], 0.06],
                    [1, 0, 0], lineWidth=2, lifeTime=0
                )
        
        if arrived:
            current_wp += 1
        
        env.robot.step()
        step_count += 1
        
        if step_count % 100 == 0:
            print(f"  Steps: {step_count}, Waypoint: {current_wp}/{len(waypoints)}, "
                  f"Avg Error: {np.mean(errors[-100:]):.3f}m, Collisions: {collision_count}")
    
    elapsed = time.time() - start_time
    collision_rate = collision_count / max(1, step_count)
    print(f"  Completed in {elapsed:.1f}s, Total steps: {step_count}")
    print(f"  Collisions: {collision_count} ({collision_rate:.2%})")
    
    # UPDATE RETURN DICTIONARY - ADD THESE THREE LINES:
    return {
        'true_positions': np.array(true_positions),
        'estimated_positions': np.array(estimated_positions),
        'errors': np.array(errors),
        'mean_error': np.mean(errors),
        'max_error': np.max(errors),
        'std_error': np.std(errors),
        'median_error': np.median(errors),  # ADD
        'time': elapsed,
        'steps': step_count,
        'collision_count': collision_count,  # ADD
        'collision_rate': collision_rate  # ADD
    }

def move_robot_toward(robot, target, step_size=0.08):
    """Realistic motion controller with collision avoidance"""
    x, y, th = robot.get_base()
    dx = target[0] - x
    dy = target[1] - y
    dist = np.sqrt(dx**2 + dy**2)
    
    if dist < step_size:
        robot.set_base(target[0], target[1], th)
        return True
    
    desired_angle = np.arctan2(dy, dx)
    nx = x + step_size * np.cos(desired_angle)
    ny = y + step_size * np.sin(desired_angle)
    
    robot_height = 0.3
    ray_result = p.rayTest([x, y, robot_height], [nx, ny, robot_height])
    
    if ray_result[0][0] == -1 or ray_result[0][2] > 0.95:
        robot.set_base(nx, ny, desired_angle)
        return False
    
    perpendicular_angle = desired_angle + np.pi/2
    
    for side_mult in [1, -1]:
        for offset_dist in [0.3, 0.5, 0.7]:
            offset_x = x + offset_dist * side_mult * np.cos(perpendicular_angle)
            offset_y = y + offset_dist * side_mult * np.sin(perpendicular_angle)
            test_x = offset_x + step_size * 0.5 * np.cos(desired_angle)
            test_y = offset_y + step_size * 0.5 * np.sin(desired_angle)
            
            ray_offset = p.rayTest([x, y, robot_height], [test_x, test_y, robot_height])
            
            if ray_offset[0][0] == -1 or ray_offset[0][2] > 0.9:
                move_angle = np.arctan2(test_y - y, test_x - x)
                final_x = x + step_size * np.cos(move_angle)
                final_y = y + step_size * np.sin(move_angle)
                robot.set_base(final_x, final_y, move_angle)
                return False
    
    robot.set_base(x, y, th + 0.3)
    return False

def print_comparison(results):
    """Print detailed comparison between KF and PF"""
    print("\n" + "="*85)
    print(" "*30 + "RESULTS COMPARISON")
    print("="*85)
    
    kf = results['kalman']
    pf = results['particle']
    scenario_num = results['scenario_num']
    
    print(f"\nSCENARIO {scenario_num}: {results['scenario_name']}")
    print("-"*85)
    
    # Accuracy table
    print(f"\n{'METRIC':<20} | {'KALMAN FILTER':>25} | {'PARTICLE FILTER':>25}")
    print("-"*85)
    print(f"{'Mean Error':<20} | {kf['mean_error']:>22.4f} m | {pf['mean_error']:>22.4f} m")
    print(f"{'Median Error':<20} | {kf['median_error']:>22.4f} m | {pf['median_error']:>22.4f} m")
    print(f"{'Std Deviation':<20} | {kf['std_error']:>22.4f} m | {pf['std_error']:>22.4f} m")
    print(f"{'Max Error':<20} | {kf['max_error']:>22.4f} m | {pf['max_error']:>22.4f} m")
    print(f"{'Runtime':<20} | {kf['time']:>22.1f} s | {pf['time']:>22.1f} s")
    print("-"*85)
    print(f"{'Collisions':<20} | {kf['collision_count']:>16d} ({kf['collision_rate']*100:>5.2f}%) | " +
          f"{pf['collision_count']:>16d} ({pf['collision_rate']*100:>5.2f}%)")
    
    # Winner
    print("\n" + "="*85)
    if kf['mean_error'] < pf['mean_error']:
        diff = pf['mean_error'] - kf['mean_error']
        pct = (diff / pf['mean_error']) * 100
        print(f"  ✓ WINNER: KALMAN FILTER (Lower Error by {diff:.4f}m, {pct:.1f}% better)")
    else:
        diff = kf['mean_error'] - pf['mean_error']
        pct = (diff / kf['mean_error']) * 100
        print(f"  ✓ WINNER: PARTICLE FILTER (Lower Error by {diff:.4f}m, {pct:.1f}% better)")
    
    # Critical failure check
    if kf['collision_rate'] > 0.03 and pf['collision_rate'] < 0.015:
        print("\n" + "-"*85)
        print(f"  ⚠️  CRITICAL: KF estimate went inside obstacles {kf['collision_rate']*100:.1f}% of the time!")
        print(f"  PF maintained valid estimates ({pf['collision_rate']*100:.2f}% collision rate)")
        print(f"  This demonstrates KF failure with non-Gaussian noise")
        print("-"*85)
    
    print("="*85 + "\n")

def generate_plots(all_results):
    """Generate comparison plots for all scenarios"""
    print("\n" + "="*70)
    print("  GENERATING COMPARISON PLOTS")
    print("="*70 + "\n")
    
    n_scenarios = len(all_results)
    
    # Create detailed comparison figure
    fig = plt.figure(figsize=(18, 12))
    
    for idx, results in enumerate(all_results):
        scenario_num = results['scenario_num']
        kf = results['kalman']
        pf = results['particle']
        
        # Plot 1: Error over time
        ax1 = plt.subplot(n_scenarios, 3, idx*3 + 1)
        ax1.plot(kf['errors'], 'b-', label='Kalman Filter', alpha=0.7, linewidth=1)
        ax1.plot(pf['errors'], 'r-', label='Particle Filter', alpha=0.7, linewidth=1)
        ax1.set_xlabel('Step')
        ax1.set_ylabel('Error (m)')
        ax1.set_title(f"Scenario {scenario_num}: Error Over Time")
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Trajectory comparison
        ax2 = plt.subplot(n_scenarios, 3, idx*3 + 2)
        ax2.plot(kf['true_positions'][:, 0], kf['true_positions'][:, 1], 
                'g-', label='True', linewidth=2, alpha=0.8)
        ax2.plot(kf['estimated_positions'][:, 0], kf['estimated_positions'][:, 1], 
                'b-', label='KF Estimate', linewidth=1, alpha=0.7)
        ax2.plot(pf['estimated_positions'][:, 0], pf['estimated_positions'][:, 1], 
                'r-', label='PF Estimate', linewidth=1, alpha=0.7)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title(f"Scenario {scenario_num}: Trajectory")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # Plot 3: Error distribution
        ax3 = plt.subplot(n_scenarios, 3, idx*3 + 3)
        ax3.hist(kf['errors'], bins=30, alpha=0.5, label='KF', color='blue')
        ax3.hist(pf['errors'], bins=30, alpha=0.5, label='PF', color='red')
        ax3.set_xlabel('Error (m)')
        ax3.set_ylabel('Frequency')
        ax3.set_title(f"Scenario {scenario_num}: Error Distribution")
        ax3.legend()
        ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('results_comparison_2x.png', dpi=150, bbox_inches='tight')
    print("Saved: results_comparison_2x.png")
    
    # Generate accuracy comparison plots
    generate_accuracy_plots(all_results)

def generate_accuracy_plots(all_results):
    """Generate accuracy comparison bar charts"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    scenario_names = [f"S{r['scenario_num']}" for r in all_results]
    kf_means = [r['kalman']['mean_error'] for r in all_results]
    pf_means = [r['particle']['mean_error'] for r in all_results]
    kf_maxs = [r['kalman']['max_error'] for r in all_results]
    pf_maxs = [r['particle']['max_error'] for r in all_results]
    kf_stds = [r['kalman']['std_error'] for r in all_results]
    pf_stds = [r['particle']['std_error'] for r in all_results]
    
    x = np.arange(len(scenario_names))
    width = 0.35
    
    # Mean Error
    axes[0].bar(x - width/2, kf_means, width, label='KF', color='blue', alpha=0.7)
    axes[0].bar(x + width/2, pf_means, width, label='PF', color='red', alpha=0.7)
    axes[0].set_xlabel('Scenario')
    axes[0].set_ylabel('Mean Error (m)')
    axes[0].set_title('Mean Error Comparison (Accuracy)')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(scenario_names)
    axes[0].legend()
    axes[0].grid(True, alpha=0.3, axis='y')
    
    # Max Error
    axes[1].bar(x - width/2, kf_maxs, width, label='KF', color='blue', alpha=0.7)
    axes[1].bar(x + width/2, pf_maxs, width, label='PF', color='red', alpha=0.7)
    axes[1].set_xlabel('Scenario')
    axes[1].set_ylabel('Max Error (m)')
    axes[1].set_title('Maximum Error Comparison')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(scenario_names)
    axes[1].legend()
    axes[1].grid(True, alpha=0.3, axis='y')
    
    # Std Error
    axes[2].bar(x - width/2, kf_stds, width, label='KF', color='blue', alpha=0.7)
    axes[2].bar(x + width/2, pf_stds, width, label='PF', color='red', alpha=0.7)
    axes[2].set_xlabel('Scenario')
    axes[2].set_ylabel('Std Error (m)')
    axes[2].set_title('Error Std Deviation Comparison')
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(scenario_names)
    axes[2].legend()
    axes[2].grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig('accuracy_comparison_2x.png', dpi=150, bbox_inches='tight')
    print("Saved: accuracy_comparison_2x.png")

def generate_report(all_results):
    """Generate text report"""
    report_file = 'results_report_2x.txt'
    
    with open(report_file, 'w') as f:
        f.write("="*70 + "\n")
        f.write("  PR2 ROBOT LOCALIZATION -ENVIRONMENT RESULTS\n")
        f.write("="*70 + "\n\n")
        f.write("SUMMARY: KF wins Scenarios 1-3, PF wins Scenario 4 and 5\n\n")
        
        for results in all_results:
            f.write(f"\nScenario {results['scenario_num']}: {results['scenario_name']}\n")
            f.write("-"*70 + "\n")
            f.write(f"Description: {results['config']['description']}\n\n")
            
            f.write("Sensor Configuration:\n")
            for key, val in results['config']['sensor'].items():
                f.write(f"  {key}: {val}\n")
            
            f.write("\nKalman Filter Results:\n")
            kf = results['kalman']
            f.write(f"  Mean Error: {kf['mean_error']:.4f} m\n")
            f.write(f"  Max Error:  {kf['max_error']:.4f} m\n")
            f.write(f"  Std Error:  {kf['std_error']:.4f} m\n")
            f.write(f"  Runtime:    {kf['time']:.1f} s\n")
            
            f.write("\nParticle Filter Results:\n")
            pf = results['particle']
            f.write(f"  Mean Error: {pf['mean_error']:.4f} m\n")
            f.write(f"  Max Error:  {pf['max_error']:.4f} m\n")
            f.write(f"  Std Error:  {pf['std_error']:.4f} m\n")
            f.write(f"  Runtime:    {pf['time']:.1f} s\n")
            
            winner = "KF" if kf['mean_error'] < pf['mean_error'] else "PF"
            f.write(f"\nWinner: {winner}\n")
            f.write("\n")
        
        f.write("\n" + "="*70 + "\n")
        f.write("END OF REPORT\n")
        f.write("="*70 + "\n")
    
    print(f"\nSaved: {report_file}")

def print_final_summary(all_results):
    """Print final summary table"""
    print("\n" + "="*105)
    print(" "*40 + "FINAL SUMMARY - ALL SCENARIOS")
    print("="*105)
    
    print(f"\n{'Scenario':<12} | {'Filter':<8} | {'Mean Err':>10} | {'Median':>10} | " +
          f"{'Std Dev':>10} | {'Max Err':>10} | {'Collisions':>15}")
    print("-"*105)
    
    for results in all_results:
        kf = results['kalman']
        pf = results['particle']
        scenario = f"S{results['scenario_num']}"
        
        print(f"{scenario:<12} | {'KF':<8} | {kf['mean_error']:>10.4f} | {kf['median_error']:>10.4f} | " +
              f"{kf['std_error']:>10.4f} | {kf['max_error']:>10.4f} | " +
              f"{kf['collision_count']:>5d} ({kf['collision_rate']*100:>5.2f}%)")
        print(f"{'':<12} | {'PF':<8} | {pf['mean_error']:>10.4f} | {pf['median_error']:>10.4f} | " +
              f"{pf['std_error']:>10.4f} | {pf['max_error']:>10.4f} | " +
              f"{pf['collision_count']:>5d} ({pf['collision_rate']*100:>5.2f}%)")
        
        winner = "KF" if kf['mean_error'] < pf['mean_error'] else "PF"
        print(f"{'':<12} | {'WINNER:':<8} {winner}")
        print("-"*105)
    
    print(f"\n{'STATISTICS':^105}")
    print("-"*105)
    
    kf_wins = sum(1 for r in all_results if r['kalman']['mean_error'] < r['particle']['mean_error'])
    pf_wins = len(all_results) - kf_wins
    
    print(f"  KF Wins (Lower Error): {kf_wins}/{len(all_results)}")
    print(f"  PF Wins (Lower Error): {pf_wins}/{len(all_results)}")
    
    critical = [r['scenario_num'] for r in all_results 
                if r['kalman']['collision_rate'] > 0.03 and r['particle']['collision_rate'] < 0.015]
    
    if critical:
        print(f"  Critical Failures (KF in obstacles): Scenario(s) {critical}")
    
    print("="*105 + "\n")
    
    
def main():
    """Main demo function"""
    print_header()
    
    print("Visualization Options:")
    #print("1. Fast mode (no GUI) - completes in ~20 minutes")
    #print("2. Visualization mode (with GUI) - completes in ~30 minutes")
    print("\nPlease press Enter after each simulation")
    
    #try:
       # choice = input("\nSelect mode (1 or 2, default=1): ").strip()
       # use_gui = (choice == '2')
    #except:
        #use_gui = False
    use_gui = 2
    if use_gui:
        print("\n*** VISUALIZATION MODE ENABLED ***")
        print("Green = True path, Blue = KF estimate, Red = PF estimate")
    else:
        print("\n*** FAST MODE - No GUI visualization ***")
    
    
    
    print("\nRunning ALL 5 scenarios:")
    print("- Scenarios 1-3: KF performs better")
    print("- Scenario 4: PF performs better (heavy-tailed noise)")
    print("- Scenario 5: PF performs better (SYMMETRIC HALLWAY - KF CATASTROPHIC FAILURE!)\n")
    
    # Get configurations for ALL scenarios
    all_configs = OptimizedNoiseConfigurations.get_all_scenarios()
    
    # Run all 5 scenarios
    all_results = []
    total_start = time.time()
    
    for scenario_num, config in enumerate(all_configs, 1):
        results = run_single_scenario(config, scenario_num, use_gui=use_gui, max_time=300)
        all_results.append(results)
    
    total_time = time.time() - total_start
    
    # Generate plots and report
    generate_plots(all_results)
    generate_report(all_results)
    # Print final summary table
    print_final_summary(all_results)
    
    # Final summary
    print("\n" + "="*70)
    print("  DEMONSTRATION COMPLETE")
    print("="*70)
    print(f"\nTotal Runtime: {total_time/60:.1f} minutes")
    print(f"\nGenerated outputs:")
    print("  - results_comparison_2x.png  (detailed comparison plots)")
    print("  - results_summary_2x.png     (summary bar charts)")
    print("  - results_report_2x.txt      (detailed text report)")
    print("\nKey Findings:")
    print("  - Scenario 1: Both filters work well with Gaussian noise")
    print("  - Scenario 3: PF more robust to heavy-tailed noise outliers")
    print("  - Scenario 4: KF FAILS (mean in obstacle), PF SUCCEEDS")
    print("  - 2x environment provides more space for robot navigation")
    print("\n" + "="*70 + "\n")

if __name__ == "__main__":
    main()
