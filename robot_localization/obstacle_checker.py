"""
obstacle_checker_fixed.py
-------------------------
Fixed obstacle detection with proper collision checking and improved logic.
"""

"""
obstacle_checker.py
-------------------
Enhanced obstacle detection with symmetric environment support
"""

import pybullet as p
import numpy as np

# *** ADD THESE THREE FUNCTIONS HERE ***

def check_point_in_central_wall_symmetric(env, point):
    """
    Specifically check if point is in the central wall of symmetric environment
    """
    x, y = point
    
    if not hasattr(env, 'wall_thickness'):
        return False
    
    wall_half_thickness = env.wall_thickness / 2.0
    
    if abs(y) < wall_half_thickness + 0.2:
        if abs(x) < env.corridor_width / 2.0:
            return True
    
    return False


def check_point_in_obstacle_or_wall_enhanced(env, point, safety_margin=0.3):
    """
    Enhanced collision check that works for both regular and symmetric environments
    """
    x, y = point
    
    # First check standard obstacles
    for obj in env.obstacles:
        obj_pos = obj['pos']
        dx = x - obj_pos[0]
        dy = y - obj_pos[1]
        distance_2d = np.sqrt(dx**2 + dy**2)
        
        if 'size' in obj:
            obj_radius = max(obj['size'][0], obj['size'][1])
        elif 'radius' in obj:
            obj_radius = obj['radius']
        else:
            obj_radius = 0.5
        
        if distance_2d < (obj_radius + safety_margin):
            return True
    
    # Check if this is symmetric environment
    if hasattr(env, 'wall_thickness') and hasattr(env, 'corridor_width'):
        # Symmetric environment - check central wall specifically
        return check_point_in_central_wall_symmetric(env, point)
    
    # For regular house environment, check walls with bounding boxes
    for wall in env.walls:
        wall_pos = wall['pos']
        wall_size = wall['size']
        
        x_min = wall_pos[0] - wall_size[0] - safety_margin
        x_max = wall_pos[0] + wall_size[0] + safety_margin
        y_min = wall_pos[1] - wall_size[1] - safety_margin
        y_max = wall_pos[1] + wall_size[1] + safety_margin
        
        if x_min <= x <= x_max and y_min <= y <= y_max:
            if wall_size[0] < 10 and wall_size[1] < 10:
                return True
    
    return False


def check_point_in_obstacle(env, point, safety_margin=0.2):
    """
    Check if a point is inside any obstacle or wall using proper distance checking.
    
    Parameters
    ----------
    env : HouseEnvironment
        The environment with obstacles and walls
    point : tuple
        (x, y) position to check
    safety_margin : float
        Safety margin around obstacles (default 0.2m)
        
    Returns
    -------
    bool
        True if point is dangerously close to obstacle (likely inside), False otherwise
    """
    return check_point_in_obstacle_or_wall_enhanced(env, point, safety_margin)
    
    x, y = point
    
    # Check distance to all obstacles (not walls - walls are expected boundaries)
    for obj in env.obstacles:  # Only check obstacles, not walls
        obj_pos = obj['pos']
        
        # Calculate 2D distance from point to obstacle center
        dx = x - obj_pos[0]
        dy = y - obj_pos[1]
        distance_2d = np.sqrt(dx**2 + dy**2)
        
        # Determine obstacle effective radius
        if 'size' in obj:
            # Box obstacle - use maximum extent
            obj_radius = max(obj['size'][0], obj['size'][1])
        elif 'radius' in obj:
            # Cylinder obstacle
            obj_radius = obj['radius']
        else:
            obj_radius = 0.5  # Default
        
        # Check if point is inside obstacle (with safety margin)
        if distance_2d < (obj_radius + safety_margin):
            return True
    
    return False


def print_comparison_enhanced(results):
    """Print detailed comparison between KF and PF with proper failure detection"""
    print("\n" + "="*80)
    print(" "*25 + "DETAILED RESULTS COMPARISON")
    print("="*80)
    
    kf = results['kalman']
    pf = results['particle']
    scenario_num = results['scenario_num']
    
    print(f"\nSCENARIO {scenario_num}: {results['scenario_name']}")
    print("-"*80)
    
    # Accuracy Metrics Table
    print(f"\n{'METRIC':<25} | {'KALMAN FILTER':>20} | {'PARTICLE FILTER':>20}")
    print("-"*80)
    print(f"{'Mean Error (m)':<25} | {kf['mean_error']:>20.4f} | {pf['mean_error']:>20.4f}")
    print(f"{'Median Error (m)':<25} | {kf['median_error']:>20.4f} | {pf['median_error']:>20.4f}")
    print(f"{'Std Deviation (m)':<25} | {kf['std_error']:>20.4f} | {pf['std_error']:>20.4f}")
    print(f"{'Max Error (m)':<25} | {kf['max_error']:>20.4f} | {pf['max_error']:>20.4f}")
    print(f"{'Min Error (m)':<25} | {np.min(kf['errors']):>20.4f} | {np.min(pf['errors']):>20.4f}")
    print("-"*80)
    print(f"{'Runtime (s)':<25} | {kf['time']:>20.1f} | {pf['time']:>20.1f}")
    print(f"{'Total Steps':<25} | {kf['steps']:>20d} | {pf['steps']:>20d}")
    print("-"*80)
    
    # Obstacle Collision Analysis
    print(f"\n{'OBSTACLE COLLISION ANALYSIS':^80}")
    print("-"*80)
    print(f"{'Steps in Obstacle':<25} | {kf['collision_count']:>20d} | {pf['collision_count']:>20d}")
    print(f"{'Collision Rate (%)':<25} | {kf['collision_rate']*100:>20.2f} | {pf['collision_rate']*100:>20.2f}")
    
    # Winner Determination
    print("\n" + "="*80)
    print(f"{'ACCURACY WINNER':^80}")
    print("="*80)
    
    kf_better_accuracy = kf['mean_error'] < pf['mean_error']
    
    if kf_better_accuracy:
        improvement = ((pf['mean_error'] - kf['mean_error']) / pf['mean_error']) * 100
        print(f"  ✓ KALMAN FILTER WINS (Lower Error)")
        print(f"    • Better by: {pf['mean_error'] - kf['mean_error']:.4f} meters")
        print(f"    • Improvement: {improvement:.1f}%")
    else:
        improvement = ((kf['mean_error'] - pf['mean_error']) / kf['mean_error']) * 100
        print(f"  ✓ PARTICLE FILTER WINS (Lower Error)")
        print(f"    • Better by: {kf['mean_error'] - pf['mean_error']:.4f} meters")
        print(f"    • Improvement: {improvement:.1f}%")
    
    # Critical Failure Detection - Fixed Logic
    print("\n" + "="*80)
    
    kf_has_collisions = kf['collision_rate'] > 0.05  # >5% collision rate
    pf_has_collisions = pf['collision_rate'] > 0.05
    kf_much_worse_collisions = kf['collision_rate'] > pf['collision_rate'] * 2
    
    if kf_has_collisions and kf_much_worse_collisions:
        # KF has significant collisions AND much worse than PF
        print(f"{'⚠️  CRITICAL FAILURE DETECTED  ⚠️':^80}")
        print("="*80)
        print(f"  KF Mean Estimate went INSIDE OBSTACLES significantly more than PF!")
        print(f"  • KF Collision Rate: {kf['collision_rate']*100:.1f}% ({kf['collision_count']} steps)")
        print(f"  • PF Collision Rate: {pf['collision_rate']*100:.1f}% ({pf['collision_count']} steps)")
        print(f"  • Despite {'better' if kf_better_accuracy else 'worse'} mean error, KF estimate is INVALID (inside obstacles)")
        print(f"  • This demonstrates KF's failure with non-Gaussian/multimodal noise!")
        print("="*80)
    elif kf_has_collisions or pf_has_collisions:
        print(f"{'⚠️  SOME INSTABILITY DETECTED  ⚠️':^80}")
        print("="*80)
        print(f"  • KF had {kf['collision_count']} instances inside obstacles ({kf['collision_rate']*100:.1f}%)")
        print(f"  • PF had {pf['collision_count']} instances inside obstacles ({pf['collision_rate']*100:.1f}%)")
        print("="*80)
    else:
        print(f"{'✓ BOTH FILTERS MAINTAIN VALID ESTIMATES':^80}")
        print("="*80)
        print(f"  • Both filters avoid obstacles successfully")
        print(f"  • KF Collision Rate: {kf['collision_rate']*100:.2f}%")
        print(f"  • PF Collision Rate: {pf['collision_rate']*100:.2f}%")
        print("="*80)
    
    print("\n")


def print_final_summary(all_results):
    """Print final summary table for all scenarios"""
    print("\n" + "="*100)
    print(" "*35 + "FINAL SUMMARY - ALL SCENARIOS")
    print("="*100)
    
    # Header
    print(f"\n{'Scenario':<12} | {'Algorithm':<10} | {'Mean Err':>10} | {'Median':>10} | "
          f"{'Std Dev':>10} | {'Max Err':>10} | {'Collisions':>12}")
    print("-"*100)
    
    # Data for each scenario
    for results in all_results:
        scenario = f"S{results['scenario_num']}"
        kf = results['kalman']
        pf = results['particle']
        
        # KF row
        print(f"{scenario:<12} | {'KF':<10} | {kf['mean_error']:>10.4f} | {kf['median_error']:>10.4f} | "
              f"{kf['std_error']:>10.4f} | {kf['max_error']:>10.4f} | {kf['collision_count']:>7d} ({kf['collision_rate']*100:>4.1f}%)")
        
        # PF row
        print(f"{'':<12} | {'PF':<10} | {pf['mean_error']:>10.4f} | {pf['median_error']:>10.4f} | "
              f"{pf['std_error']:>10.4f} | {pf['max_error']:>10.4f} | {pf['collision_count']:>7d} ({pf['collision_rate']*100:>4.1f}%)")
        
        # Winner based on accuracy
        winner = "KF" if kf['mean_error'] < pf['mean_error'] else "PF"
        
        # But note if winner has invalid estimates
        kf_invalid = kf['collision_rate'] > 0.05
        pf_invalid = pf['collision_rate'] > 0.05
        
        if winner == "KF" and kf_invalid and not pf_invalid:
            print(f"{'':<12} | {'→ Winner:':<10} {winner} (but has invalid estimates - mean in obstacles)")
        elif winner == "PF" and pf_invalid and not kf_invalid:
            print(f"{'':<12} | {'→ Winner:':<10} {winner} (but has invalid estimates - mean in obstacles)")
        else:
            print(f"{'':<12} | {'→ Winner:':<10} {winner}")
        print("-"*100)
    
    # Overall statistics
    print(f"\n{'OVERALL STATISTICS':^100}")
    print("-"*100)
    
    kf_wins = sum(1 for r in all_results if r['kalman']['mean_error'] < r['particle']['mean_error'])
    pf_wins = len(all_results) - kf_wins
    
    print(f"  • Kalman Filter Wins (Lower Error): {kf_wins}/{len(all_results)} scenarios")
    print(f"  • Particle Filter Wins (Lower Error): {pf_wins}/{len(all_results)} scenarios")
    
    kf_critical_failures = sum(1 for r in all_results 
                                if r['kalman']['collision_rate'] > 0.05 and 
                                r['kalman']['collision_rate'] > r['particle']['collision_rate'] * 2)
    
    print(f"  • KF Critical Failures (mean in obstacle, PF much better): {kf_critical_failures} scenarios")
    
    # Find the scenario where KF fails but PF succeeds
    failure_scenarios = [r['scenario_num'] for r in all_results 
                        if r['kalman']['collision_rate'] > 0.05 and 
                        r['particle']['collision_rate'] < 0.05]
    
    if failure_scenarios:
        print(f"  • Scenarios where KF FAILS but PF SUCCEEDS: {failure_scenarios}")
    else:
        print(f"  • Note: No clear scenario where KF fails but PF succeeds found")
        print(f"    (May need to adjust bimodal noise parameters)")
    
    print("\n" + "="*100)
    print(" "*20 + "KEY INSIGHT: Compare collision rates vs accuracy in each scenario")
    print("="*100 + "\n")
