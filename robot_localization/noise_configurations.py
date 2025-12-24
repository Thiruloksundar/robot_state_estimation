"""
noise_configurations.py
-----------------------
Recommended noise configurations for different experimental scenarios.
These settings make the localization problem interesting but solvable.
"""

class NoiseConfigurations:
    """
    Noise parameter configurations for different scenarios.
    
    Each configuration specifies:
    - Sensing noise (sensor measurement uncertainty)
    - Action noise (motion/process uncertainty)
    - Expected behavior of KF and PF
    """
    
    @staticmethod
    def scenario_1_both_work():
        """
        Scenario 1: Normal Gaussian Noise - Both Filters Work Well
        
        Purpose: Baseline comparison showing both filters can handle
        moderate Gaussian noise.
        
        Expected Result:
        - KF: Works well, optimal for Gaussian noise
        - PF: Works well, slightly more computation
        - Similar accuracy
        """
        return {
            'name': 'Scenario 1: Both Filters Work',
            'description': 'Moderate Gaussian noise - baseline comparison',
            
            # Sensor noise
            'sensor': {
                'pos_noise_std': 0.10,      # 10cm position uncertainty
                'theta_noise_std': 0.06,    # ~3.4° orientation uncertainty
                'noise_model': 'gaussian',
            },
            
            # Action/Motion noise
            'action': {
                'process_std': 0.03,        # 3cm motion uncertainty
                'control_std': (0.03, 0.03, 0.02)  # (dx, dy, dtheta)
            },
            
            # Filter parameters
            'kalman': {
                'process_std': 0.03,
                'meas_std': 0.10
            },
            'particle': {
                'n_particles': 400,
                'pos_std': 0.10,
                'theta_std': 0.06
            }
        }
    
    @staticmethod
    def scenario_2_high_noise():
        """
        Scenario 2: High Gaussian Noise - Robustness Test
        
        Purpose: Test filter robustness with challenging noise levels
        
        Expected Result:
        - KF: Still works but higher error
        - PF: More robust due to non-parametric nature
        - PF slightly outperforms KF
        """
        return {
            'name': 'Scenario 2: High Noise Robustness',
            'description': 'High Gaussian noise - testing limits',
            
            'sensor': {
                'pos_noise_std': 0.20,      # 20cm position uncertainty
                'theta_noise_std': 0.12,    # ~6.9° orientation uncertainty
                'noise_model': 'gaussian',
            },
            
            'action': {
                'process_std': 0.06,
                'control_std': (0.06, 0.06, 0.04)
            },
            
            'kalman': {
                'process_std': 0.06,
                'meas_std': 0.20
            },
            'particle': {
                'n_particles': 600,  # More particles for higher noise
                'pos_std': 0.20,
                'theta_std': 0.12
            }
        }
    
    @staticmethod
    def scenario_3_heavy_tailed():
        """
        Scenario 3: Heavy-Tailed Noise - KF Struggles, PF Succeeds
        
        Purpose: Show KF weakness with non-Gaussian noise (outliers)
        
        Expected Result:
        - KF: Badly affected by outliers, large errors
        - PF: Handles outliers naturally, remains stable
        - **Clear advantage for PF**
        """
        return {
            'name': 'Scenario 3: Heavy-Tailed Noise (KF Struggles)',
            'description': 'Occasional large outliers break KF assumptions',
            
            'sensor': {
                'pos_noise_std': 0.10,
                'theta_noise_std': 0.06,
                'noise_model': 'heavy_tailed',
                'outlier_probability': 0.15  # 15% chance of bad measurement
            },
            
            'action': {
                'process_std': 0.03,
                'control_std': (0.03, 0.03, 0.02)
            },
            
            'kalman': {
                'process_std': 0.03,
                'meas_std': 0.10  # Doesn't know about outliers!
            },
            'particle': {
                'n_particles': 500,
                'pos_std': 0.10,
                'theta_std': 0.06
            }
        }
    
    @staticmethod
    def scenario_4_bimodal_failure():
        """
        Scenario 4: BIMODAL NOISE - KF COMPLETE FAILURE, PF SUCCESS
        
        Purpose: Create the REQUIRED failure case where KF mean goes
        inside obstacle but PF maintains correct estimate.
        
        This is THE KEY SCENARIO for your project!
        
        Expected Result:
        - KF: Mean estimate can be inside walls/obstacles (between two modes)
        - PF: Maintains multiple hypotheses, converges to correct mode
        - **Dramatic failure for KF, success for PF**
        
        Best location: Near doorways where ambiguity exists
        (Could be in room A or room B)
        """
        return {
            'name': 'Scenario 4: BIMODAL - KF FAILURE (OBSTACLE)',
            'description': 'Multimodal uncertainty causes KF mean in obstacle',
            
            'sensor': {
                'pos_noise_std': 0.08,
                'theta_noise_std': 0.05,
                'noise_model': 'bimodal',
                'bimodal_separation': 1.0  # 1.0m between two modes (reduced from 1.5m)
            },
            
            'action': {
                'process_std': 0.02,  # Lower motion noise helps PF converge faster
                'control_std': (0.02, 0.02, 0.01)
            },
            
            'kalman': {
                'process_std': 0.02,
                'meas_std': 0.08  # Can't represent bimodality!
            },
            'particle': {
                'n_particles': 1000,  # More particles for better convergence
                'pos_std': 0.08,
                'theta_std': 0.05
            },
            
            # Special note for this scenario
            'special_notes': [
                'Robot should traverse doorways slowly',
                'Bimodal measurements simulate ambiguity',
                'KF mean will average between modes → inside wall!',
                'PF maintains separate hypotheses for each room'
            ]
        }
    
    @staticmethod
    def scenario_5_distance_dependent():
        """
        Scenario 5: Distance-Dependent Noise - GPS-like Sensor
        
        Purpose: Realistic sensor where accuracy degrades with distance
        
        Expected Result:
        - Both filters work but accuracy varies with location
        - Interesting for showing adaptive behavior
        """
        return {
            'name': 'Scenario 5: Distance-Dependent Noise',
            'description': 'Noise increases with distance from origin',
            
            'sensor': {
                'pos_noise_std': 0.05,  # Base noise
                'theta_noise_std': 0.03,
                'noise_model': 'distance_dependent',
            },
            
            'action': {
                'process_std': 0.03,
                'control_std': (0.03, 0.03, 0.02)
            },
            
            'kalman': {
                'process_std': 0.03,
                'meas_std': 0.10  # Average across distances
            },
            'particle': {
                'n_particles': 500,
                'pos_std': 0.10,
                'theta_std': 0.06
            }
        }
    
    @staticmethod
    def get_all_scenarios():
        """Get all scenario configurations"""
        return [
            NoiseConfigurations.scenario_1_both_work(),
            NoiseConfigurations.scenario_2_high_noise(),
            NoiseConfigurations.scenario_3_heavy_tailed(),
            NoiseConfigurations.scenario_4_bimodal_failure(),
            NoiseConfigurations.scenario_5_distance_dependent()
        ]
    
    @staticmethod
    def print_scenario(scenario):
        """Pretty print a scenario configuration"""
        print("\n" + "="*70)
        print(f"  {scenario['name']}")
        print("="*70)
        print(f"Description: {scenario['description']}")
        print("\nSensor Configuration:")
        for key, value in scenario['sensor'].items():
            print(f"  {key:20s}: {value}")
        print("\nAction/Motion Configuration:")
        for key, value in scenario['action'].items():
            print(f"  {key:20s}: {value}")
        print("\nKalman Filter Parameters:")
        for key, value in scenario['kalman'].items():
            print(f"  {key:20s}: {value}")
        print("\nParticle Filter Parameters:")
        for key, value in scenario['particle'].items():
            print(f"  {key:20s}: {value}")
        
        if 'special_notes' in scenario:
            print("\nSpecial Notes:")
            for note in scenario['special_notes']:
                print(f"  • {note}")
        print("="*70)
    
    @staticmethod
    def print_all_scenarios():
        """Print all scenarios"""
        scenarios = NoiseConfigurations.get_all_scenarios()
        print("\n" + "#"*70)
        print("#  NOISE CONFIGURATIONS FOR LOCALIZATION EXPERIMENTS")
        print("#"*70)
        
        for scenario in scenarios:
            NoiseConfigurations.print_scenario(scenario)


if __name__ == "__main__":
    # Print all scenarios
    NoiseConfigurations.print_all_scenarios()
    
    print("\n" + "#"*70)
    print("#  RECOMMENDATION FOR YOUR PROJECT:")
    print("#"*70)
    print("""
Run experiments with scenarios 1, 3, and 4:

1. Scenario 1: Shows baseline - both filters work
2. Scenario 3: Shows KF struggles with outliers, PF is robust  
3. Scenario 4: Shows CRITICAL FAILURE CASE
   - KF mean ends up INSIDE OBSTACLE
   - PF maintains correct estimate
   - This is your MAIN result!

For your report, include:
- All sensor and action noise parameters used
- Plots comparing KF vs PF accuracy over time
- Screenshots showing KF mean inside obstacle (Scenario 4)
- Error statistics for each scenario
    """)
    print("#"*70)
