"""
sensor_improved.py
------------------
Enhanced location sensor with multiple noise models and configurable parameters.

This sensor allows you to tune noise levels to make the localization problem
interesting but not impossible.

Noise Models:
1. Gaussian noise (default) - KF assumes this
2. Non-Gaussian noise - Heavy-tailed (causes KF to fail)
3. Bimodal noise - Multi-hypothesis (causes KF to fail)
4. Distance-dependent noise - More noise further from reference
"""

import numpy as np
import pybullet as p

class ImprovedLocationSensor:
    def __init__(self, robot, 
                 pos_noise_std=0.1,           # Position noise standard deviation
                 theta_noise_std=0.05,        # Orientation noise standard deviation
                 noise_model='gaussian',      # 'gaussian', 'heavy_tailed', 'bimodal', 'distance_dependent'
                 outlier_probability=0.0,     # Probability of getting a bad measurement
                 bimodal_separation=0.5):     # Separation for bimodal noise
        """
        Enhanced location sensor with multiple noise models.
        
        Parameters
        ----------
        robot : PR2Robot
            Robot instance to read true position from
        pos_noise_std : float
            Standard deviation for position noise (meters)
            - LOW (easy): 0.02-0.05
            - MEDIUM (interesting): 0.08-0.15
            - HIGH (challenging): 0.2-0.4
        theta_noise_std : float
            Standard deviation for orientation noise (radians)
            - LOW: 0.01-0.03
            - MEDIUM: 0.05-0.1
            - HIGH: 0.15-0.3
        noise_model : str
            Type of noise distribution
            - 'gaussian': Normal distribution (KF works well)
            - 'heavy_tailed': Occasional large errors (KF struggles)
            - 'bimodal': Two possible readings (KF fails, PF succeeds)
            - 'distance_dependent': Noise increases with distance
        outlier_probability : float
            For heavy_tailed model: probability of large error (0.0-0.3)
        bimodal_separation : float
            For bimodal model: distance between two modes (meters)
        """
        self.robot = robot
        self.pos_noise_std = float(pos_noise_std)
        self.theta_noise_std = float(theta_noise_std)
        self.noise_model = noise_model
        self.outlier_probability = float(outlier_probability)
        self.bimodal_separation = float(bimodal_separation)
        
        # For distance-dependent noise
        self.reference_point = [0.0, 0.0]  # Origin
        
        # Statistics tracking
        self.measurement_count = 0
        self.outlier_count = 0
        
    def get_true_position(self):
        """Get exact robot position from simulation"""
        return self.robot.get_base()
    
    def get_noisy_position(self):
        """
        Get noisy sensor reading based on configured noise model
        
        Returns
        -------
        (x_noisy, y_noisy, theta_noisy) : tuple
            Noisy position estimate
        """
        self.measurement_count += 1
        x_true, y_true, theta_true = self.get_true_position()
        
        if self.noise_model == 'gaussian':
            return self._gaussian_noise(x_true, y_true, theta_true)
        
        elif self.noise_model == 'heavy_tailed':
            return self._heavy_tailed_noise(x_true, y_true, theta_true)
        
        elif self.noise_model == 'bimodal':
            return self._bimodal_noise(x_true, y_true, theta_true)
        
        elif self.noise_model == 'distance_dependent':
            return self._distance_dependent_noise(x_true, y_true, theta_true)
        
        else:
            raise ValueError(f"Unknown noise model: {self.noise_model}")
    
    def _gaussian_noise(self, x, y, theta):
        """Standard Gaussian noise (what Kalman Filter assumes)"""
        noise_x = np.random.normal(0, self.pos_noise_std)
        noise_y = np.random.normal(0, self.pos_noise_std)
        noise_theta = np.random.normal(0, self.theta_noise_std)
        
        return (x + noise_x, y + noise_y, theta + noise_theta)
    
    def _heavy_tailed_noise(self, x, y, theta):
        """
        Heavy-tailed noise: mostly Gaussian but occasional large outliers.
        This breaks Kalman Filter assumptions!
        """
        if np.random.random() < self.outlier_probability:
            # Large outlier (10x normal noise)
            noise_x = np.random.normal(0, self.pos_noise_std * 10)
            noise_y = np.random.normal(0, self.pos_noise_std * 10)
            noise_theta = np.random.normal(0, self.theta_noise_std * 5)
            self.outlier_count += 1
        else:
            # Normal Gaussian noise
            noise_x = np.random.normal(0, self.pos_noise_std)
            noise_y = np.random.normal(0, self.pos_noise_std)
            noise_theta = np.random.normal(0, self.theta_noise_std)
        
        return (x + noise_x, y + noise_y, theta + noise_theta)
    
    def _bimodal_noise(self, x, y, theta):
        """
        Bimodal noise: measurement could be from one of two modes.
        This creates ambiguity - perfect for showing PF advantage!
        
        Example: robot near a doorway - sensor might report position
        in either of two adjacent rooms.
        """
        # Randomly choose which mode
        if np.random.random() < 0.5:
            # Mode 1: offset in one direction
            offset_x = self.bimodal_separation
            offset_y = 0.0
        else:
            # Mode 2: offset in opposite direction
            offset_x = -self.bimodal_separation
            offset_y = 0.0
        
        # Add Gaussian noise on top of bimodal offset
        noise_x = np.random.normal(offset_x, self.pos_noise_std)
        noise_y = np.random.normal(offset_y, self.pos_noise_std)
        noise_theta = np.random.normal(0, self.theta_noise_std)
        
        return (x + noise_x, y + noise_y, theta + noise_theta)
    
    def _distance_dependent_noise(self, x, y, theta):
        """
        Noise increases with distance from reference point.
        Simulates sensors like GPS that are less accurate far from base station.
        """
        # Calculate distance from reference point
        dx = x - self.reference_point[0]
        dy = y - self.reference_point[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # Noise scales with distance (minimum is base noise)
        distance_scale = 1.0 + distance * 0.1  # 10% increase per meter
        effective_pos_noise = self.pos_noise_std * distance_scale
        effective_theta_noise = self.theta_noise_std * distance_scale
        
        noise_x = np.random.normal(0, effective_pos_noise)
        noise_y = np.random.normal(0, effective_pos_noise)
        noise_theta = np.random.normal(0, effective_theta_noise)
        
        return (x + noise_x, y + noise_y, theta + noise_theta)
    
    def set_noise_model(self, noise_model, **kwargs):
        """
        Change noise model at runtime
        
        Parameters
        ----------
        noise_model : str
            New noise model type
        **kwargs : dict
            Additional parameters (pos_noise_std, theta_noise_std, etc.)
        """
        self.noise_model = noise_model
        
        if 'pos_noise_std' in kwargs:
            self.pos_noise_std = kwargs['pos_noise_std']
        if 'theta_noise_std' in kwargs:
            self.theta_noise_std = kwargs['theta_noise_std']
        if 'outlier_probability' in kwargs:
            self.outlier_probability = kwargs['outlier_probability']
        if 'bimodal_separation' in kwargs:
            self.bimodal_separation = kwargs['bimodal_separation']
    
    def get_statistics(self):
        """Return sensor statistics"""
        outlier_rate = self.outlier_count / max(1, self.measurement_count)
        return {
            'measurement_count': self.measurement_count,
            'outlier_count': self.outlier_count,
            'outlier_rate': outlier_rate,
            'noise_model': self.noise_model,
            'pos_noise_std': self.pos_noise_std,
            'theta_noise_std': self.theta_noise_std
        }
    
    def print_statistics(self):
        """Print sensor statistics"""
        stats = self.get_statistics()
        print("\n" + "="*60)
        print("SENSOR STATISTICS")
        print("="*60)
        print(f"Noise Model:        {stats['noise_model']}")
        print(f"Position Noise:     {stats['pos_noise_std']:.4f} m")
        print(f"Orientation Noise:  {stats['theta_noise_std']:.4f} rad")
        print(f"Total Measurements: {stats['measurement_count']}")
        if self.noise_model == 'heavy_tailed':
            print(f"Outlier Count:      {stats['outlier_count']}")
            print(f"Outlier Rate:       {stats['outlier_rate']:.2%}")
        print("="*60)


# Preset sensor configurations for different scenarios
class SensorPresets:
    """Predefined sensor configurations for different scenarios"""
    
    @staticmethod
    def easy_gaussian():
        """Easy scenario: low Gaussian noise"""
        return {
            'pos_noise_std': 0.03,
            'theta_noise_std': 0.02,
            'noise_model': 'gaussian'
        }
    
    @staticmethod
    def medium_gaussian():
        """Medium scenario: moderate Gaussian noise (interesting)"""
        return {
            'pos_noise_std': 0.12,
            'theta_noise_std': 0.08,
            'noise_model': 'gaussian'
        }
    
    @staticmethod
    def hard_gaussian():
        """Hard scenario: high Gaussian noise"""
        return {
            'pos_noise_std': 0.25,
            'theta_noise_std': 0.15,
            'noise_model': 'gaussian'
        }
    
    @staticmethod
    def heavy_tailed_kf_failure():
        """
        KF failure scenario: occasional large outliers
        Kalman Filter will struggle, Particle Filter handles better
        """
        return {
            'pos_noise_std': 0.08,
            'theta_noise_std': 0.05,
            'noise_model': 'heavy_tailed',
            'outlier_probability': 0.15  # 15% chance of bad measurement
        }
    
    @staticmethod
    def bimodal_kf_failure():
        """
        KF failure scenario: bimodal measurements
        Perfect case where PF works but KF fails completely!
        The mean of KF might end up in a wall between the two modes.
        """
        return {
            'pos_noise_std': 0.08,
            'theta_noise_std': 0.05,
            'noise_model': 'bimodal',
            'bimodal_separation': 1.2  # 1.2m between modes
        }
    
    @staticmethod
    def distance_dependent():
        """GPS-like sensor that gets worse with distance"""
        return {
            'pos_noise_std': 0.05,
            'theta_noise_std': 0.03,
            'noise_model': 'distance_dependent'
        }


def test_sensor_models():
    """Test different sensor noise models"""
    from robot import PR2Robot
    import matplotlib.pyplot as plt
    
    print("Testing different sensor noise models...")
    
    robot = PR2Robot(gui=False)
    robot.set_base(0, 0, 0)
    
    # Test each noise model
    models = ['gaussian', 'heavy_tailed', 'bimodal', 'distance_dependent']
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.flatten()
    
    for idx, model in enumerate(models):
        sensor = ImprovedLocationSensor(
            robot,
            pos_noise_std=0.1,
            theta_noise_std=0.05,
            noise_model=model,
            outlier_probability=0.2,
            bimodal_separation=0.8
        )
        
        # Collect measurements
        measurements = []
        for _ in range(500):
            noisy = sensor.get_noisy_position()
            measurements.append([noisy[0], noisy[1]])
        
        measurements = np.array(measurements)
        
        # Plot
        axes[idx].scatter(measurements[:, 0], measurements[:, 1], 
                         alpha=0.3, s=10)
        axes[idx].scatter(0, 0, c='red', s=100, marker='x', 
                         label='True Position')
        axes[idx].set_title(f'{model.replace("_", " ").title()} Noise')
        axes[idx].set_xlabel('X (m)')
        axes[idx].set_ylabel('Y (m)')
        axes[idx].legend()
        axes[idx].grid(True, alpha=0.3)
        axes[idx].axis('equal')
        
        sensor.print_statistics()
    
    plt.tight_layout()
    plt.savefig('sensor_noise_models.png', dpi=150)
    print("\nSaved plot to 'sensor_noise_models.png'")
    plt.show()
    
    robot.disconnect()


if __name__ == "__main__":
    test_sensor_models()
