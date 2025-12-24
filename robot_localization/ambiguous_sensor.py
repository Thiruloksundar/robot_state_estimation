"""
ambiguous_sensor.py
-------------------
Distance-based sensor that CANNOT distinguish between symmetric corridors.

The sensor reports "distance to nearest wall" which is the SAME
in both Corridor A and Corridor B, creating ambiguity.
"""

import numpy as np

class AmbiguousDistanceSensor:
    def __init__(self, robot, environment, pos_noise=0.15, distance_noise=0.1):
        """
        Ambiguous sensor that measures distance to walls
        
        Parameters
        ----------
        robot : PR2Robot
            Robot instance
        environment : SymmetricEnvironment
            The symmetric environment
        pos_noise : float
            Noise in position estimate
        distance_noise : float
            Noise in distance measurement
        """
        self.robot = robot
        self.env = environment
        self.pos_noise = pos_noise
        self.distance_noise = distance_noise
        
        # Corridor parameters (for computing "ambiguous" measurements)
        self.corridor_width = environment.corridor_width
        self.wall_thickness = environment.wall_thickness
        self.corridor_a_y = environment.corridor_width/2 + environment.wall_thickness/2
        self.corridor_b_y = -(environment.corridor_width/2 + environment.wall_thickness/2)
    
    def get_true_position(self):
        """Get exact robot position"""
        return self.robot.get_base()
    
    def get_noisy_position(self):
        """
        Get noisy position estimate that creates AMBIGUITY
        
        Key trick: We report position as if it could be in EITHER corridor.
        Sometimes report as if in Corridor A, sometimes as if in Corridor B.
        
        This creates a bimodal distribution where KF will average
        the two corridors → mean ends up in the central wall!
        
        Returns
        -------
        (x_noisy, y_noisy, theta_noisy) : tuple
        """
        x_true, y_true, theta_true = self.get_true_position()
        
        # Add noise to x and theta (these are unambiguous)
        noisy_x = x_true + np.random.normal(0, self.pos_noise)
        noisy_theta = theta_true + np.random.normal(0, 0.05)
        
        # KEY AMBIGUITY: Create bimodal measurement in Y direction
        # Robot is actually in Corridor A, but sensor sometimes reports
        # as if in Corridor B (symmetric position)
        
        # Calculate distance from robot to center of its actual corridor
        distance_from_center = abs(y_true - self.corridor_a_y)
        
        # With 50% probability, "flip" to the symmetric corridor
        if np.random.random() < 0.5:
            # Report as if in Corridor A (actual position)
            noisy_y = self.corridor_a_y + np.random.normal(0, self.pos_noise)
        else:
            # Report as if in Corridor B (symmetric position) - AMBIGUITY!
            noisy_y = self.corridor_b_y + np.random.normal(0, self.pos_noise)
        
        return (float(noisy_x), float(noisy_y), float(noisy_theta))
    
    def get_distance_to_walls(self):
        """
        Get distance to nearest walls (same in both corridors - this is the ambiguity!)
        
        Returns
        -------
        dict with distances to walls
        """
        x, y, _ = self.get_true_position()
        
        # Distance to side walls (same in both corridors!)
        if y > 0:  # Corridor A
            distance_to_outer = (self.corridor_a_y + self.corridor_width/2) - y
            distance_to_center = y - (self.corridor_a_y - self.corridor_width/2)
        else:  # Corridor B (symmetric!)
            distance_to_outer = y - (self.corridor_b_y - self.corridor_width/2)
            distance_to_center = (self.corridor_b_y + self.corridor_width/2) - y
        
        # Add noise
        distance_to_outer += np.random.normal(0, self.distance_noise)
        distance_to_center += np.random.normal(0, self.distance_noise)
        
        return {
            'outer_wall': distance_to_outer,
            'center_wall': distance_to_center
        }
