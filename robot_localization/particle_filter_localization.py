"""
particle_filter_localization.py
--------------------------------
Particle Filter implementation for robot localization.

State: [x, y, theta] - position and orientation
Represents probability distribution as set of weighted particles
"""

import numpy as np
import math

class ParticleFilterLocalization:
    def __init__(self, n_particles=500, pos_std=0.10, theta_std=0.06):
        """
        Initialize Particle Filter for localization
        
        Parameters
        ----------
        n_particles : int
            Number of particles
        pos_std : float
            Position standard deviation for initialization
        theta_std : float
            Orientation standard deviation for initialization
        """
        self.n = n_particles
        self.pos_std = pos_std
        self.theta_std = theta_std
        
        # Particles: N x 3 array [x, y, theta]
        self.particles = None
        
        # Weights: N x 1 array
        self.weights = np.ones(self.n) / self.n
    
    def init_uniform(self, center, spread=0.5):
        """
        Initialize particles around a center point
        
        Parameters
        ----------
        center : tuple
            (x, y, theta) center point
        spread : float
            Standard deviation for initial distribution
        """
        cx, cy, cth = center
        
        # Sample particles around center
        xs = np.random.normal(cx, spread, size=self.n)
        ys = np.random.normal(cy, spread, size=self.n)
        thetas = np.random.normal(cth, 0.4, size=self.n)
        
        self.particles = np.vstack([xs, ys, thetas]).T
        self.weights[:] = 1.0 / self.n
    
    def predict(self, control, control_std=(0.03, 0.03, 0.02)):
        """
        Prediction step: propagate particles according to motion model
        
        Parameters
        ----------
        control : tuple
            (dx, dy, dtheta) - control input
        control_std : tuple
            (std_dx, std_dy, std_dtheta) - control noise
        """
        dx_c, dy_c, dth_c = control
        std_dx, std_dy, std_dth = control_std
        
        # Add noisy control to each particle
        dx = np.random.normal(dx_c, std_dx, size=self.n)
        dy = np.random.normal(dy_c, std_dy, size=self.n)
        dth = np.random.normal(dth_c, std_dth, size=self.n)
        
        self.particles[:, 0] += dx
        self.particles[:, 1] += dy
        self.particles[:, 2] += dth
    
    def update(self, measurement, meas_std):
        """
        Update step: reweight particles based on measurement likelihood
        
        Parameters
        ----------
        measurement : tuple
            (x_meas, y_meas) - measured position
        meas_std : float
            Measurement noise standard deviation
        """
        mx, my = measurement
        
        # Compute likelihood for each particle
        # Using Gaussian likelihood on position
        dx = self.particles[:, 0] - mx
        dy = self.particles[:, 1] - my
        d_squared = dx**2 + dy**2
        
        sigma_squared = meas_std ** 2
        
        # Gaussian likelihood
        weights = (1.0 / (2 * np.pi * sigma_squared)) * \
                  np.exp(-0.5 * d_squared / sigma_squared)
        
        # Add small constant to avoid numerical issues
        weights += 1e-12
        
        # Normalize weights
        self.weights = weights / np.sum(weights)
    
    def resample(self):
        """
        Resample particles based on weights (systematic resampling)
        """
        n = self.n
        
        # Systematic resampling
        positions = (np.arange(n) + np.random.random()) / n
        
        cumulative_sum = np.cumsum(self.weights)
        
        indexes = np.zeros(n, dtype=int)
        i, j = 0, 0
        
        while i < n:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        
        # Replace particles
        self.particles = self.particles[indexes]
        
        # Reset weights to uniform
        self.weights.fill(1.0 / n)
    
    def estimate(self):
        """
        Compute weighted mean estimate of state
        
        Returns
        -------
        (x, y, theta) : tuple
            Estimated state
        """
        # Weighted mean for x and y
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)
        
        # Circular mean for theta
        sin_sum = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        cos_sum = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        theta = math.atan2(sin_sum, cos_sum)
        
        return (float(x), float(y), float(theta))
    
    def get_particles(self):
        """
        Get current particles
        
        Returns
        -------
        particles : np.array (N, 3)
            Current particle positions and orientations
        weights : np.array (N,)
            Current particle weights
        """
        return self.particles.copy(), self.weights.copy()
    
    def effective_sample_size(self):
        """
        Compute effective sample size (ESS)
        
        Returns
        -------
        float
            Effective number of particles
        """
        return 1.0 / np.sum(self.weights ** 2)
