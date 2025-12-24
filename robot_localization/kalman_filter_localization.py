"""
kalman_filter_localization.py
------------------------------
Kalman Filter implementation for robot localization.

State: [x, y, vx, vy] - position and velocity
Measurement: [x, y] - noisy position readings
"""

import numpy as np

class KalmanFilterLocalization:
    def __init__(self, dt=0.05, process_std=0.03, meas_std=0.10):
        """
        Initialize Kalman Filter for localization
        
        Parameters
        ----------
        dt : float
            Time step (seconds)
        process_std : float
            Process noise standard deviation (motion uncertainty)
        meas_std : float
            Measurement noise standard deviation (sensor uncertainty)
        """
        self.dt = dt
        
        # State: [x, y, vx, vy]
        self.x = np.zeros((4, 1))
        
        # Covariance matrix
        self.P = np.eye(4) * 1.0
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        
        # Process noise covariance
        self.Q = np.eye(4) * (process_std ** 2)
        
        # Measurement matrix (we measure x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Measurement noise covariance
        self.R = np.eye(2) * (meas_std ** 2)
    
    def predict(self):
        """Prediction step: propagate state and covariance forward"""
        # Predicted state
        self.x = self.F @ self.x
        
        # Predicted covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, z):
        """
        Update step: incorporate new measurement
        
        Parameters
        ----------
        z : np.array (2, 1)
            Measurement vector [x_measured, y_measured]
        """
        # Innovation (measurement residual)
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P
    
    def set_pose(self, x, y, vx=0.0, vy=0.0):
        """
        Set the state to a specific pose
        
        Parameters
        ----------
        x, y : float
            Position
        vx, vy : float
            Velocity
        """
        self.x = np.array([[x], [y], [vx], [vy]])
    
    def get_estimate(self):
        """
        Get current position estimate
        
        Returns
        -------
        (x, y) : tuple
            Estimated position
        """
        return (float(self.x[0, 0]), float(self.x[1, 0]))
    
    def get_covariance(self):
        """
        Get position covariance
        
        Returns
        -------
        np.array (2, 2)
            Position covariance matrix
        """
        # Extract position covariance (top-left 2x2 block)
        return self.P[:2, :2].copy()
