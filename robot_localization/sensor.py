"""
sensor.py
---------
Simple noisy location sensor for PR2 robot in PyBullet.

Features:
    - get_true_position()
    - get_noisy_position()
    - configurable noise
    - standalone usage

Usage example:
    from sensor import LocationSensor
    from robot import PR2Robot

    robot = PR2Robot(gui=True)
    sensor = LocationSensor(robot, pos_noise=0.05, theta_noise=0.02)

    for _ in range(1000):
        noisy = sensor.get_noisy_position()
        print("Noisy pose:", noisy)
        robot.step()
"""

import numpy as np

class LocationSensor:
    def __init__(self, robot, pos_noise=0.05, theta_noise=0.02):
        """
        A simple PR2 location sensor simulator.

        Parameters
        ----------
        robot : PR2Robot
            Robot instance this sensor reads from.
        pos_noise : float
            Standard deviation (meters) for noisy position.
        theta_noise : float
            Standard deviation (radians) for noisy heading.
        """
        self.robot = robot
        self.pos_noise = float(pos_noise)
        self.theta_noise = float(theta_noise)

    # -------------------------------------------------
    # True state
    # -------------------------------------------------
    def get_true_position(self):
        """
        Get the exact base pose (x, y, theta) from PyBullet.

        Returns
        -------
        (x, y, theta)
        """
        return self.robot.get_base()

    # -------------------------------------------------
    # Noisy state
    # -------------------------------------------------
    def get_noisy_position(self):
        """
        Get a noisy estimate of robot pose.

        Returns
        -------
        (x_noisy, y_noisy, theta_noisy)
        """
        x, y, theta = self.get_true_position()

        noisy_x = x + np.random.normal(0, self.pos_noise)
        noisy_y = y + np.random.normal(0, self.pos_noise)
        noisy_theta = theta + np.random.normal(0, self.theta_noise)

        return float(noisy_x), float(noisy_y), float(noisy_theta)

    # -------------------------------------------------
    # Convenience
    # -------------------------------------------------
    def set_noise(self, pos_noise=None, theta_noise=None):
        """
        Change sensor noise at runtime.
        """
        if pos_noise is not None:
            self.pos_noise = float(pos_noise)
        if theta_noise is not None:
            self.theta_noise = float(theta_noise)

