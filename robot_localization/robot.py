# robot.py
"""
robot.py
--------
Minimal PR2 robot interface for PyBullet.

Features:
- Connect/disconnect
- Load real PR2 robot (DRAKE_PR2_URDF)
- Set base pose
- Get base pose
- Step simulation

Usage:
    from robot import PR2Robot

    r = PR2Robot(gui=True)
    r.set_base(1.0, 0.0, 1.57)
    print(r.get_base())
    r.step()
    r.disconnect()
"""

import pybullet as p
import pybullet_data
import time
import math

from pybullet_tools.utils import (
    connect, disconnect, set_point, get_pose, set_pose, HideOutput, wait_if_gui
)
from pybullet_tools.pr2_utils import DRAKE_PR2_URDF


class PR2Robot:

    def __init__(self, gui=True):
        """
        Create a PyBullet session and load the PR2 model.
        gui = True → p.GUI
        gui = False → p.DIRECT
        """
        self.client = connect(use_gui=gui)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        with HideOutput():
            self.robot = p.loadURDF(DRAKE_PR2_URDF, basePosition=[0,0,0.08], useFixedBase=False)

        # store useful robot data
        self.last_base_pose = (0.0, 0.0, 0.0)

    # -------------------------------------------------
    # Base control
    # -------------------------------------------------
    def set_base(self, x, y, theta):
        """
        Sets PR2 base in world.
        (x, y) in meters, theta in radians.
        """
        pos = [float(x), float(y), 0.08]
        orn = p.getQuaternionFromEuler([0, 0, float(theta)])
        p.resetBasePositionAndOrientation(self.robot, pos, orn)
        self.last_base_pose = (x, y, theta)

    def get_base(self):
        """
        Returns (x, y, theta) tuple.
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        euler = p.getEulerFromQuaternion(orn)
        x, y, _ = pos
        _, _, theta = euler
        return (float(x), float(y), float(theta))

    # -------------------------------------------------
    # Simulation control
    # -------------------------------------------------
    def step(self):
        """Step PyBullet simulation by one timestep."""
        p.stepSimulation()

    def sleep(self, dt=0.01):
        """Convenience method for slow stepping."""
        time.sleep(dt)

    def disconnect(self):
        """Close pybullet window."""
        wait_if_gui('Closing PR2 robot simulation.')
        disconnect()

    # -------------------------------------------------
    # Optional convenience methods
    # -------------------------------------------------
    def move_base_by(self, dx, dy, dtheta):
        """
        Increment base pose relative to current pose.
        """
        x, y, th = self.get_base()
        self.set_base(x + dx, y + dy, th + dtheta)


# -----------------------------------------------------
# Small demo (run only if executed directly)
# -----------------------------------------------------
if __name__ == "__main__":
    r = PR2Robot(gui=True)

    # sweep a small circle
    for i in range(200):
        x = 0.5 * math.cos(i * 0.05)
        y = 0.5 * math.sin(i * 0.05)
        r.set_base(x, y, i * 0.05)
        r.step()
        r.sleep(0.01)

    r.disconnect()

