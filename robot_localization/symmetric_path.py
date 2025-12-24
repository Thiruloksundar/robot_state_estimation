"""
symmetric_path.py
-----------------
Path planning for symmetric hallway environment
"""

import numpy as np
import pybullet as p

class SymmetricPathPlanner:
    def __init__(self, environment):
        """Initialize path planner for symmetric environment"""
        self.env = environment
        self.corridor_width = environment.corridor_width
        self.wall_thickness = environment.wall_thickness
        self.corridor_length = environment.corridor_length
        
        # Corridor A center (where robot actually is)
        self.corridor_a_y = self.corridor_width/2 + self.wall_thickness/2
        self.corridor_b_y = -(self.corridor_width/2 + self.wall_thickness/2)
    
    def get_symmetric_path(self):
        """
        Create a simple path through Corridor A
        
        Robot moves from left to right through Corridor A.
        Due to symmetry, sensor readings are ambiguous.
        
        Returns
        -------
        list of [x, y] waypoints
        """
        waypoints = []
        
        # Path through Corridor A (where robot actually is)
        y_pos = self.corridor_a_y
        
        # Move from left to right
        x_positions = np.linspace(-8, 8, 40)
        
        for x in x_positions:
            waypoints.append([x, y_pos])
        
        return waypoints
    
    def visualize_path(self, waypoints, color=[1, 0, 0], line_width=3):
        """Draw the path"""
        print(f"Visualizing symmetric path with {len(waypoints)} waypoints")
        
        for i in range(len(waypoints) - 1):
            start = [waypoints[i][0], waypoints[i][1], 0.1]
            end = [waypoints[i+1][0], waypoints[i+1][1], 0.1]
            p.addUserDebugLine(start, end, color, line_width)
        
        # Mark corridor centers for visualization
        p.addUserDebugLine([-10, self.corridor_a_y, 0.5], [10, self.corridor_a_y, 0.5],
                          [0, 1, 0], 2)  # Corridor A (green)
        p.addUserDebugLine([-10, self.corridor_b_y, 0.5], [10, self.corridor_b_y, 0.5],
                          [0, 0, 1], 2)  # Corridor B (blue)


def test_symmetric_path():
    """Test path planning"""
    from symmetric_environment import SymmetricEnvironment
    
    env = SymmetricEnvironment(use_gui=True)
    planner = SymmetricPathPlanner(env)
    
    waypoints = planner.get_symmetric_path()
    planner.visualize_path(waypoints)
    
    print(f"\nPath through Corridor A (green line)")
    print(f"Robot will move from x=-8 to x=8")
    print(f"Sensor readings are AMBIGUOUS (same in both corridors)")
    
    input("\nPress Enter to close...")
    env.cleanup()


if __name__ == "__main__":
    test_symmetric_path()
