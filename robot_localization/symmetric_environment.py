"""
symmetric_environment.py
------------------------
Symmetric hallway environment designed to break Kalman Filter.

Two identical parallel corridors separated by a wall.
The sensor cannot distinguish between corridor A and B.
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
from robot import PR2Robot

class SymmetricEnvironment:
    """
    Symmetric Hallway Environment
    
    Layout:
    ┌────────────────┐
    │  Corridor A    │ ← Robot starts here
    ├────────────────┤
    │   WALL         │ ← Thick obstacle wall
    ├────────────────┤
    │  Corridor B    │ ← Identical to A
    └────────────────┘
    
    Key feature: Sensor readings are ambiguous (same distance to walls in both corridors)
    """
    
    def __init__(self, use_gui=True):
        self.robot = PR2Robot(gui=use_gui)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Symmetric hallway dimensions
        self.corridor_width = 4.0    # Width of each corridor
        self.corridor_length = 20.0  # Length of corridors
        self.wall_thickness = 1.5    # Thick central wall (obstacle)
        self.wall_height = 2.5
        
        # Storage
        self.walls = []
        self.obstacles = []
        
        # Build symmetric environment
        self.build_symmetric_hallways()
        self.add_identical_furniture()
        self.position_robot()
        
        # Camera view
        p.resetDebugVisualizerCamera(
            cameraDistance=15,
            cameraYaw=90,
            cameraPitch=-45,
            cameraTargetPosition=[0, 0, 0]
        )
    
    def build_symmetric_hallways(self):
        """Build two identical parallel corridors with central wall"""
        wt = self.wall_thickness
        wh = self.wall_height
        cw = self.corridor_width
        cl = self.corridor_length
        
        print("Building symmetric hallways...")
        
        # Corridor A is at y = +corridor_width/2 + wall_thickness/2
        # Corridor B is at y = -corridor_width/2 - wall_thickness/2
        
        corridor_a_y = cw/2 + wt/2
        corridor_b_y = -(cw/2 + wt/2)
        
        # CENTRAL WALL (the obstacle that KF mean will end up in)
        self.create_wall([0, 0, wh/2], [cl/2, wt/2, wh/2], 
                        color=[0.3, 0.3, 0.3, 1], name="central_wall")
        
        # OUTER WALLS - Corridor A (top)
        self.create_wall([0, corridor_a_y + cw/2, wh/2], 
                        [cl/2, 0.1, wh/2], color=[0.8, 0.8, 0.8, 1])
        
        # OUTER WALLS - Corridor B (bottom)
        self.create_wall([0, corridor_b_y - cw/2, wh/2], 
                        [cl/2, 0.1, wh/2], color=[0.8, 0.8, 0.8, 1])
        
        # END WALLS (both sides)
        total_height = corridor_a_y + cw/2 - (corridor_b_y - cw/2)
        self.create_wall([cl/2, 0, wh/2], 
                        [0.1, total_height/2, wh/2], color=[0.8, 0.8, 0.8, 1])
        self.create_wall([-cl/2, 0, wh/2], 
                        [0.1, total_height/2, wh/2], color=[0.8, 0.8, 0.8, 1])
        
        print(f"Symmetric hallways complete! Central wall at y=0")
    
    def add_identical_furniture(self):
        """Add IDENTICAL obstacles in both corridors for symmetry"""
        cw = self.corridor_width
        wt = self.wall_thickness
        
        corridor_a_y = cw/2 + wt/2
        corridor_b_y = -(cw/2 + wt/2)
        
        # Identical box positions (mirrored across y=0)
        furniture_x_positions = [-6, -3, 0, 3, 6]
        
        for x in furniture_x_positions:
            # Corridor A furniture
            self.create_obstacle(
                [x, corridor_a_y + 0.5, 0.3],
                [0.3, 0.3, 0.3],
                p.GEOM_BOX,
                [0.7, 0.5, 0.3, 1]
            )
            
            # Corridor B furniture (IDENTICAL position relative to corridor center)
            self.create_obstacle(
                [x, corridor_b_y - 0.5, 0.3],
                [0.3, 0.3, 0.3],
                p.GEOM_BOX,
                [0.7, 0.5, 0.3, 1]
            )
        
        print(f"Added {len(self.obstacles)} symmetric obstacles")
    
    def create_wall(self, position, size, color=[0.9, 0.9, 0.85, 1], name="wall"):
        """Create a wall segment"""
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color)
        wall_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        self.walls.append({
            "id": wall_id, 
            "pos": position, 
            "size": size,
            "type": "wall",
            "name": name
        })
        return wall_id
    
    def create_obstacle(self, position, half_extents, shape_type, color):
        """Create an obstacle"""
        collision_shape = p.createCollisionShape(shape_type, halfExtents=half_extents)
        visual_shape = p.createVisualShape(shape_type, halfExtents=half_extents, rgbaColor=color)
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        self.obstacles.append({
            "id": obstacle_id, 
            "pos": position, 
            "size": half_extents,
            "shape": shape_type, 
            "color": color, 
            "type": "obstacle"
        })
        return obstacle_id
    
    def position_robot(self):
        """Position robot in Corridor A"""
        cw = self.corridor_width
        wt = self.wall_thickness
        corridor_a_y = cw/2 + wt/2
        
        # Start robot in Corridor A (top corridor)
        robot_x = -8.0
        robot_y = corridor_a_y
        robot_theta = 0.0
        
        self.robot.set_base(robot_x, robot_y, robot_theta)
        print(f"Robot positioned in Corridor A at: ({robot_x:.2f}, {robot_y:.2f})")
    
    def get_robot_position(self):
        """Get current robot position"""
        return self.robot.get_base()
    
    def cleanup(self):
        """Disconnect from PyBullet"""
        self.robot.disconnect()


def main():
    """Test symmetric environment"""
    print("=" * 70)
    print("  SYMMETRIC HALLWAY ENVIRONMENT - KF FAILURE DEMO")
    print("=" * 70)
    
    env = SymmetricEnvironment(use_gui=True)
    
    robot_pos = env.get_robot_position()
    print(f"\nRobot in Corridor A at: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
    print("\nThis environment is SYMMETRIC:")
    print("- Corridor A (top) and B (bottom) are identical")
    print("- Central wall separates them")
    print("- Sensor cannot distinguish which corridor robot is in")
    print("- KF will average between both → mean in central wall!")
    
    input("\nPress Enter to close...")
    env.cleanup()


if __name__ == "__main__":
    main()
