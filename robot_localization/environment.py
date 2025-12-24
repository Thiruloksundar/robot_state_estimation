import pybullet as p
import pybullet_data
import numpy as np
import time
from robot import PR2Robot

class HouseEnvironment:
    """
    2x Scaled House Environment with 4 rooms and a central corridor.
    PR2 Robot spawns in Room 4 (Bedroom).
    
    All dimensions are doubled while obstacles remain original size.
    
    Layout:
    ┌─────────────────────────────┐
    │         │         │         │
    │  Room 1 │Corridor │ Room 2  │
    │         │         │         │
    │         │         │         │
    ├─────────┴─────────┴─────────┤
    │         │         │         │
    │  Room 3 │Corridor │ Room 4  │
    │         │         │(BEDROOM)│
    │         │         │  [PR2]  │
    └─────────────────────────────┘
    """
    
    def __init__(self, use_gui=True):
        # Connect to PyBullet using PR2Robot
        self.robot = PR2Robot(gui=use_gui)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # House dimensions (2x scaled from original)
        self.wall_thickness = 0.2
        self.wall_height = 2.5
        self.room_width = 15.0      # 2x from 7.5
        self.room_depth = 13.5      # 2x from 6.75
        self.corridor_width = 7.5   # 2x from 3.75
        self.doorway_width = 5.0    # Increased from 3.6 for easier passage
        
        # Storage for all objects
        self.walls = []
        self.obstacles = []
        
        # Build the house
        self.build_house_walls()
        self.add_obstacles()
        self.position_robot_in_bedroom()
        
        # Set camera view focused on bedroom (Room 4)
        room4_center = [self.room_width/2 + self.corridor_width/2, 
                       -self.room_depth/2 - self.corridor_width/2]
        p.resetDebugVisualizerCamera(
            cameraDistance=24,  # 2x from 12
            cameraYaw=45,
            cameraPitch=-35,
            cameraTargetPosition=[room4_center[0], room4_center[1], 0]
        )
        
    def position_robot_in_bedroom(self):
        """Position PR2 robot in Room 4 (Bedroom)"""
        # Room 4 center position
        room4_x = self.room_width/2 + self.corridor_width/2
        room4_y = -self.room_depth/2 - self.corridor_width/2
        
        # Place robot in open area of bedroom (avoiding bed and furniture)
        robot_x = room4_x - 3.0  # 2x from 1.5
        robot_y = room4_y + 2.0  # 2x from 1.0
        robot_theta = 0.0
        
        self.robot.set_base(robot_x, robot_y, robot_theta)
        print(f"PR2 robot positioned in Bedroom at: ({robot_x:.2f}, {robot_y:.2f}, θ={robot_theta:.2f})")
        
    def create_wall(self, position, size, color=[0.9, 0.9, 0.85, 1]):
        """Create a wall segment"""
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
        visual_shape = p.createVisualShape(
            p.GEOM_BOX, 
            halfExtents=size,
            rgbaColor=color
        )
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
            "type": "wall"
        })
        return wall_id
    
    def build_house_walls(self):
        """Build complete house structure with no gaps"""
        wt = self.wall_thickness
        wh = self.wall_height
        rw = self.room_width
        rd = self.room_depth
        cw = self.corridor_width
        dw = self.doorway_width
        
        # Total house dimensions
        total_width = 2 * rw + cw
        total_depth = 2 * rd + cw
        
        print("Building 2x scaled house walls...")
        
        # OUTER PERIMETER WALLS
        self.create_wall([0, total_depth/2, wh/2], [total_width/2 + wt, wt/2, wh/2])
        self.create_wall([0, -total_depth/2, wh/2], [total_width/2 + wt, wt/2, wh/2])
        self.create_wall([total_width/2, 0, wh/2], [wt/2, total_depth/2, wh/2])
        self.create_wall([-total_width/2, 0, wh/2], [wt/2, total_depth/2, wh/2])
        
        # HORIZONTAL DIVIDING WALLS
        self.create_wall([-total_width/2 + rw/2 + wt/2, 0, wh/2], [rw/2 + wt/2, wt/2, wh/2])
        self.create_wall([total_width/2 - rw/2 - wt/2, 0, wh/2], [rw/2 + wt/2, wt/2, wh/2])
        
        # VERTICAL WALLS (separating rooms from corridor)
        # Room 1 wall (top left)
        door1_y_center = (cw/2 + rd/2)
        wall1_top_y = cw/2 + rd
        self.create_wall([-cw/2, (door1_y_center + dw/2 + wall1_top_y)/2, wh/2],
                        [wt/2, (wall1_top_y - door1_y_center - dw/2)/2, wh/2])
        self.create_wall([-cw/2, (door1_y_center - dw/2)/2, wh/2],
                        [wt/2, (door1_y_center - dw/2)/2, wh/2])
        
        # Room 3 wall (bottom left)
        door3_y_center = -(cw/2 + rd/2)
        wall3_bottom_y = -(cw/2 + rd)
        self.create_wall([-cw/2, (door3_y_center + dw/2)/2, wh/2],
                        [wt/2, (door3_y_center + dw/2)/2, wh/2])
        self.create_wall([-cw/2, (door3_y_center - dw/2 + wall3_bottom_y)/2, wh/2],
                        [wt/2, (door3_y_center - dw/2 - wall3_bottom_y)/2, wh/2])
        
        # Room 2 wall (top right)
        door2_y_center = (cw/2 + rd/2)
        wall2_top_y = cw/2 + rd
        self.create_wall([cw/2, (door2_y_center + dw/2 + wall2_top_y)/2, wh/2],
                        [wt/2, (wall2_top_y - door2_y_center - dw/2)/2, wh/2])
        self.create_wall([cw/2, (door2_y_center - dw/2)/2, wh/2],
                        [wt/2, (door2_y_center - dw/2)/2, wh/2])
        
        # Room 4 wall (bottom right)
        door4_y_center = -(cw/2 + rd/2)
        wall4_bottom_y = -(cw/2 + rd)
        self.create_wall([cw/2, (door4_y_center + dw/2)/2, wh/2],
                        [wt/2, (door4_y_center + dw/2)/2, wh/2])
        self.create_wall([cw/2, (door4_y_center - dw/2 + wall4_bottom_y)/2, wh/2],
                        [wt/2, (door4_y_center - dw/2 - wall4_bottom_y)/2, wh/2])
        
        print(f"2x House structure complete! Total walls: {len(self.walls)}")
    
    def create_obstacle(self, position, half_extents, shape_type, color):
        """Create an obstacle with specified shape (ORIGINAL SIZE - not scaled)"""
        collision_shape = p.createCollisionShape(shape_type, halfExtents=half_extents)
        visual_shape = p.createVisualShape(shape_type, halfExtents=half_extents, rgbaColor=color)
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        self.obstacles.append({
            "id": obstacle_id, "pos": position, "size": half_extents,
            "shape": shape_type, "color": color, "type": "obstacle"
        })
        return obstacle_id
    
    def create_cylinder_obstacle(self, position, radius, height, color):
        """Create a cylindrical obstacle (ORIGINAL SIZE - not scaled)"""
        collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
        visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        self.obstacles.append({
            "id": obstacle_id, "pos": position, "radius": radius,
            "height": height, "shape": "cylinder", "color": color, "type": "obstacle"
        })
        return obstacle_id
    
    def add_obstacles(self):
        """Add obstacles to all rooms and corridor - ORIGINAL SIZE, positioned in 2x space"""
        # Room centers (2x scaled positions)
        room1_center = [-self.room_width/2 - self.corridor_width/2, self.room_depth/2 + self.corridor_width/2]
        room2_center = [self.room_width/2 + self.corridor_width/2, self.room_depth/2 + self.corridor_width/2]
        room3_center = [-self.room_width/2 - self.corridor_width/2, -self.room_depth/2 - self.corridor_width/2]
        room4_center = [self.room_width/2 + self.corridor_width/2, -self.room_depth/2 - self.corridor_width/2]
        
        # Room 1 (Office) - obstacles at 2x positions but original sizes
        r1x, r1y = room1_center
        self.create_obstacle([r1x - 3.0, r1y + 3.0, 0.4], [0.7, 0.35, 0.4], p.GEOM_BOX, [0.6, 0.4, 0.2, 1])
        self.create_obstacle([r1x - 5.0, r1y + 3.6, 0.8], [0.2, 0.5, 0.8], p.GEOM_BOX, [0.4, 0.3, 0.2, 1])
        self.create_cylinder_obstacle([r1x - 1.6, r1y + 1.6, 0.25], 0.3, 0.5, [0.1, 0.1, 0.1, 1])
        self.create_obstacle([r1x + 4.0, r1y + 3.6, 0.4], [0.3, 0.35, 0.4], p.GEOM_BOX, [0.5, 0.5, 0.5, 1])
        self.create_obstacle([r1x + 4.4, r1y - 2.0, 0.3], [0.35, 0.35, 0.3], p.GEOM_BOX, [0.65, 0.45, 0.3, 1])
        
        # Room 2 (Living Room)
        r2x, r2y = room2_center
        self.create_obstacle([r2x - 0.6, r2y + 3.6, 0.35], [1.0, 0.4, 0.35], p.GEOM_BOX, [0.2, 0.3, 0.7, 1])
        self.create_obstacle([r2x - 0.4, r2y + 1.0, 0.25], [0.5, 0.4, 0.25], p.GEOM_BOX, [0.7, 0.8, 0.9, 0.7])
        self.create_obstacle([r2x + 5.0, r2y + 2.0, 0.3], [0.2, 0.6, 0.3], p.GEOM_BOX, [0.1, 0.1, 0.1, 1])
        self.create_cylinder_obstacle([r2x + 5.0, r2y + 4.4, 0.35], 0.25, 0.7, [0.2, 0.6, 0.2, 1])
        self.create_cylinder_obstacle([r2x - 4.0, r2y + 3.0, 0.4], 0.15, 0.8, [0.9, 0.9, 0.3, 1])
        self.create_obstacle([r2x - 3.6, r2y - 1.6, 0.35], [0.5, 0.5, 0.35], p.GEOM_BOX, [0.5, 0.3, 0.2, 1])
        
        # Room 3 (Kitchen)
        r3x, r3y = room3_center
        self.create_obstacle([r3x - 3.6, r3y - 3.6, 0.45], [0.35, 0.8, 0.45], p.GEOM_BOX, [0.95, 0.95, 0.95, 1])
        self.create_obstacle([r3x - 0.4, r3y - 4.4, 0.45], [0.8, 0.35, 0.45], p.GEOM_BOX, [0.95, 0.95, 0.95, 1])
        self.create_obstacle([r3x - 5.2, r3y - 4.6, 0.9], [0.35, 0.35, 0.9], p.GEOM_BOX, [0.8, 0.8, 0.85, 1])
        # Moved obstacle away from path - was at [r3x + 1.2, r3y + 1.0] blocking corridor exit
        self.create_obstacle([r3x - 2.5, r3y + 3.2, 0.45], [0.6, 0.5, 0.45], p.GEOM_BOX, [0.7, 0.5, 0.3, 1])
        self.create_cylinder_obstacle([r3x + 4.6, r3y - 3.6, 0.35], 0.2, 0.7, [0.1, 0.4, 0.1, 1])
        self.create_cylinder_obstacle([r3x + 5.5, r3y + 1.5, 0.25], 0.25, 0.5, [0.6, 0.4, 0.2, 1])
        
        # Room 4 (Bedroom)
        r4x, r4y = room4_center
        self.create_obstacle([r4x + 0.6, r4y - 3.0, 0.4], [0.9, 1.0, 0.4], p.GEOM_BOX, [0.6, 0.3, 0.7, 1])
        self.create_obstacle([r4x - 2.0, r4y - 1.2, 0.35], [0.3, 0.3, 0.35], p.GEOM_BOX, [0.65, 0.45, 0.25, 1])
        self.create_obstacle([r4x + 3.2, r4y - 1.2, 0.35], [0.3, 0.3, 0.35], p.GEOM_BOX, [0.65, 0.45, 0.25, 1])
        self.create_obstacle([r4x + 5.0, r4y + 3.6, 1.0], [0.3, 0.7, 1.0], p.GEOM_BOX, [0.5, 0.35, 0.2, 1])
        self.create_obstacle([r4x - 4.6, r4y + 4.0, 0.45], [0.5, 0.3, 0.45], p.GEOM_BOX, [0.9, 0.9, 0.9, 1])
        self.create_cylinder_obstacle([r4x + 3.2, r4y - 0.6, 0.4], 0.12, 0.8, [1.0, 0.5, 0.1, 1])
        
        # Corridor - positioned at 2x scale
        self.create_obstacle([-3.0, 2.6, 0.25], [0.7, 0.25, 0.25], p.GEOM_BOX, [0.8, 0.2, 0.2, 1])
        self.create_cylinder_obstacle([2.8, 3.0, 0.45], 0.18, 0.9, [0.8, 0.5, 0.4, 1])
        self.create_obstacle([3.2, -2.8, 0.4], [0.35, 0.35, 0.4], p.GEOM_BOX, [0.2, 0.6, 0.6, 1])
        self.create_cylinder_obstacle([-2.6, -3.2, 0.35], 0.15, 0.7, [0.1, 0.3, 0.8, 1])
        
        print(f"Total obstacles created: {len(self.obstacles)} (original size, 2x positions)")
    
    def get_robot_position(self):
        """Get current robot position"""
        return self.robot.get_base()
    
    def move_robot(self, x, y, theta):
        """Move robot to specified position"""
        self.robot.set_base(x, y, theta)
    
    def get_environment_info(self):
        """Print environment information"""
        info = f"""
        ╔══════════════════════════════════════════════════════════╗
        ║      2x SCALED HOUSE ENVIRONMENT INFORMATION             ║
        ╚══════════════════════════════════════════════════════════╝
        
        Layout: 4 rooms around central corridor (2x scale)
        - Room dimensions: {self.room_width}m x {self.room_depth}m
        - Corridor width: {self.corridor_width}m
        - Doorway width: {self.doorway_width}m
        - Obstacle sizes: ORIGINAL (not scaled)
        
        Total walls: {len(self.walls)}
        Total obstacles: {len(self.obstacles)}
        
        Room Contents:
        - Room 1 (Top-Left):    Office (5 items)
        - Room 2 (Top-Right):   Living Room (6 items)
        - Room 3 (Bottom-Left): Kitchen (6 items)
        - Room 4 (Bottom-Right): Bedroom (6 items) ★ PR2 ROBOT HERE ★
        - Corridor: 4 items
        """
        print(info)
    
    def run_visualization(self, duration=60):
        """Run the environment for visualization"""
        print("\nRunning environment visualization...")
        print("Close the PyBullet window to exit.")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.robot.step()
            time.sleep(1./240.)
    
    def cleanup(self):
        """Disconnect from PyBullet"""
        self.robot.disconnect()


def main():
    """Main function to create and visualize the 2x scaled house environment"""
    print("=" * 70)
    print("  PR2 ROBOT IN 2x SCALED BEDROOM - HOUSE ENVIRONMENT")
    print("=" * 70)
    
    # Create environment
    env = HouseEnvironment(use_gui=True)
    
    # Print environment information
    env.get_environment_info()
    
    # Get robot position
    robot_pos = env.get_robot_position()
    print(f"\nPR2 Robot spawned at: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}, θ={robot_pos[2]:.2f})")
    
    print("\n" + "=" * 70)
    print("2x SCALED ENVIRONMENT READY:")
    print("- PR2 Robot is in Room 4 (Bedroom)")
    print("- Environment is 2x larger")
    print("- Obstacles remain original size")
    print("- All rooms fully furnished and accessible")
    print("=" * 70)
    
    # Run visualization
    try:
        env.run_visualization(duration=300)
    except KeyboardInterrupt:
        print("\nVisualization stopped by user.")
    finally:
        env.cleanup()
        print("Environment closed.")


if __name__ == "__main__":
    main()
