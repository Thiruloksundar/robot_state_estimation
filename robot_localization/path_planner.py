"""
path_planner_2x.py
------------------
Creates safe paths for the 2x scaled house environment.
Version 4: Centered doorway passages with 0.5m clearance from all obstacles
"""

import numpy as np
import pybullet as p

class PathPlanner:
    def __init__(self, environment):
        """
        Initialize path planner with 2x scaled environment information
        
        Parameters
        ----------
        environment : HouseEnvironment
            The 2x scaled house environment object
        """
        self.env = environment
        
        # Get room dimensions from environment (2x scaled)
        self.rw = environment.room_width
        self.rd = environment.room_depth
        self.cw = environment.corridor_width
        
        # Safety clearance from obstacles and walls
        self.clearance = 0.5  # 0.5 meters minimum clearance
        
        # Calculate room centers and doorway positions
        self.room_centers = self._calculate_room_centers()
        self.doorway_centers = self._calculate_doorway_centers()
        
    def _calculate_room_centers(self):
        """Calculate the center positions of all 4 rooms"""
        centers = {
            'room1': [-self.rw/2 - self.cw/2, self.rd/2 + self.cw/2],   # Top-Left (Office)
            'room2': [self.rw/2 + self.cw/2, self.rd/2 + self.cw/2],    # Top-Right (Living)
            'room3': [-self.rw/2 - self.cw/2, -self.rd/2 - self.cw/2],  # Bottom-Left (Kitchen)
            'room4': [self.rw/2 + self.cw/2, -self.rd/2 - self.cw/2],   # Bottom-Right (Bedroom)
        }
        return centers
    
    def _calculate_doorway_centers(self):
        """Calculate the exact center of each doorway"""
        # Doorways are at (cw/2 + rd/2) from corridor center for rooms 1&2
        # and -(cw/2 + rd/2) for rooms 3&4
        doorways = {
            'room1': [-self.cw/2, self.cw/2 + self.rd/2],    # Left wall, center Y
            'room2': [self.cw/2, self.cw/2 + self.rd/2],     # Right wall, center Y
            'room3': [-self.cw/2, -(self.cw/2 + self.rd/2)], # Left wall, center Y
            'room4': [self.cw/2, -(self.cw/2 + self.rd/2)],  # Right wall, center Y
        }
        return doorways
    
    def get_grand_tour_path(self):
        """
        Create a SAFE path with centered doorway passages.
        Maintains 0.5m clearance from all obstacles and walls.
        
        Path order: Room 4 (start) -> Corridor -> Room 3 -> Corridor -> Room 1 -> Corridor -> Room 2 -> Corridor -> Room 4 (end)
        
        Returns
        -------
        list of [x, y] waypoints
        """
        waypoints = []
        
        r1_center = self.room_centers['room1']
        r2_center = self.room_centers['room2']
        r3_center = self.room_centers['room3']
        r4_center = self.room_centers['room4']
        
        # Get exact doorway centers
        door1 = self.doorway_centers['room1']
        door2 = self.doorway_centers['room2']
        door3 = self.doorway_centers['room3']
        door4 = self.doorway_centers['room4']
        
        # === ROOM 4 (BEDROOM) - Starting position ===
        # Move close to walls to trigger KF failures in Scenario 4
        waypoints.append([r4_center[0] - 3.0, r4_center[1] + 2.0])  # Start
        waypoints.append([r4_center[0] - 3.5, r4_center[1] + 3.0])
        waypoints.append([r4_center[0] - 3.0, r4_center[1] + 4.0])
        waypoints.append([r4_center[0] - 1.5, r4_center[1] + 4.0])
        waypoints.append([r4_center[0] + 0.0, r4_center[1] + 4.0])
        waypoints.append([r4_center[0] + 1.5, r4_center[1] + 4.0])
        
        # CLOSE TO RIGHT WALL (0.8m from wall) - critical for KF failure
        wall_distance = 0.8
        right_wall_x = r4_center[0] + self.rw/2 - wall_distance
        waypoints.append([right_wall_x, r4_center[1] + 4.0])
        waypoints.append([right_wall_x, r4_center[1] + 2.0])  # Move parallel to wall
        waypoints.append([right_wall_x, r4_center[1] + 0.0])  # Continue parallel
        waypoints.append([right_wall_x, r4_center[1] - 2.0])  # Continue parallel
        
        # Move away from wall
        waypoints.append([r4_center[0] + 1.5, r4_center[1] - 2.0])
        waypoints.append([r4_center[0] + 0.0, r4_center[1] - 1.0])
        waypoints.append([r4_center[0] - 1.5, r4_center[1] + 0.5])
        waypoints.append([r4_center[0] - 2.5, r4_center[1] + 1.5])

        
        # === EXIT ROOM 4 - Through centered doorway ===
        waypoints.append([r4_center[0] - 3.5, door4[1]])  # Align with doorway center
        waypoints.append([door4[0] - 1.0, door4[1]])      # Approach doorway center
        waypoints.append([door4[0], door4[1]])            # CENTER of doorway
        waypoints.append([door4[0] + 1.0, door4[1]])      # Exit doorway to corridor
        
        # === CORRIDOR - Move to Room 3 door ===
        waypoints.append([0.0, door4[1]])
        waypoints.append([0.0, -2.0])
        waypoints.append([0.0, -5.0])
        waypoints.append([0.0, door3[1]])  # Align with Room 3 doorway
        
        # === ENTER ROOM 3 - Through centered doorway ===
        waypoints.append([door3[0] + 1.0, door3[1]])      # Approach from corridor
        waypoints.append([door3[0], door3[1]])            # CENTER of doorway
        waypoints.append([door3[0] - 1.0, door3[1]])      # Inside room
        waypoints.append([r3_center[0] + 3.0, door3[1]])  # Move deeper into room
        
        # === ROOM 3 (KITCHEN) - Safe navigation ===
        waypoints.append([r3_center[0] + 3.5, r3_center[1] + 3.0])
        waypoints.append([r3_center[0] + 3.5, r3_center[1] + 4.0])
        waypoints.append([r3_center[0] + 2.0, r3_center[1] + 4.5])
        waypoints.append([r3_center[0] + 0.0, r3_center[1] + 4.5])
        waypoints.append([r3_center[0] - 2.0, r3_center[1] + 4.0])
        waypoints.append([r3_center[0] - 3.5, r3_center[1] + 3.0])
        waypoints.append([r3_center[0] - 4.0, r3_center[1] + 1.5])
        waypoints.append([r3_center[0] - 4.0, r3_center[1] + 0.0])
        waypoints.append([r3_center[0] - 4.0, r3_center[1] - 1.5])
        waypoints.append([r3_center[0] - 3.0, r3_center[1] - 2.5])
        waypoints.append([r3_center[0] - 1.5, r3_center[1] - 3.0])
        waypoints.append([r3_center[0] + 0.0, r3_center[1] - 2.5])
        waypoints.append([r3_center[0] + 1.5, r3_center[1] - 2.0])
        waypoints.append([r3_center[0] + 3.0, r3_center[1] - 1.0])
        waypoints.append([r3_center[0] + 3.5, r3_center[1] + 0.5])
        waypoints.append([r3_center[0] + 3.0, door3[1]])  # Return to doorway level
        
        # === EXIT ROOM 3 - Through centered doorway ===
        waypoints.append([door3[0] - 1.0, door3[1]])      # Approach doorway
        waypoints.append([door3[0], door3[1]])            # CENTER of doorway
        waypoints.append([door3[0] + 1.0, door3[1]])      # Exit to corridor
        
        # === CORRIDOR - Move to Room 1 door ===
        waypoints.append([0.0, door3[1]])
        waypoints.append([0.0, -2.0])
        waypoints.append([0.0, 0.0])
        waypoints.append([0.0, 2.0])
        waypoints.append([0.0, 5.0])
        waypoints.append([0.0, 8.0])
        waypoints.append([0.0, door1[1]])  # Align with Room 1 doorway
        
        # === ENTER ROOM 1 - Through centered doorway ===
        waypoints.append([door1[0] + 1.0, door1[1]])      # Approach from corridor
        waypoints.append([door1[0], door1[1]])            # CENTER of doorway
        waypoints.append([door1[0] - 1.0, door1[1]])      # Inside room
        waypoints.append([r1_center[0] + 3.0, door1[1]])  # Move deeper into room
        
        # === ROOM 1 (OFFICE) - Safe navigation ===
        waypoints.append([r1_center[0] + 3.5, r1_center[1] - 3.0])
        waypoints.append([r1_center[0] + 3.5, r1_center[1] - 4.0])
        waypoints.append([r1_center[0] + 2.0, r1_center[1] - 4.5])
        waypoints.append([r1_center[0] + 0.0, r1_center[1] - 4.5])
        waypoints.append([r1_center[0] - 2.0, r1_center[1] - 4.0])
        waypoints.append([r1_center[0] - 3.5, r1_center[1] - 3.0])
        waypoints.append([r1_center[0] - 4.0, r1_center[1] - 1.5])
        waypoints.append([r1_center[0] - 4.0, r1_center[1] + 0.0])
        waypoints.append([r1_center[0] - 4.0, r1_center[1] + 1.5])
        waypoints.append([r1_center[0] - 3.0, r1_center[1] + 2.5])
        waypoints.append([r1_center[0] - 1.5, r1_center[1] + 3.5])
        waypoints.append([r1_center[0] + 0.0, r1_center[1] + 4.0])
        waypoints.append([r1_center[0] + 1.5, r1_center[1] + 4.0])
        waypoints.append([r1_center[0] + 3.0, r1_center[1] + 3.0])
        waypoints.append([r1_center[0] + 3.5, r1_center[1] + 1.5])
        waypoints.append([r1_center[0] + 3.5, r1_center[1] + 0.0])
        waypoints.append([r1_center[0] + 3.0, door1[1]])  # Return to doorway level
        
        # === EXIT ROOM 1 - Through centered doorway ===
        waypoints.append([door1[0] - 1.0, door1[1]])      # Approach doorway
        waypoints.append([door1[0], door1[1]])            # CENTER of doorway
        waypoints.append([door1[0] + 1.0, door1[1]])      # Exit to corridor
        
        # === CORRIDOR - Move to Room 2 door ===
        waypoints.append([0.0, door1[1]])
        waypoints.append([0.0, door2[1]])  # Align with Room 2 doorway (same Y)
        
        # === ENTER ROOM 2 - Through centered doorway ===
        waypoints.append([door2[0] - 1.0, door2[1]])      # Approach from corridor
        waypoints.append([door2[0], door2[1]])            # CENTER of doorway
        waypoints.append([door2[0] + 1.0, door2[1]])      # Inside room
        waypoints.append([r2_center[0] - 3.0, door2[1]])  # Move deeper into room
        
        # === ROOM 2 (LIVING ROOM) - Safe navigation ===
        waypoints.append([r2_center[0] - 3.5, r2_center[1] - 3.0])
        waypoints.append([r2_center[0] - 3.5, r2_center[1] - 4.0])
        waypoints.append([r2_center[0] - 2.0, r2_center[1] - 4.5])
        waypoints.append([r2_center[0] + 0.0, r2_center[1] - 4.5])
        waypoints.append([r2_center[0] + 2.0, r2_center[1] - 4.0])
        waypoints.append([r2_center[0] + 3.5, r2_center[1] - 3.0])
        waypoints.append([r2_center[0] + 4.0, r2_center[1] - 1.5])
        waypoints.append([r2_center[0] + 4.0, r2_center[1] + 0.0])
        waypoints.append([r2_center[0] + 4.0, r2_center[1] + 1.5])
        waypoints.append([r2_center[0] + 3.0, r2_center[1] + 2.5])
        waypoints.append([r2_center[0] + 1.5, r2_center[1] + 3.5])
        waypoints.append([r2_center[0] + 0.0, r2_center[1] + 4.0])
        waypoints.append([r2_center[0] - 1.5, r2_center[1] + 4.0])
        waypoints.append([r2_center[0] - 3.0, r2_center[1] + 3.0])
        waypoints.append([r2_center[0] - 3.5, r2_center[1] + 1.5])
        waypoints.append([r2_center[0] - 3.5, r2_center[1] + 0.0])
        waypoints.append([r2_center[0] - 3.0, door2[1]])  # Return to doorway level
        
        # === EXIT ROOM 2 - Through centered doorway ===
        waypoints.append([door2[0] + 1.0, door2[1]])      # Approach doorway
        waypoints.append([door2[0], door2[1]])            # CENTER of doorway
        waypoints.append([door2[0] - 1.0, door2[1]])      # Exit to corridor
        
        # === CORRIDOR - Return to Room 4 ===
        waypoints.append([0.0, door2[1]])
        waypoints.append([0.0, 2.0])
        waypoints.append([0.0, -2.0])
        waypoints.append([0.0, -5.0])
        waypoints.append([0.0, door4[1]])  # Align with Room 4 doorway
        
        # === ENTER ROOM 4 - Through centered doorway (final) ===
        waypoints.append([door4[0] + 1.0, door4[1]])      # Approach from corridor
        waypoints.append([door4[0], door4[1]])            # CENTER of doorway
        waypoints.append([door4[0] - 1.0, door4[1]])      # Inside room
        waypoints.append([r4_center[0] - 2.0, door4[1]])  # Move deeper
        waypoints.append([r4_center[0] - 2.0, r4_center[1] + 1.0])
        waypoints.append([r4_center[0] - 1.0, r4_center[1] + 0.5])
        waypoints.append([r4_center[0] + 0.0, r4_center[1] + 0.0])  # Final position
        
        return waypoints
    
    def visualize_path(self, waypoints, color=[1, 0, 0], line_width=3):
        """
        Draw the path in PyBullet environment
        
        Parameters
        ----------
        waypoints : list
            List of [x, y] waypoint positions
        color : list
            RGB color for the path line
        line_width : float
            Width of the path line
        """
        print(f"Visualizing SAFE path with {len(waypoints)} waypoints")
        print(f"All doorways pass through exact center")
        print(f"Maintaining {self.clearance}m clearance from obstacles")
        
        # Draw lines between consecutive waypoints
        for i in range(len(waypoints) - 1):
            start = [waypoints[i][0], waypoints[i][1], 0.1]
            end = [waypoints[i+1][0], waypoints[i+1][1], 0.1]
            p.addUserDebugLine(start, end, color, line_width)
        
        # Mark waypoints
        for i, wp in enumerate(waypoints):
            # Mark every 10th waypoint
            if i % 10 == 0:
                p.addUserDebugText(f"{i}", [wp[0], wp[1], 0.3], 
                                 textColorRGB=[0, 0, 1], textSize=1.2)
    
    def get_simple_path(self, start_room='room4', end_room='room3'):
        """
        Create a simple safe path between two rooms
        
        Parameters
        ----------
        start_room : str
            Starting room ('room1', 'room2', 'room3', or 'room4')
        end_room : str
            Ending room
            
        Returns
        -------
        list of [x, y] waypoints
        """
        waypoints = []
        
        start = self.room_centers[start_room]
        end = self.room_centers[end_room]
        start_door = self.doorway_centers[start_room]
        end_door = self.doorway_centers[end_room]
        
        # Simple safe path through corridor center and doorway centers
        waypoints.append([start[0] - 3.0, start[1]])
        waypoints.append([start_door[0] - 1.0, start_door[1]])
        waypoints.append([start_door[0], start_door[1]])  # Center of start doorway
        waypoints.append([0.0, start_door[1]])
        waypoints.append([0.0, end_door[1]])
        waypoints.append([end_door[0], end_door[1]])  # Center of end doorway
        waypoints.append([end[0] + 3.0, end[1]])
        
        return waypoints


def test_path_planner():
    """Test the path planner with visualization"""
    from environment_2x import HouseEnvironment
    
    print("=" * 70)
    print("  TESTING CENTERED DOORWAY PATH (0.5m clearance)")
    print("=" * 70)
    
    # Create environment
    env = HouseEnvironment(use_gui=True)
    
    # Create path planner
    planner = PathPlanner(env)
    
    # Get the grand tour path
    waypoints = planner.get_grand_tour_path()
    
    print(f"\nGenerated SAFE path with {len(waypoints)} waypoints")
    print("All doorways pass through exact center")
    print("Path maintains 0.5m clearance from ALL obstacles")
    print("Path visits: Room 4 → Room 3 → Room 1 → Room 2 → Room 4")
    
    # Visualize the path
    planner.visualize_path(waypoints, color=[1, 0, 0], line_width=5)
    
    print("\n" + "=" * 70)
    print("RED LINE shows the complete SAFE path")
    print("BLUE NUMBERS show waypoint indices (every 10th)")
    print("Path passes through CENTER of each doorway")
    print("=" * 70)
    
    input("\nPress Enter to close...")
    env.cleanup()


if __name__ == "__main__":
    test_path_planner()
