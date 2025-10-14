import numpy as np
import heapq
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity

from rclpy.qos import (
    ReliabilityPolicy,
    QoSProfile,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from PIL import Image
from geometry_msgs.msg import Twist


np.set_printoptions(
    2, suppress=True, threshold=np.inf
)  # Print numpy arrays to specified d.p., suppress scientific notation (e.g. 1e-5), and do not truncate

set_logger_level("path_planning", level=LoggingSeverity.WARN) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

is_simulation = True # Remember to configure this to False if testing for the real lab setup
if is_simulation:
    max_translate_velocity = 1.4
else:
    max_translate_velocity = 0.4

occupancy_grid_resolution = 0.2

sim_goal_list = [(3.4, -3.6), (3.2, 0.2), (2.4, -3.6), (-0.4, -3.8)]
sim_grid_start = (-1.0, -5.0)

irl_goal_list = [(2.1, -1.7), (2.3, -0.3), (1.5, -1.7), (0.3, -1.7)]
irl_grid_start = (0.1, -1.9)

heading_movement = True


class WaypointNode(Node):
    '''Node to calculate path and move robot towards given goal_coordinates, using pose info from either gazebo odometer or optitrack'''
    def __init__(self, map_array:np.array, goal_list:list, is_simulation:bool=True):
        super().__init__('waypoint')
        self.get_logger().info("Starting WaypointNode")

        self.is_simulation = is_simulation

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)

            self.map_sub = self.create_subscription( 
                PoseStamped,
                '/vrpn_mocap/bingda_003/pose',
                self.optitrack_callback, 
                qos_profile
                )
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node       
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

        self.goal_list = goal_list
        self.map_array = map_array

        self.pose = None
        self.waypoint_list = None
        self.goal_reached = True
        self.current_waypoint_idx = 0
        self.current_goal_idx = 0

    def yaw_from_quaternion(self, q):
        '''Returns yaw angle (in rad) for orientation based on given quaternion input q'''
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def optitrack_callback(self, msg:PoseStamped):
        '''Callback to calculate 2D pose info from Optitrack node. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        x, y = msg.pose.position.x, msg.pose.position.y
        heading = np.rad2deg(self.yaw_from_quaternion(msg.pose.orientation))
        self.pose = np.array((x,y,heading))
        return self.pose

    def odometer_callback(self, msg):
        '''Callback to calculate 2D pose info from Gazebo odomoter. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        latest_pose_msg = msg.pose.pose 
        heading = np.rad2deg(self.yaw_from_quaternion(latest_pose_msg.orientation))
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        return self.pose

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        '''Publishes a Twist message to ROS to move a robot. Inputs are x and y linear velocities, as well as turn (z-axis yaw) angular velocity.'''
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)


    def set_waypoints(self, waypoints:list):
        '''Set new waypoints when a goal has been reached'''
        self.goal_reached = False
        self.waypoints = waypoints
        self.current_waypoint_idx = 0


    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""
        global sim_grid_start, occupancy_grid_resolution
        if self.pose is None:
            return # Does not run if no pose received from Odom or Optitrack
        self.get_logger().info(f"Pose: {self.pose}")

        if self.goal_reached:
            if self.current_goal_idx >= len(self.goal_list):
                self.get_logger().warn("All goals reached")
                raise SystemExit # Will exit out of the spin loop due to the try/except catch
            
            # Initialise the relevant map/grid variables, mainly start and goal indices
            start_coords = self.pose[:2] # Get coords from odom/optitrack
            # start_coords = np.round(start_coords, 1) 
            if self.is_simulation:
                x_start, y_start, resolution = sim_grid_start[0], sim_grid_start[1], occupancy_grid_resolution
            else:
                x_start, y_start, resolution = irl_grid_start[0], irl_grid_start[1], occupancy_grid_resolution

            goal_coords = self.goal_list[self.current_goal_idx]
            # Conversion between numerical coordinates and array indices
            start = (int(start_coords[0]//resolution - x_start//resolution), int(start_coords[1]//resolution - y_start//resolution))
            goal = (int(goal_coords[0]//resolution - x_start//resolution), int(goal_coords[1]//resolution - y_start//resolution))
            
            if start == goal:
                self.get_logger().warn("Already at goal, moving to next goal")
                self.goal_reached = True
                self.current_goal_idx += 1
                return
            
            # Use grid and A* search to find solution. If in sim, draw solution out
            grid = Grid(self.map_array, starting_position=start, goal_position=goal)
            if not grid.check_grid_validity():
                self.get_logger().warn("Invalid start and/or goal")
            solution_path = a_star_search(grid)
            waypoint_list = convert_path_to_waypoints(solution_path)
            if self.is_simulation: grid.draw_grid_map(waypoint_list, solution_path)

            # Set the proper waypoints using given solutions
            coordinate_waypoints = []
            for point in waypoint_list:
                coordinate_waypoints.append((point[0]*resolution+x_start, point[1]*resolution+y_start))
            self.set_waypoints(coordinate_waypoints)

        else:
            if self.current_waypoint_idx == len(self.waypoints): # All waypoints reached, means goal reached
                self.get_logger().warn("Goal reached!")
                self.goal_reached = True
                self.current_goal_idx += 1

                if self.current_goal_idx >= len(self.goal_list): # All goals reached, exit system
                    self.get_logger().warn("All goals reached!")
                    raise SystemExit # Exit and stop spin so rclpy can shutdown

            else:
                target_waypoint = np.array(self.waypoints[self.current_waypoint_idx])
                if np.linalg.norm(self.pose[:2] - target_waypoint) < 0.03: # If less than threshold distance away from target waypoint
                    self.move_2D() # Stop
                    if self.current_waypoint_idx+1 != len(self.waypoints):
                        self.get_logger().info(f"Waypoint {target_waypoint} reached. Next target is {self.waypoints[self.current_waypoint_idx+1]}")
                    self.current_waypoint_idx += 1 # Set next waypoint

                else:
                    if not heading_movement:
                        # Strafe variation; only x and y linear velocites
                        x = target_waypoint[0] - self.pose[0]
                        y = target_waypoint[1] - self.pose[1]

                        if 0.0 < x < 0.3:
                            x = 0.3
                        elif -0.3 < x < 0.0:
                            x = -0.3
                        if 0.0 < y < 0.3:
                            y = 0.3
                        elif -0.3 < y < 0.0:
                            y = -0.3

                        self.move_2D(x, y)

                    else:
                        # Heading variation; only x forward velocity and yaw turning
                        x_delta = target_waypoint[0] - self.pose[0]
                        y_delta = target_waypoint[1] - self.pose[1]
                        goal_heading = np.rad2deg(np.arctan2(y_delta, x_delta))
                        actual_heading = self.pose[2] 

                        heading_delta = goal_heading - actual_heading
                        if heading_delta < -180:
                            heading_delta += 360
                        elif heading_delta > 180:
                            heading_delta -= 360
                            
                        if abs(heading_delta) < 30: # Only have forward movement if heading is within +- 30deg
                            x_vel = np.sqrt(x_delta**2 + y_delta**2)
                            if 0.0 < x_vel < 0.2:
                                x_vel = 0.2
                            elif -0.2 < x_vel < 0.0:
                                x_vel = -0.2
                        else:
                            if np.linalg.norm(self.pose[:2] - target_waypoint) > 0.1: 
                                x_vel = 0.0
                            else:
                                x_vel = 0.5  # If near target, don't care about adjusting heading since it'll fluctuate wildly

                        self.move_2D(x_vel, 0.0, np.deg2rad(heading_delta)*2)
                    

class Grid():
    '''
    Grid class to use with occupancy grid. Contains the following functions:
        check_grid_validity : Uses flood fill to check if there's a valid path from start to goal position
        draw_grid_map : Creates a colour image of the grid, as well as waypoints and full solution path if given.

    __init__ input Args:
        grid_array : 2D numpy array representing the occupancy grid
        starting_position : tuple of starting indices within the numpy array
        goal_position : tuple of goal indices within the numpy array. Works with negative indices as well
    '''
    def __init__(self, grid_array:np.array=np.array([]), starting_position:tuple=(0,0), goal_position:tuple=(-1,-1)):
        self.grid = grid_array
        self.shape = self.grid.shape
        self.starting_position = starting_position

        # If goal position given with negative index, need to convert to +ve
        if goal_position[0] < 0:
            goal_x = self.shape[0] + goal_position[0]
        else:
            goal_x = goal_position[0]
        if goal_position[1] < 0:
            goal_y = self.shape[1] + goal_position[1]
        else:
            goal_y = goal_position[1]   

        self.goal_position = (goal_x, goal_y)

    def check_grid_validity(self):
        '''Use flood fill to check if there's a viable path between start and goal positions'''
        grid = self.grid.copy()
        flood_stack = [(self.goal_position[0], self.goal_position[1])]
        while flood_stack:
            tile = flood_stack[0]
            del flood_stack[0]
            try:
                next_tile = (tile[0]+1, tile[1])
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0]-1, tile[1])
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0], tile[1]+1)
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
            try:
                next_tile = (tile[0], tile[1]-1)
                if grid[next_tile] == 0:
                    grid[next_tile] = 1
                    flood_stack.append(next_tile)
            except:pass
        if grid[self.starting_position] == 1: # Means the flood is able to reach starting position from the ending position
            return True
        else:
            return False

    def draw_grid_map(self, waypoints:list=(), path:list=(), obstacle_threshold:float=50):
        '''Creates an image of the maze and path taken. Maze walls in blue, empty space in white, path taken in green and waypoints in red

        Args:
            waypoints : list (or other iterable) of tuple coordinates representing all the grid indices for the waypoints. Will be represented in red, takes precedence over path
            path : list (or other iterable) of tuple coordinates representing all the grid indices forming the solution path. Will be represented in green
            obstacle_threshold : Optional float to indicate threshhold for whether a grid is considered occupied. Not important for ca2             
        '''
        image_grid = np.ones((self.grid.shape[0],self.grid.shape[1],3), dtype=np.uint8)
        image_grid[self.grid <= obstacle_threshold] = (255,255,255)
        image_grid[self.grid > obstacle_threshold] = (0,0,255)

        for x, y in path:
            image_grid[x][y] = (0,255,0)

        for point in waypoints:
            image_grid[point] = (255,0,0)

        image_grid = np.flip(image_grid, axis=1)[::-1]
        img = Image.fromarray(image_grid, 'RGB')

        # Resize image
        base_width = 500
        wpercent = (base_width / float(img.size[0]))
        hsize = int((float(img.size[1]) * float(wpercent)))
        img = img.resize((base_width, hsize), Image.Resampling.NEAREST)

        img.show()


def heuristic_cost(destination:tuple, cell_coordinates):
    '''Uses Manhattan distance to calculate heuristic cost from the cell to the destination, used for A* pp'''
    return abs(destination[0]-cell_coordinates[0]) + abs(destination[1]-cell_coordinates[1])

def trace_path(cell_details:list, destination:tuple):
    '''Calculate and return the final, optimised A* path as an ordered list of cells to visit'''
    path = []
    x, y = destination[0], destination[1]

    while not (cell_details[x][y].parent_coords == (x,y)):
        path.append((x,y))
        x, y = cell_details[x][y].parent_coords
    
    path.append((x, y))
    path.reverse()
    return path


def move_direction(starting_cell, destination_cell):
    '''Function used for converting full A* path to waypoints'''
    return (destination_cell[0]-starting_cell[0], destination_cell[1]-starting_cell[1])


def convert_path_to_waypoints(path:list):
    '''Converts full A* path to select waypoints by removing all cells along a straight line (except the start and end cells of the line)'''
    waypoints = [path[0]]
    for idx in range(1, len(path)-1):
        if move_direction(path[idx-1], path[idx]) != move_direction(path[idx], path[idx+1]):
            waypoints.append(path[idx])
    waypoints.append(path[-1])
    return waypoints

            
class Cell:
    '''Cell class used for A* search'''
    def __init__(self, parent_coords:tuple=(0,0)):
        self.parent_coords = parent_coords
        self.f = np.inf # Total cost
        self.g = np.inf # Cost from start position
        self.h = 0 # Heuristic cost to destination (reward) cell

def get_valid_actions(grid:Grid, coordinates:tuple, obstacle_threshold:float):
    '''Returns list of valid actions based on grid position. Currently set to only allow 4 cardinal direction movement'''
    x, y = coordinates
    valid_actions = []
    if x > 0 and grid.grid[x-1, y] <= obstacle_threshold: # Cell below
        valid_actions.append((-1, 0)) 

    if x < grid.shape[0]-1 and grid.grid[x+1, y] <= obstacle_threshold: # Cell above
        valid_actions.append((1, 0))

    if y > 0 and grid.grid[x, y-1] <= obstacle_threshold: # Cell left
        valid_actions.append((0, -1))

    if y < grid.shape[1]-1 and grid.grid[x, y+1] <= obstacle_threshold: # Cell right
        valid_actions.append((0, 1))

    ### Code below forms the basis for diagonal paths as well; didn't have enough time to implement but feel free to try it yourself
    # if x > 0 and y > 0 and grid.grid[x-1, y-1] <= 0: # Cell bottom left
    #     valid_actions.append((-1, -1)) 

    # if x > 0 and y < grid.shape[1]-1 and grid.grid[x-1, y+1] <= 0: # Cell bottom right
    #     valid_actions.append((-1, 1)) 

    # if x < grid.shape[0]-1 and y > 0 and grid.grid[x+1, y-1] <= 0: # Cell top left
    #     valid_actions.append((1, -1)) 

    # if x < grid.shape[0]-1 and y < grid.shape[1]-1 and grid.grid[x+1, y+1] <= 0: # Cell top right
    #     valid_actions.append((1, 1)) 

    return valid_actions

def a_star_search(grid:Grid, obstacle_threshold:float=50):
    '''Takes in a Grid object (which has an occupancy grid/array, start and end goals) and outputs an A* shortest path. Obstacles threshold unused for CA2
    Shamelessly ripped off from GeeksforGeeks https://www.geeksforgeeks.org/dsa/a-search-algorithm/'''
    cells_to_visit = []
    visited_cells = np.zeros(grid.shape)
    cell_details = [[Cell() for _ in range(grid.shape[1])] for _ in range(grid.shape[0])]

    starting_cell = cell_details[grid.starting_position[0]][grid.starting_position[1]]
    starting_cell.f, starting_cell.g, starting_cell.h = 0, 0, 0
    starting_cell.parent_coords = grid.starting_position

    heapq.heappush(cells_to_visit, (0.0, grid.starting_position[0],  grid.starting_position[1]))
    print("Start A* search")
    while cells_to_visit:
        
        cell = heapq.heappop(cells_to_visit)
        x, y = cell[1], cell[2]
        visited_cells[x][y] = 1
        valid_actions = get_valid_actions(grid, (x,y), obstacle_threshold)

        for action in valid_actions:
            neighbour_cell_x = x + action[0]
            neighbour_cell_y = y + action[1]
            if visited_cells[neighbour_cell_x][neighbour_cell_y] != 1 and grid.grid[neighbour_cell_x, neighbour_cell_y] <= obstacle_threshold: 
            # If neighbour cell not visited before and not a obstacle
                
                if (neighbour_cell_x, neighbour_cell_y) == grid.goal_position:
                    # Found reward destination
                    cell_details[neighbour_cell_x][neighbour_cell_y].parent_coords = (x,y)
                    print("Path found!")
                    return trace_path(cell_details, grid.goal_position)
                else:
                    neighbour_g = cell_details[x][y].g + 1 # Cost is cost of parent cell + 1, ie. distance from parent cell
                    neighbour_h = heuristic_cost(grid.goal_position, (neighbour_cell_x, neighbour_cell_y)) # Cost is Manhattan distance from destination
                    neighbour_f = neighbour_g + neighbour_h # Predicted total cost is distance from parent cell +1, plus the Manhattan distance to destination

                    # If new neighbour_f value is smaller than what's on the list, or neighbour cell has never been visited
                    if cell_details[neighbour_cell_x][neighbour_cell_y].f > neighbour_f or np.isinf(cell_details[neighbour_cell_x][neighbour_cell_y].f):
                        heapq.heappush(cells_to_visit, (neighbour_f, neighbour_cell_x, neighbour_cell_y))
                        cell_details[neighbour_cell_x][neighbour_cell_y].f = neighbour_f
                        cell_details[neighbour_cell_x][neighbour_cell_y].g = neighbour_g
                        cell_details[neighbour_cell_x][neighbour_cell_y].h = neighbour_h
                        cell_details[neighbour_cell_x][neighbour_cell_y].parent_coords = (x, y)
    
    # If the while loop is finished entirely, it means the destination tile was not found but all possible tiles have been explored
    print("Failed to find path")
    return None


def main(args=None):
    global is_simulation
    print("Starting path planning")
    rclpy.init(args=args)

    # Load the proper occupancy grid numpy array
    import os
    filepath = os.path.dirname(os.path.realpath(__file__))
    if is_simulation:
        map_array = np.load(filepath + '/ca2_sim_map.npy', allow_pickle=True)
        waypoint = WaypointNode(map_array, sim_goal_list, is_simulation)
    else:
        map_array = np.load(filepath + '/ca2_irl_map.npy', allow_pickle=True)
        waypoint = WaypointNode(map_array, irl_goal_list, is_simulation)

    # Start spinning the waypoint node and only stop once SystemExit error is raised within the node callback
    try:
        rclpy.spin(waypoint)
    except SystemExit:
        print("Shutting down")

    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()