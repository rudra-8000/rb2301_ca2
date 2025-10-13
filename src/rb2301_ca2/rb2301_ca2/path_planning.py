import numpy as np
import heapq
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
)
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from PIL import Image
from geometry_msgs.msg import Twist


np.set_printoptions(
    2, suppress=True, threshold=np.inf
)  # Print numpy arrays to specified d.p., suppress scientific notation (e.g. 1e-5), and do not truncate

set_logger_level("path_planning", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

max_translate_velocity = 1.0

is_simulation = True

goal_list = [(3.4, -3.6), (3.2, 0.2), (2.4, -3.6), (-0.4, -3.8)]


class WaypointNode(Node):
    '''Node to move robot to received waypoints, using pose info from either gazebo odometer or optitrack'''
    def __init__(self, map_array:np.array, goal_list:list, sim:bool=True):
        super().__init__('waypoint')
        self.get_logger().info("Starting WaypointNode")

        self.sim = sim

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.sim:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odomoter_callback, 2)
        else:
            qos_profile = QoSProfile(
                depth=2,
                reliability=ReliabilityPolicy.BEST_EFFORT
                )

            self.map_sub = self.create_subscription( 
                PoseStamped,
                '/vrpn_mocap/bingda/pose',
                self.optitrack_callback, 
                qos_profile
                )
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node       
        self.timer = self.create_timer(1, self.timer_callback)  # Runs at 20Hz. Can be changed.

        self.pose = None
        self.map_array = map_array
    
        self.waypoint_list = None
        self.current_waypoint_idx = 0

        self.goal_list = goal_list
        self.current_goal_idx = 0
        self.goal_reached = True

    def optitrack_callback(self, msg:PoseStamped):
        '''Callback to calculate 2D pose info from Optitrack node. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        x, y = msg.pose.position.x, msg.pose.position.y
        rpy_euler = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        heading = np.rad2deg(rpy_euler[2])
        self.pose = np.array((x,y,heading))
        return self.pose

    def odomoter_callback(self, msg):
        '''Callback to calculate 2D pose info from Gazebo odomoter. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        latest_pose_msg = msg.pose.pose 
        quat = latest_pose_msg.orientation
        rpy_euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        heading = np.rad2deg(rpy_euler[2])
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        return self.pose

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)


    def set_waypoints(self, waypoints:list):
        self.goal_reached = False
        self.waypoints = waypoints
        self.current_waypoint_idx = 0


    def timer_callback(self):
        """Controller loop"""
        if self.pose is None:
            return # Does not run if no pose received from Odom or Optitrack
        self.get_logger().debug(f"pose: {self.pose}")

        if self.goal_reached:
            if self.current_goal_idx >= len(self.goal_list):
                return # Finished exploring
            
            start_coords = self.pose[:2]
            start_coords = np.round(start_coords, 1)
            x_start, y_start, resolution = -1.0, -5.0, .2
            goal_coords = self.goal_list[self.current_goal_idx]

            start = (int(start_coords[0]//resolution - x_start//resolution), int(start_coords[1]//resolution - y_start//resolution))
            goal = (int(goal_coords[0]//resolution - x_start//resolution), int(goal_coords[1]//resolution - y_start//resolution))

            grid = Grid(self.map_array, starting_position=start, goal_position=goal)
            
            if not grid.check_grid_validity():
                self.get_logger().warn("Invalid start and/or goal")

            solution_path = a_star_search(grid)
            waypoint_list = convert_path_to_waypoints(solution_path)
            grid.draw_grid_map(waypoint_list, solution_path)

            coordinate_waypoints = []
            for point in waypoint_list:
                coordinate_waypoints.append((point[0]*resolution+x_start, point[1]*resolution+y_start))

            self.set_waypoints(coordinate_waypoints)

        else:
            if self.current_waypoint_idx == len(self.waypoints):
                self.get_logger().info("Goal reached!")
                self.goal_reached = True
                self.current_goal_idx += 1

            else:
                target_waypoint = np.array(self.waypoints[self.current_waypoint_idx])
                self.get_logger().debug(f"Target: {target_waypoint}")
                if np.linalg.norm(self.pose[:2] - target_waypoint) < 0.05:
                    
                    self.move_2D()
                    if self.current_waypoint_idx+1 != len(self.waypoints):
                        self.get_logger().info(f"Waypoint {target_waypoint} reached. Next target is {self.waypoints[self.current_waypoint_idx+1]}")
                    self.current_waypoint_idx += 1
                else:
                    x = target_waypoint[0] - self.pose[0]
                    y = target_waypoint[1] - self.pose[1]

                    if 0.0 < x < 0.15:
                        x = 0.15
                    elif -0.15 < x < 0.0:
                        x = -0.15
                    if 0.0 < y < 0.15:
                        y = 0.15
                    elif -0.15 < y < 0.0:
                        y = -0.15

                    self.move_2D(x, y)


class Grid():
    '''
    Atributes:
    grid_array : numpy array representing the occupancy grid
    starting_position : tuple of starting indices within the grid
    goal_position : tuple of goal indices within the grid. Works with negative indices as well
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
        # Use flood fill to check if there's a viable path between start and goal positions
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
        '''Creates an image of the maze and path taken. 
        Maze walls in blue, empty space in white, path taken in green and waypoints in red'''
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
    # Uses Manhattan distance to calculate heuristic cost from the cell to the destination
    return abs(destination[0]-cell_coordinates[0]) + abs(destination[1]-cell_coordinates[1])

def trace_path(cell_details:list, destination:tuple):
    # Returns the final, optimised A* path as an ordered list of cells to visit
    path = []
    x, y = destination[0], destination[1]

    while not (cell_details[x][y].parent_coords == (x,y)):
        path.append((x,y))
        x, y = cell_details[x][y].parent_coords
    
    path.append((x, y))
    path.reverse()

    return path


def move_direction(starting_cell, destination_cell):
    return (destination_cell[0]-starting_cell[0], destination_cell[1]-starting_cell[1])


def convert_path_to_waypoints(path:list):
    waypoints = [path[0]]
    for idx in range(1, len(path)-1):
        if move_direction(path[idx-1], path[idx]) != move_direction(path[idx], path[idx+1]):
            waypoints.append(path[idx])
    waypoints.append(path[-1])

    return waypoints



            
class Cell:
    def __init__(self, parent_coords:tuple=(0,0)):
        self.parent_coords = parent_coords
        self.f = np.inf # Total cost
        self.g = np.inf # Cost from start position
        self.h = 0 # Heuristic cost to destination (reward) cell

def get_valid_actions(grid:Grid, coordinates:tuple, obstacle_threshold:float):
    # Returns list of valid actions based on grid position. Currently set to only allow 4 cardinal direction movement
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
    import os
    filepath = os.path.dirname(os.path.realpath(__file__))
    map_array = np.load(filepath + '/ca2_sim_map.npy')
    print(map_array.dtype)
    irl_map = np.zeros((15,10), dtype=np.int8)
    irl_map[0,:] = 99
    irl_map[-1,:] = 99
    irl_map[:,0] = 99
    irl_map[:,-1] = 99
    irl_map[11,8] = 99
    irl_map[9,6:9] = 99
    irl_map[13,6] = 99
    irl_map[10,6] = 99
    irl_map[12:14,4] = 99
    irl_map[12,2] = 99
    irl_map[9,1:5] = 99
    irl_map[8,4] = 99
    irl_map[3:7,7] = 99
    irl_map[6,6] = 99
    irl_map[3,6] = 99
    irl_map[2:6,4] = 99
    irl_map[3,1:4] = 99
    irl_map[3:6,1:3] = 99
    irl_map[1,2] = 99


    grid = Grid(irl_map)
    grid.draw_grid_map()
    np.save('ca2_irl_map.npy', grid)

    # print("Starting path planning")
    # rclpy.init(args=args)

    # import os
    # filepath = os.path.dirname(os.path.realpath(__file__))
    # if is_simulation:
    #     map_array = np.load(filepath + '/ca2_sim_map.npy')
    # else:
    #     map_array = np.load(filepath + '/ca2_irl_map.npy')
    
    # waypoint = WaypointNode(map_array, goal_list)
    # while waypoint.pose is None:
    #     rclpy.spin_once(waypoint)

    # rclpy.spin(waypoint)
    
    # rclpy.shutdown()


if __name__ == '__main__':
    main()



