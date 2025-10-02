import numpy as np
import heapq
import rclpy
from rclpy.node import Node
# from std_msgs.msg import String
# from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
)
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
# from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from PIL import Image
from geometry_msgs.msg import Twist


np.set_printoptions(
    2, suppress=True, threshold=np.inf
)  # Print numpy arrays to specified d.p., suppress scientific notation (e.g. 1e-5), and do not truncate


class WaypointNode(Node):
    '''Node to move robot to received waypoints, using pose info from gazebo/optitrack to assist in movement'''
    def __init__(self, waypoints:list, sim:bool=True):
        super().__init__('waypoint')
        self.get_logger().info("Starting WaypointNode")

        self.sim = sim

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.sim:
            self.subscription = self.create_subscription(Odometry, 'odom', self.listener_callback, 2)
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node       
        self.timer = self.create_timer(1, self.timer_callback)  # Runs at 20Hz. Can be changed.
        
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.pose = None


    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        twist_msg = Twist()
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def listener_callback(self, msg):
        '''This callback will run everytime the rclpy executor spins'''
        latest_pose_msg = msg.pose.pose 
        quat = latest_pose_msg.orientation
        rpy_euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        heading = np.rad2deg(rpy_euler[2])
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        return self.pose
    

    def timer_callback(self):
        """Controller loop"""
        if self.pose is None:
            return # Does not run if no pose received
        print(self.pose)
        
        if self.current_waypoint_idx == len(self.waypoints):
            self.get_logger().info("Destination reached, shutting down!")
            self.destroy_node()
            rclpy.shutdown()
        else:
            target_waypoint = np.array(self.waypoints[self.current_waypoint_idx])
            if np.linalg.norm(self.pose[:2] - target_waypoint) < 0.1:
                self.current_waypoint_idx += 1
            else:
                x = target_waypoint[0] - self.pose[0]
                y = target_waypoint[1] - self.pose[1]
                self.move_2D(x, y)




class OptitrackNode(Node):
    '''ROS2 node to read pose data published by optitrack'''
    def __init__(self):
        super().__init__('optitrack')
        self.get_logger().info("Starting Optitrack subscriber")

        qos_profile = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.map_sub = self.create_subscription( 
            PoseStamped,
            '/vrpn_mocap/bingda/pose',
            self.sub_optitrack_callback, 
            qos_profile
            )
        
        # self.timer = self.create_timer(0.05, self.timer_callback)
        self.pose = None
        self.timer = self.create_timer(1, self.timer_callback)  # Runs at 20Hz. Can be changed.

    def sub_optitrack_callback(self, msg:PoseStamped):
        '''This callback will run everytime the rclpy executor spins'''
        x, y = msg.pose.position.x, msg.pose.position.y
        rpy_euler = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        heading = np.rad2deg(rpy_euler[2])
        self.pose = [x,y,heading]

    def timer_callback(self):
        """Controller loop"""

        if self.pose is None:
            return # Does not run if no pose received
        print(self.pose )



class MapNode(Node):
    '''ROS2 node to read map data published by the mapper'''
    def __init__(self):
        super().__init__('MapNode')

        # Subscribe to the scan ROS topic, which is what the lidar (whether simulated or real) publishes to
        qos_profile = QoSProfile(
            # history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, # Need to use this setting since the publishing map node uses transient local
            # reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription( 
            OccupancyGrid,
            'map',
            # 'global_costmap',
            self.map_sub_callback, 
            qos_profile
            )
        
        self.map = None

    def map_sub_callback(self, msg):
        '''This callback will run everytime the rclpy executor spins'''
        # print(msg.info, type(msg.data), '\n')
        x_size, y_size, resolution = msg.info.width, msg.info.height, msg.info.resolution
        self.map = np.array(msg.data) 
        self.map = np.resize(self.map, (y_size, x_size))[::-1].transpose() # Resize map to given dimensions, then transpose so x is row and y is column
        self.map[self.map <= 50] = 0
        self.map[self.map > 50] = 1
        
        print(self.map)
        print(self.map.shape)


class Grid():
    '''
    Atributes:
    grid_array : numpy array representing the occupancy grid
    generate_grid_size : tuple to represent occupancy grid to generate, if grid_array not given
    num_obstacles : number of obstacles to place within generated_grid, if grid_array not given. Optional; if none given, defaults to having 25% of cells be obstacles
    starting_position : tuple of starting coordinates / indices within the grid
    goal_position : tuple of goal coordinates / indices within the grid. If negative indices given, will count backwards 
    '''
    def __init__(self, grid_array:np.array=np.array([]), generate_grid_size:tuple=None, num_obstacles:int=None, 
                 starting_position:tuple=(0,0), goal_position:tuple=(-1,-1)):

        if grid_array.any():
            self.grid = grid_array
        elif generate_grid_size:
            self.grid = self.generate_random_grid(generate_grid_size, num_obstacles, goal_position)
        else:
            print("Please input either a grid or grid_size to generate")
            raise AttributeError
        
        self.shape = self.grid.shape
        self.starting_position = starting_position

        # If goal position given with negative index, need to convert to +ve
        if goal_position[0] < 0:
            reward_x = self.shape[0] + goal_position[0]
        else:
            reward_x = goal_position[0]
        if goal_position[1] < 0:
            reward_y = self.shape[1] + goal_position[1]
        else:
            reward_y = goal_position[1]   

        self.goal_position = (reward_x, reward_y)

    def generate_random_grid(self, grid_size:tuple, num_obstacles:int=None, goal_position:tuple=(-1,-1)):
        # Generates a random grid using the given 2D dimensions, and num_obstacles
        if not num_obstacles:
            # If no num_obstacles given, use the default where 25% of the grid are obstacles
            num_obstacles = round(grid_size[0] * grid_size[1] * 0.25)

        while True:
            # Continuous loop to generate grid and check if it's valid. If valid, break out of the loop
            grid = np.zeros(grid_size)
            grid[goal_position] = 1 # Set goal
            available_grids = np.arange(grid_size[0] * grid_size[1])

            # The top left and bottom right of the grids are start & finish and should not have obstacles
            available_grids = np.delete(available_grids, 0)
            available_grids = np.delete(available_grids, -1)  # EDIT: Replace this with actual goal_position
            obstacle_list = []
            
            for _ in range(num_obstacles):
                # Randomly place obstacles in the remaining available grid tiles
                random_grid = np.random.choice(available_grids)
                obstacle_list.append(random_grid)
                obstacle_x, obstacle_y = random_grid // grid_size[0], random_grid % grid_size[0]
                grid[obstacle_x, obstacle_y] = -1 # Set obstacles to -1
                np.delete(available_grids, random_grid-1)

            if self.check_grid_validity(grid.copy()):
                # If grid is valid (ie. path exists between start and goal), then break. Else generate another grid
                break

        return grid

    def check_grid_validity(self):
        # Use flood fill to check if there's a viable path
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


def draw_path(grid:Grid, path:list, waypoints:list):
    '''Creates an image of the maze and path taken. 
    Maze walls in blue, empty space in white, path taken in green and waypoints in red'''
    image_grid = np.ones((grid.grid.shape[0],grid.grid.shape[1],3), dtype=np.uint8)
    image_grid[grid.grid == 0] = (255,255,255)
    image_grid[grid.grid == 1] = (0,0,255)
    for x, y in path:
        image_grid[x][y] = (0,255,0)

    for point in waypoints:
        image_grid[point] = (255,0,0)
    image_grid = image_grid[::-1]
    img = Image.fromarray(image_grid, 'RGB')

    # Resize image
    base_width = 500
    wpercent = (base_width / float(img.size[0]))
    hsize = int((float(img.size[1]) * float(wpercent)))
    img = img.resize((base_width, hsize), Image.Resampling.NEAREST)

    img.show()
            
            
class Cell:
    def __init__(self, parent_coords:tuple=(0,0)):
        self.parent_coords = parent_coords
        self.f = np.inf # Total cost
        self.g = np.inf # Cost from start position
        self.h = 0 # Heuristic cost to destination (reward) cell

def get_valid_actions(grid:Grid, coordinates:tuple):
    # Returns list of valid actions based on grid position. Currently set to only allow 4 cardinal direction movement
    x, y = coordinates
    valid_actions = []
    if x > 0 and grid.grid[x-1, y] <= 0: # Cell below
        valid_actions.append((-1, 0)) 

    if x < grid.shape[0]-1 and grid.grid[x+1, y] <= 0: # Cell above
        valid_actions.append((1, 0))

    if y > 0 and grid.grid[x, y-1] <= 0: # Cell left
        valid_actions.append((0, -1))

    if y < grid.shape[1]-1 and grid.grid[x, y+1] <= 0: # Cell right
        valid_actions.append((0, 1))
    # print(grid.grid)
    # print(grid.grid[x-1:x+2, y-1:y+2])
    return valid_actions

def a_star_search(grid:Grid):
    cells_to_visit = []
    visited_cells = np.zeros(grid.shape)
    cell_details = [[Cell() for _ in range(grid.shape[1])] for _ in range(grid.shape[0])]

    starting_cell = cell_details[grid.starting_position[0]][grid.starting_position[1]]
    starting_cell.f, starting_cell.g, starting_cell.h = 0, 0, 0
    starting_cell.parent_coords = grid.starting_position

    heapq.heappush(cells_to_visit, (0.0, grid.starting_position[0],  grid.starting_position[1]))
    print(cells_to_visit)
    print("Start A* search")
    while cells_to_visit:
        
        cell = heapq.heappop(cells_to_visit)
        x, y = cell[1], cell[2]
        visited_cells[x][y] = 1
        # print(grid.grid[x])

        # print(cell, cells_to_visit,)

        # valid_action_indices = np.flatnonzero(~np.isnan(grid.q_table[x, y]))
        # valid_actions = [action_list[i] for i in valid_action_indices]
        valid_actions = get_valid_actions(grid, (x,y))
        # print(len(cells_to_visit))
        for action in valid_actions:
            neighbour_cell_x = x + action[0]
            neighbour_cell_y = y + action[1]
            if visited_cells[neighbour_cell_x][neighbour_cell_y] != 1 and grid.grid[neighbour_cell_x, neighbour_cell_y] <= 0: 
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
                        # if visited_cells[neighbour_cell_x][neighbour_cell_y] == 0:
                        # if (neighbour_f, neighbour_cell_x, neighbour_cell_y) not in cells_to_visit:
                        heapq.heappush(cells_to_visit, (neighbour_f, neighbour_cell_x, neighbour_cell_y))
                        # visited_cells[neighbour_cell_x][neighbour_cell_y] = -1 # Mark as in heap
                        cell_details[neighbour_cell_x][neighbour_cell_y].f = neighbour_f
                        cell_details[neighbour_cell_x][neighbour_cell_y].g = neighbour_g
                        cell_details[neighbour_cell_x][neighbour_cell_y].h = neighbour_h
                        cell_details[neighbour_cell_x][neighbour_cell_y].parent_coords = (x, y)
    
    print(np.flatnonzero(visited_cells).shape)
    # If the while loop is finished entirely, it means the destination tile was not found but all possible tiles have been explored
    print("Failed to find path")
    return None


def main(args=None):
    print("Starting path planning")
    # rclpy.init(args=args)

    # mapper = MapNode()
    # # opti = OptitrackNode()
    # # odom = OdomNode()

    # rclpy.spin_once(mapper)
    map = np.load('/home/marmot/Documents/rb2301/map.npy')
    resolution, x_start, y_start = 0.15, -0.15, -4.2

    start, goal = (6,5), (28,7)
    # start, goal = (-5, -5), (-2,-2)
    # start, goal = (4,8), (4,5) 
    # np.save('map.npy', mapper.map)
    grid = Grid(map, starting_position=start, goal_position=goal)
    # 
    print(grid.grid.shape, grid.grid[start], grid.grid[goal])

    if grid.check_grid_validity():
        print("Map start and goal valid")
    else:
        print("Invalid start and goal")
    solution = a_star_search(grid)
    print(f"Solution: {solution}")
    waypoints = convert_path_to_waypoints(solution)
    print(waypoints)
    coordinate_waypoints = []
    for waypoint in waypoints:
        coordinate_waypoints.append((waypoint[0]*resolution+x_start, waypoint[1]*resolution+y_start))
    
    print(coordinate_waypoints)

    draw_path(grid, solution, waypoints)
    # mapper.destroy_node()

    # waypoint = WaypointNode(waypoints)
    # rclpy.spin(waypoint)
    # rclpy.shutdown()


if __name__ == '__main__':
    # grid = Grid(generate_grid_size=(8,8), num_obstacles=round(8*8*0.75))
    # res = a_star_search(grid)
    # print(grid.grid, "\n", res)
    # draw_path(grid, res)
    main()
    # arr = np.load('/home/marmot/Downloads/rb2301_nav/map.npy')
    # print(arr, arr.max(), arr.min())

