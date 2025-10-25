import numpy as np
import xml.etree.ElementTree as ET
import os
from shutil import move, copy2

randomise = True

workspace_directory = os.path.dirname(os.path.realpath(__file__))[:-20]
overwrite_file = workspace_directory + '/rb2301_gz/worlds/obstacle_course_world_fp.sdf'
original_file = workspace_directory + '/rb2301_gz/worlds/obstacle_course_world_fp_original.sdf'

# def modify_wall_location():
#     obstacle = ET.Element("include")
#     uri = ET.Element("uri")
#     uri.text = obstacle_model
#     obstacle.append(uri)
#     name = ET.Element("name")
#     name.text = f'coke{n}'
#     obstacle.append(name)
#     pose = ET.Element("pose")
#     pose.text = f'{x} {y} 0 0 0 0'
#     obstacle.append(pose)
#     return obstacle


# def add_coke_element(x, y, n):
#     obstacle = ET.Element("include")
#     uri = ET.Element("uri")
#     uri.text = obstacle_model
#     obstacle.append(uri)
#     name = ET.Element("name")
#     name.text = f'coke{n}'
#     obstacle.append(name)
#     pose = ET.Element("pose")
#     pose.text = f'{x} {y} 0 0 0 0'
#     obstacle.append(pose)
#     return obstacle

def modify_sdf_file():
    with open(original_file, 'r') as original:
        with open(overwrite_file, 'w') as overwrite:
            original_lines = original

    if randomise:
        print("Modifying gate location...")
        tree = ET.parse(overwrite_file)
        root = tree.getroot()
        world = root[0]

        x_offset = -np.random.random()*0.8 
        y_offset = -np.random.random()*0.6

        gate_y_start = -np.random.random() * 0.7 - 1.4
        gate_y_end = gate_y_start - 1.6

        for element in reversed(world): # Remove all coke obstacles
            if element.tag == 'include':
                pose = element[2].text.split(' ')

                if element[1].text == 'nist_maze_wall_120_configurable_left':
                    element[2].text = f'{4.2+x_offset} {gate_y_start+y_offset} -0.7 0 0 1.57079633'
                elif element[1].text == 'nist_maze_wall_120_configurable_right':
                    element[2].text = f'{4.2+x_offset} {gate_y_end+y_offset} -0.7 0 0 1.57079633'
                else:
                    if float(pose[0]) > 1.0:
                        pose[0] = str(float(pose[0]) + x_offset)
                    if float(pose[1]) < -2.6:
                        pose[1] = str(float(pose[1]) + y_offset) 
                    element[2].text = ' '.join(pose)    
            elif element.tag == 'actor':
                for waypoint_idx in range(3):
                    waypoint_pose = element[2][3][waypoint_idx][1].text.split(' ')
                    waypoint_pose[0] = str(float(waypoint_pose[0]) + x_offset)
                    waypoint_pose[1] = str(float(waypoint_pose[1]) + y_offset)
                    element[2][3][waypoint_idx][1].text = ' '.join(waypoint_pose)  
                    
        tree.write(overwrite_file)

if __name__ == '__main__':
    modify_sdf_file()