import numpy as np
from controller import Supervisor
from PIL import Image
import math

MAP_SIZEmt = 2
MAP_RESOLUTION = 0.08  # metros por celda


MAP_SIZE = int(MAP_SIZEmt / MAP_RESOLUTION)  # número de celdas por lado

MAP_CENTER = MAP_SIZE // 2
occupancy_grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)




def world_to_map(x, y):
    mx = int(MAP_CENTER + x / MAP_RESOLUTION)
    my = int(MAP_CENTER - y / MAP_RESOLUTION)
    return mx, my

def update_occupancy_grid(robot_pose, lidar_data, lidar_resolution, lidar_fov):
    robot_x, robot_y, robot_theta = robot_pose
    angle_min = -lidar_fov / 2
    angle_step = lidar_fov / lidar_resolution

    for i in range(lidar_resolution):
        distance = lidar_data[i]
        if distance < 0.01 or distance > 2.5:
            continue
        angle = angle_min + i * angle_step
        lx = distance * math.cos(angle)
        ly = distance * math.sin(angle)
        gx = robot_x + lx * math.cos(robot_theta) - ly * math.sin(robot_theta)
        gy = robot_y + lx * math.sin(robot_theta) + ly * math.cos(robot_theta)
        mx, my = world_to_map(gx, gy)
        print(f"Robot está en la celda del grid: ({mx}, {my})")
        if 0 <= mx < MAP_SIZE and 0 <= my < MAP_SIZE:
            occupancy_grid[mx, my] = 2  # Obstáculo

def save_map_image(filename="map.png",robot_pose=None):
    img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
    img[occupancy_grid == 2] = [0, 0, 0]
    img[occupancy_grid == 0] = [128, 128, 128]
    if robot_pose:
        robot_x, robot_y, robot_theta = robot_pose
        mx, my = world_to_map(robot_x, robot_y)
        if 0 <= mx < MAP_SIZE and 0 <= my < MAP_SIZE:
            img[mx, my] = [255, 0, 0]
    Image.fromarray(img).save(filename)