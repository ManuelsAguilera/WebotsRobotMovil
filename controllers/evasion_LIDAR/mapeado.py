import numpy as np
from controller import Supervisor
from PIL import Image
import math

MAP_SIZEmt = 2
MAP_RESOLUTION = 0.01  # metros por celda

MAP_SIZE = int(MAP_SIZEmt / MAP_RESOLUTION)  # número de celdas por lado
print("Tamaño del mapa:", MAP_SIZE, "celdas")
MAP_CENTER = MAP_SIZE // 2
occupancy_grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)

def world_to_map(x, y):
    mx = int(MAP_CENTER + x / MAP_RESOLUTION)
    my = int(MAP_CENTER - y / MAP_RESOLUTION)
    return mx, my

def bresenham(x0, y0, x1, y1):
    """Algoritmo de Bresenham para trazar una línea entre dos puntos en una cuadrícula. Marcar lo q mira el LIDAR"""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points

def update_occupancy_grid(robot_pose, lidar_data, lidar_resolution, lidar_fov):
    robot_x, robot_z, robot_theta = robot_pose
    angle_min = -lidar_fov / 2
    angle_step = lidar_fov / lidar_resolution

    mx0, mz0 = world_to_map(robot_x, robot_z)

    for i in range(lidar_resolution):
        distance = lidar_data[i]
        if distance < 0.05 or distance > 1.5:
            continue
        angle = angle_min + i * angle_step
        lx = distance * math.cos(angle)
        lz = distance * math.sin(angle)
        gx = robot_x + lx * math.cos(robot_theta) - lz * math.sin(robot_theta)
        gz = robot_z + lx * math.sin(robot_theta) + lz * math.cos(robot_theta)
        mx1, mz1 = world_to_map(gx, gz)

       # Marca celdas libres entre el robot y el obstáculo
        for (mx, mz) in bresenham(mx0, mz0, mx1, mz1)[:-1]:
            if 0 <= mx < MAP_SIZE and 0 <= mz < MAP_SIZE:
                if occupancy_grid[mx, mz] == 0:
                    # Marca como libre si no es un obstáculo
                    if occupancy_grid[mx, mz] != 2:
                        occupancy_grid[mx, mz] = 1  # Siempre marca como libre
                    else:
                        occupancy_grid[mx, mz] = 0
                

        # Marca la celda del obstáculo
        if 0 <= mx1 < MAP_SIZE and 0 <= mz1 < MAP_SIZE:
            if occupancy_grid[mx1, mz1] == 0:
                occupancy_grid[mx1, mz1] = 2  # Obstáculo

def map_array(robot_pose=None):
    img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
    img[occupancy_grid == 2] = [0, 0, 255] #Ocupado
    img[occupancy_grid == 1] = [0, 255, 0]  # Área libre
    img[occupancy_grid == 0] = [255, 255, 255]
    if robot_pose:
        robot_x, robot_y, robot_theta = robot_pose
        mx, my = world_to_map(robot_x, robot_y)
        if 0 <= mx < MAP_SIZE and 0 <= my < MAP_SIZE:
            img[mx, my] = [255, 0, 0]
    return img,MAP_SIZE