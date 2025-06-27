from controller import Robot, Keyboard
from PIL import Image
import math
from mapeado import update_occupancy_grid, map_array, world_to_map, occupancy_grid, porcentaje_explorado
from planificador import dijkstra

# Parámetros
TIME_STEP = 128
MAX_SPEED = 6.28
rango_activacion_x = (-0.438413, -1)
rango_activacion_y = (-0.438413, -0.6)

def dentro_de_rango(pose, rango_x, rango_y):
    x, y = pose[0], pose[1]
    return (rango_x[0] <= x <= rango_x[1]) and (rango_y[0] <= y <= rango_y[1])

def obtenerPose(gps_val, comp_val):
    robot_x = gps_val[1]
    robot_z = gps_val[0]  # Usa z, no y
    theta = math.atan2(comp_val[1], comp_val[0])
    return robot_x, robot_z, theta

def seguir_camino(robot_pose, camino):
    if not camino:
        return 0.0, 0.0

    mx, my = world_to_map(robot_pose[0], robot_pose[1])

    if (mx, my) == camino[0]:
        camino.pop(0)

    if not camino:
        return 0.0, 0.0

    siguiente = camino[0]
    dx = (siguiente[0] - mx)
    dy = -(siguiente[1] - my)

    objetivo_angulo = math.atan2(dy, dx)
    angulo_error = objetivo_angulo - robot_pose[2]
    angulo_error = (angulo_error + math.pi) % (2 * math.pi) - math.pi

    if abs(angulo_error) > 0.2:
        giro = max(min(angulo_error, 1), -1)
        left_speed = -0.3 * giro * MAX_SPEED
        right_speed = 0.3 * giro * MAX_SPEED
    else:
        left_speed = 0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED

    return left_speed, right_speed

if __name__ == "__main__":
    # Inicializar robot
    robot = Robot()
    keyboard = Keyboard()
    keyboard.enable(TIME_STEP)

    #Iniciar GPS y Compass
    gps = robot.getDevice('gps')
    gps.enable(TIME_STEP)
    compass = robot.getDevice('compass')
    compass.enable(TIME_STEP)
    
    #Encontrar display
    display = robot.getDevice('display')

    # Motores
    left_motors = [
        robot.getDevice('ruedaIUp'),
        robot.getDevice('ruedaIDown')
    ]
    right_motors = [
        robot.getDevice('ruedaDUp'),
        robot.getDevice('ruedaDDown')
    ]
    for motor in left_motors + right_motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    # LIDAR
    lidar = robot.getDevice('LIDAR')
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()

    lidar_resolution = lidar.getHorizontalResolution()
    lidar_fov = lidar.getFov()
    
    # Calcular índices para sectores
    center_index = lidar_resolution // 2
    sector_size = lidar_resolution // 20  # 5% del total

    # Índices
    front_indices = list(range(center_index - sector_size - 3, center_index + sector_size + 3))
    left_indices = list(range(center_index - 3 * sector_size, center_index - sector_size))
    right_indices = list(range(center_index + sector_size, center_index + 3 * sector_size))

    #destino_world = (-0.438413, -0.810711) # barra amarilla
    destino_world = (-0.812461, --0.311011) # barra verde
    destino_map = world_to_map(destino_world[0], destino_world[1])
    camino = None
    
    estado = True # True = avanzar, False = evadir
    direccion_giro = None
    dijkstra_activado = False
    
    # Bucle principal
    while robot.step(TIME_STEP) != -1:
        lidar_data = lidar.getRangeImage()
        
        robot_pose = obtenerPose(gps.getValues(), compass.getValues())
        
        update_occupancy_grid(robot_pose, lidar_data, lidar_resolution, lidar_fov)
        
        print(f"Mapa explorado: {porcentaje_explorado():.2f}%")
        
        tecla = keyboard.getKey()
        if not dijkstra_activado and dentro_de_rango(robot_pose, rango_activacion_x, rango_activacion_y):
            inicio_map = world_to_map(robot_pose[0], robot_pose[1])

            if occupancy_grid[destino_map[0], destino_map[1]] == 2:
                print("Destino bloqueado por obstáculo. No se puede planificar.")
                camino = None
            else:
                camino = dijkstra(inicio_map, destino_map)
                if camino:
                    print("Camino encontrado:", camino)
                    dijkstra_activado = True
                else:
                    print("No se encontró camino.")

        left_speed, right_speed = (0.0, 0.0)

        if camino:
            # Movimiento por planificación
            left_speed, right_speed = seguir_camino(robot_pose, camino)

            if not camino:
                print("Destino alcanzado.")
                camino = None  # Volver a modo reactivo
        
        else:
            # Movimiento reactivo (mientras no hay camino)
            front = min([lidar_data[i] for i in front_indices if i >= 0 and i < lidar_resolution])
            left = min([lidar_data[i] for i in left_indices if i >= 0 and i < lidar_resolution])
            right = min([lidar_data[i] for i in right_indices if i >= 0 and i < lidar_resolution])

            if estado:
                direccion_giro = None  # Resetear dirección cuando avanza
                if front < 0.08:  # Objeto demasiado cerca del frente
                    estado = False
                    if left > right:
                        direccion_giro = "izquierda"
                    else:
                        direccion_giro = "derecha"
                else:
                    left_speed = MAX_SPEED * 0.8
                    right_speed = MAX_SPEED * 0.8

            else:
                if front >= 0.2 and left >= 0.1 and right >= 0.1:
                    estado = True
                else:
                    if direccion_giro == "izquierda":
                        left_speed = -0.5 * MAX_SPEED
                        right_speed = 0.5 * MAX_SPEED
                    elif direccion_giro == "derecha":
                        left_speed = 0.5 * MAX_SPEED
                        right_speed = -0.5 * MAX_SPEED

        # Visualizar mapa
        img, map_size = map_array(robot_pose)

        if camino:
            for (mx, my) in camino:
                if 0 <= mx < map_size and 0 <= my < map_size:
                    img[mx, my] = [255, 0, 255]  # Camino en magenta
        
        # Obtener tamaño del display
        display_width = display.getWidth()
        display_height = display.getHeight()

        # Redimensionar la imagen del mapa al tamaño del display
        img_resized = Image.fromarray(img).resize((display_width, display_height), resample=Image.NEAREST)
        display_image = display.imageNew(img_resized.tobytes(), display.RGB, display_width, display_height)
        
        display.imagePaste(display_image, 0, 0, False)
        display.imageDelete(display_image)
        
        # Aplicar velocidades
        for motor in left_motors:
            motor.setVelocity(left_speed)
        for motor in right_motors:
            motor.setVelocity(right_speed)