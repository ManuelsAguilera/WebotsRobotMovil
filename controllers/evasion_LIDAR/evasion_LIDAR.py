"""evasion_LIDAR controller."""

from controller import Robot,Keyboard
from PIL import Image
import math
from mapeado import update_occupancy_grid,map_array

# Parámetros
TIME_STEP = 128
MAX_SPEED = 6.28


def obtenerPose(gps_val, comp_val):
    robot_x = gps_val[1]
    robot_z = gps_val[0]  # Usa z, no y
    theta = math.atan2(comp_val[1], comp_val[0])
    return robot_x, robot_z, theta



if __name__ == "__main__":
    # Inicializar robot
    robot = Robot()
    #Inicializar controles del teclado
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

    estado = True # True = avanzar, False = evadir
    direccion_giro = None

    # Bucle principal
    while robot.step(TIME_STEP) != -1:
        lidar_data = lidar.getRangeImage()

        # Obtener distancias mínimas en cada sector
        front = min([lidar_data[i] for i in front_indices if i >= 0 and i < lidar_resolution])
        left = min([lidar_data[i] for i in left_indices if i >= 0 and i < lidar_resolution])
        right = min([lidar_data[i] for i in right_indices if i >= 0 and i < lidar_resolution])

        #print(f"Avanzar: {estado} → Front: {front:.2f} m | Left: {left:.2f} m | Right: {right:.2f} m")

        left_speed = 0.0
        right_speed = 0.0

        if estado is True:
            direccion_giro = None  # Resetear dirección cuando avanza

            if front < 0.08:  # Objeto demasiado cerca al frente
                estado = False
                if left > right:
                    direccion_giro = "izquierda"
                else:
                    direccion_giro = "derecha"
            else:
                left_speed = MAX_SPEED * 0.8
                right_speed = MAX_SPEED * 0.8

        elif estado is False:
            if front >= 0.3 and left >= 0.114 and right >= 0.114:
                estado = True
            else:
                if direccion_giro == "izquierda":
                    left_speed = -0.5 * MAX_SPEED
                    right_speed = 0.5 * MAX_SPEED
                elif direccion_giro == "derecha":
                    left_speed = 0.5 * MAX_SPEED
                    right_speed = -0.5 * MAX_SPEED

        robot_pose = obtenerPose(gps.getValues(), compass.getValues())
        update_occupancy_grid(robot_pose, lidar_data, lidar_resolution, lidar_fov)
        # Actualizar mapa de ocupación
        img,map_size = map_array(robot_pose)
        # Obtener tamaño del display
        display_width = display.getWidth()
        display_height = display.getHeight()

        # Redimensionar la imagen del mapa al tamaño del display
        img_resized = Image.fromarray(img).resize((display_width, display_height), resample=Image.NEAREST)
        display_image = display.imageNew(img_resized.tobytes(), display.RGB, display_width, display_height)
        
        display.imagePaste(display_image, 0, 0, False)
        display.imageDelete(display_image)
        #print(robot_pose)

        # Aplicar velocidades
        for motor in left_motors:
            motor.setVelocity(left_speed)
        for motor in right_motors:
            motor.setVelocity(right_speed)

