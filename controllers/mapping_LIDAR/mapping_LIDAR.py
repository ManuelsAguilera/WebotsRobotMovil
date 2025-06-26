"""mapping_LIDAR controller."""

from controller import Robot, Display
import math

# Parámetros
TIME_STEP = 64
MAX_SPEED = 6.28

# Inicializar robot
robot = Robot()

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

# GPS e IMU
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

imu = robot.getDevice('imu')
imu.enable(TIME_STEP)

# Display
display = robot.getDevice('map_display')
display_width = display.getWidth()
display_height = display.getHeight()

# Crear fondo blanco
display.setColor(0xFFFFFF)
display.fillRectangle(0, 0, display_width, display_height)

# Función para convertir coordenadas del mundo al display
def world_to_display(x, y):
    x_disp = int((x + 0.75) / 1.5 * display_width)
    y_disp = display_height - int((y + 0.75) / 1.5 * display_height)
    return x_disp, y_disp

# Calcular índices para sectores
center_index = lidar_resolution // 2
sector_size = lidar_resolution // 20

# Índices para comportamiento (seguimos usando tu lógica)
front_indices = list(range(center_index - sector_size - 3, center_index + sector_size + 3))
left_indices = list(range(center_index - 3 * sector_size, center_index - sector_size))
right_indices = list(range(center_index + sector_size, center_index + 3 * sector_size))

estado = True
direccion_giro = None
contador_giro = 0
limite_giro = 200
robot_atrapado = False

# Bucle principal
while robot.step(TIME_STEP) != -1:
    if robot_atrapado:
        for motor in left_motors + right_motors:
            motor.setVelocity(0.0)
        print("Robot atrapado. Detenido.")
        break
    
    lidar_data = lidar.getRangeImage()
    gps_values = gps.getValues()
    imu_values = imu.getRollPitchYaw()
    robot_angle = imu_values[2]  # Yaw (ángulo en z)

    robot_x = gps_values[0]
    robot_y = gps_values[1]  # Si es un robot plano, algunos usan z en vez de y, depende del eje vertical de tu entorno
    
    display.setColor(0xFF0000)  # Rojo
    robot_disp_x, robot_disp_y = world_to_display(robot_x, robot_y)
    display.fillOval(robot_disp_x - 2, robot_disp_y - 2, 2, 2)  # Robot como círculo
    
    # Mapear puntos del LIDAR
    display.setColor(0x000000)  # Color negro para los puntos
    for i, distance in enumerate(lidar_data):
        if distance == float('inf') or distance > 1.5:
            continue  # Ignorar puntos muy lejanos

        # Ángulo relativo del rayo
        ray_angle = (i / (lidar_resolution - 1)) * lidar_fov - lidar_fov / 2
        
        # Ángulo absoluto en el mundo
        absolute_angle = robot_angle + ray_angle

        # Coordenadas absolutas del punto detectado
        obs_x = robot_x + distance * math.cos(absolute_angle)
        obs_y = robot_y + distance * math.sin(absolute_angle)

        # Convertir a display
        x_disp, y_disp = world_to_display(obs_x, obs_y)
        display.drawPixel(x_disp, y_disp)
    
    # Obtener distancias mínimas para navegación
    front = min([lidar_data[i] for i in front_indices if 0 <= i < lidar_resolution])
    left = min([lidar_data[i] for i in left_indices if 0 <= i < lidar_resolution])
    right = min([lidar_data[i] for i in right_indices if 0 <= i < lidar_resolution])

    print(f"Avanzar: {estado} → Front: {front:.2f} m | Left: {left:.2f} m | Right: {right:.2f} m")

    left_speed = 0.0
    right_speed = 0.0

    if estado is True:
        direccion_giro = None
        contador_giro = 0
        
        if front < 0.12:
            estado = False
            direccion_giro = "izquierda" if left > right else "derecha"
        else:
            left_speed = MAX_SPEED * 0.8
            right_speed = MAX_SPEED * 0.8

    elif estado is False:
        contador_giro += 1
        
        if contador_giro >= limite_giro:
            robot_atrapado = True
            continue
        
        if front >= 0.4 and left >= 0.213 and right >= 0.213:
            estado = True
            contador_giro = 0
        else:
            if direccion_giro == "izquierda":
                left_speed = -0.5 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED
            elif direccion_giro == "derecha":
                left_speed = 0.5 * MAX_SPEED
                right_speed = -0.5 * MAX_SPEED

    for motor in left_motors:
        motor.setVelocity(left_speed)
    for motor in right_motors:
        motor.setVelocity(right_speed)