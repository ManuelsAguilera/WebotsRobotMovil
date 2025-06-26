"""evasion_LIDAR controller."""

from controller import Robot

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

    print(f"Avanzar: {estado} → Front: {front:.2f} m | Left: {left:.2f} m | Right: {right:.2f} m")

    left_speed = 0.0
    right_speed = 0.0

    if estado is True:
        direccion_giro = None  # Resetear dirección cuando avanza

        if front < 0.12:  # Objeto demasiado cerca al frente
            estado = False
            if left > right:
                direccion_giro = "izquierda"
            else:
                direccion_giro = "derecha"
        else:
            left_speed = MAX_SPEED * 0.8
            right_speed = MAX_SPEED * 0.8

    elif estado is False:
        # ✔️ Nueva condición: No solo mirar frente, también laterales
        if front >= 0.4 and left >= 0.213 and right >= 0.213:
            estado = True
        else:
            if direccion_giro == "izquierda":
                left_speed = -0.5 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED
            elif direccion_giro == "derecha":
                left_speed = 0.5 * MAX_SPEED
                right_speed = -0.5 * MAX_SPEED

    # Aplicar velocidades
    for motor in left_motors:
        motor.setVelocity(left_speed)
    for motor in right_motors:
        motor.setVelocity(right_speed)