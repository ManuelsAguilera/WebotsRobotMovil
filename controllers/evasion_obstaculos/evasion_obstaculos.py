"""evasion_obstaculos controller."""

from controller import Robot, Motor, DistanceSensor

TIME_STEP = 64 # int(robot.getBasicTimeStep())
robot = Robot()

sensores = []
nombreSensores = ['sensorDer', 'sensorIzq']
for i in range(2):
    sensores.append(robot.getDevice(nombreSensores[i]))
    sensores[i].enable(TIME_STEP)

ruedas = []
nombreRuedas = ['ruedaIUp', 'ruedaDUp', 'ruedaIDown', 'ruedaDDown']
#nombreRuedas = ['ruedaDDown', 'ruedaDUp', 'ruedaIUp', 'ruedaIDown']
for i in range(4):
    ruedas.append(robot.getDevice(nombreRuedas[i]))
    ruedas[i].setPosition(float('inf'))
    ruedas[i].setVelocity(0.0)

contadorObstaculos = 0
while robot.step(TIME_STEP) != -1:
    velIzq = 3.0
    velDer = 3.0
    if contadorObstaculos > 0:
        contadorObstaculos -= 1
        velIzq = 3.0
        velDer = -3.0
    else:
        for i in range(2):
            if sensores[i].getValue() < 950.0:
                contadorObstaculos = 100
    ruedas[0].setVelocity(velIzq)
    ruedas[1].setVelocity(velDer)
    ruedas[2].setVelocity(velIzq)
    ruedas[3].setVelocity(velDer)