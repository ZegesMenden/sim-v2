import matplotlib.pyplot as plt
import numpy as np
import random

from navMath import orientation, PID, FSF
from physicsMath import DOF3, actuator
from motors import rocketMotor, motorType
from dataManagement import dataLogger
from dataVisualisation import dataVisualiser
from aero import getDrag

DEG_TO_RAD = np.pi / 180 
RAD_TO_DEG = 180 / np.pi

#-------------------------------------------------------------------

timeStep = 500
simTime = 60

rocket = DOF3()
motor = rocketMotor(timeStep)

motor.maxIgnitionDelay = 0.75

motor.add_motor(motorType.d12, "ascent")

motor.add_motor(motorType.d12, 'secondary_ascent')

ori = orientation()

rocket.mmoi =  0.0404203354
rocket.drymass = 0.59
rocket.mass = rocket.drymass + motor.totalMotorMass
rocket.ori = 0
rocket.leverArm = 0.44
rocket.cpLocation = -0.3

apogee = False

PIDSpeed = 20 # HZ
dataLoggingSpeed = 40 #HZ

setpoint = 0.0

PIDDelay = 1 / PIDSpeed
logDelay = 1 / dataLoggingSpeed

PID_ori = PID(2.2, 0.6, 0.3, 0, 5, False)

#-------------------------------------------------------------------

logger = dataLogger()

logger.addDataPoint("time")

logger.addDataPoint("ori")
logger.addDataPoint("ori_sensed")
logger.addDataPoint("ori_rate")
logger.addDataPoint("setpoint")
logger.addDataPoint("actuator_output")

logger.addDataPoint("X_position")
logger.addDataPoint("Y_position")

logger.addDataPoint("X_velocity")
logger.addDataPoint("Y_velocity")
logger.addDataPoint("resultant_velocity")

logger.addDataPoint("X_acceleration")
logger.addDataPoint("Y_acceleration")
logger.addDataPoint("resultant_acceleration")

logger.addDataPoint("thrust")
logger.addDataPoint("mass")

logger.fileName = "data_out.csv"


oriPlot = dataVisualiser()
oriPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','Y_position','X_velocity','Y_velocity','resultant_velocity','X_acceleration','Y_acceleration','resultant_acceleration','thrust','mass']

posPlot = dataVisualiser()
posPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','Y_position','X_velocity','Y_velocity','resultant_velocity','X_acceleration','Y_acceleration','resultant_acceleration','thrust','mass']

accelPlot = dataVisualiser()
accelPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','Y_position','X_velocity','Y_velocity','resultant_velocity','X_acceleration','Y_acceleration','resultant_acceleration','thrust','mass']

time = 0.0
counter = 0
lastPID = 0.0

dt = 1 / timeStep

ori.trueOri = rocket.ori
ori.oriNoiseMultiplier = 0.5

TVC = actuator()

TVC.maxActuatorPosition = 20
TVC.actuatorSpeed = 50
TVC.actuatorNoise = 0.1

randWind = random.randint(75, 150) / 100 * random.choice([-1,1])

logger.initCSV(True, True)

motor.ignite("ascent", time)

#-------------------------------------------------------------------

while time < simTime:
    counter += 1
    time += dt

    # windForce = (np.sin(time * 2) + randWind) * 0.5
    if time > 0.5 and time < 1.5:
        setpoint += 5 * dt
        PID_ori.setSetpoint(setpoint)
    motor.update(time)
    rocket.mass = rocket.drymass + motor.totalMotorMass
    if rocket.posY > 5 and motor.currentThrust < 0.5:
        motor.ignite('secondary_ascent', time)
    # rocket.addForce(windForce * dt, 90 * DEG_TO_RAD)
    rocket.addForce(motor.currentThrust, TVC.currentActuatorPosition * DEG_TO_RAD / 4, rocket.leverArm)
    if rocket.vel != 0:
        rocket.addForce(0.65 * 0.01 * 1.225 * (rocket.vel * rocket.vel), np.arcsin(rocket.velY / rocket.vel), rocket.cpLocation)


    if time > lastPID + PIDDelay:
        
        lastPID = time

        ori.update(rocket.ori, PIDDelay)
        TVC.actuate(time, PID_ori.compute(ori.sensedOri * RAD_TO_DEG, PIDDelay))

        logger.recordVariable("time", time)
        logger.recordVariable("ori", rocket.ori * RAD_TO_DEG)
        logger.recordVariable("ori_sensed", ori.sensedOri * RAD_TO_DEG)
        logger.recordVariable("ori_rate", rocket.oriRate * RAD_TO_DEG)
        logger.recordVariable("setpoint", PID_ori.setPoint)
        logger.recordVariable("actuator_output", TVC.currentActuatorPosition / 4)
        logger.recordVariable("X_position", rocket.posX)
        logger.recordVariable("Y_position", rocket.posY)
        logger.recordVariable("X_velocity", rocket.velX)
        logger.recordVariable("Y_velocity", rocket.velY)
        logger.recordVariable("resultant_velocity", rocket.vel)
        logger.recordVariable("X_acceleration", rocket.accelX)
        logger.recordVariable("Y_acceleration", rocket.accelY)
        logger.recordVariable("resultant_acceleration", rocket.accel)
        logger.recordVariable("thrust", motor.currentThrust)
        logger.recordVariable("mass", rocket.mass)
        logger.saveData(False)

    if rocket.velY < -1:
        apogee = True

    rocket.update(dt)

    if rocket.posY <= 0.1 and apogee == True:
        break

plot_ori = oriPlot.graph_from_csv(['time', 'actuator_output', 'ori', 'ori_sensed', 'setpoint'])
plot_pos = posPlot.graph_from_csv(['time', 'Y_position', 'Y_velocity', 'X_position', 'X_velocity', 'thrust'])
plot_accel = accelPlot.graph_from_csv([])

plt.figure(1)

for index, dataPoint in enumerate(plot_pos):
    if index > 0:   
        plt.plot(plot_pos[0], dataPoint)

plt.xlabel("time")
plt.ylabel("alt (m), velocity (m/s), thrust (n)")


plt.figure(2)

for index, dataPoint in enumerate(plot_ori):
    if index > 0:   
        plt.plot(plot_ori[0], dataPoint)

plt.xlabel("time")
plt.ylabel("ori, sensed ori, actuator angle (deg)")

plt.show()