import numpy as np
import random
import matplotlib.pyplot as plot

from navMath import orientation, PID, FSF
from physicsMath import DOF3, actuator
from motors import rocketMotor, motorType
from dataManagement import dataLogger

DEG_TO_RAD = np.pi / 180 
RAD_TO_DEG = 180 / np.pi

#-------------------------------------------------------------------

timeStep = 300
simTime = 60

rocket = DOF3()
motor = rocketMotor(motorType.d12, timeStep)
ori = orientation()

rocket.mmoi =  0.0404203354
rocket.mass = 0.614
rocket.ori = 0
rocket.leverArm = 0.44

apogee = False

PIDSpeed = 50 # HZ
dataLoggingSpeed = 40 #HZ

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

logger.fileName = "data_out.csv"

time = 0.0
counter = 0
lastPID = 0.0

dt = 1 / timeStep

ori.trueOri = rocket.ori
ori.oriNoiseMultiplier = 0.01

TVC = actuator()

TVC.maxActuatorPosition = 20
TVC.actuatorSpeed = 50
TVC.actuatorNoise = 0.05

logger.initCSV(True, True)

motor.ignite(time)

#-------------------------------------------------------------------

while time < simTime:
    counter += 1
    time += dt

    motor.update(time)
    rocket.addForce(motor.currentThrust, TVC.currentActuatorPosition * DEG_TO_RAD / 4)

    # print(rocket.posY)

    if time > lastPID + PIDDelay:
        
        lastPID = time

        ori.update(rocket.ori, PIDDelay)
        TVC.actuate(time, PID_ori.compute(ori.sensedOri * RAD_TO_DEG, PIDDelay))

        logger.recordVariable("time", time)
        logger.recordVariable("ori", rocket.ori * RAD_TO_DEG)
        logger.recordVariable("ori", ori.sensedOri * RAD_TO_DEG)
        logger.recordVariable("ori_rate", rocket.oriRate * RAD_TO_DEG)
        logger.recordVariable("setpoint", 0)
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
        logger.saveData(False)

    if rocket.velY < -1:
        apogee = True

    rocket.update(dt)

    if rocket.posY <= 0.1 and apogee == True:
        break