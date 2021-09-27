import numpy as np
import random
import matplotlib.pyplot as plot

from navMath import orientation, PID, FSF
from physicsMath import DOF3, actuator
from motors import rocketMotor, motorType
from dataManagement import dataLogger

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

PIDSpeed = 50 # HZ
dataLoggingSpeed = 40 #HZ

PIDDelay = 1 / PIDSpeed
logDelay = 1 / dataLoggingSpeed

PID_ori = PID(2.2, 0.6, 0.3, 0, 5, False)

#-------------------------------------------------------------------

DEG_TO_RAD = 180/np.pi
RAD_TO_DEG = np.pi/180

logger = dataLogger()

logger.addDataPoint("time")

logger.addDataPoint("ori")
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
ori.oriNoiseMultiplier = 0.1

TVC = actuator()

TVC.maxActuatorPosition = 20
TVC.actuatorSpeed = 50
TVC.actuatorNoise = 0.2

logger.initCSV(True, True)

motor.ignite(time)

#-------------------------------------------------------------------

while time < simTime:
    counter += 1
    time += dt

    print(rocket.posY)

    if time > lastPID + PIDDelay:
        
        lastPID = time

        ori.update(rocket.ori, PIDDelay)
        TVC.actuate(time, PID_ori.compute(ori.sensedOri * RAD_TO_DEG, PIDDelay))

        logger.recordVariable("time", time)
        logger.recordVariable("ori", rocket.ori)
        logger.recordVariable("ori_rate", rocket.oriRate)
        logger.recordVariable("setpoint", 0)
        logger.recordVariable("actuator_output", TVC.currentActuatorPosition)
        logger.recordVariable("X_position", rocket.posX)
        logger.recordVariable("Y_position", rocket.posY)
        logger.recordVariable("X_velocity", rocket.velX)
        logger.recordVariable("Y_velocity", rocket.velY)
        logger.recordVariable("resultant_velocity", 0)
        logger.recordVariable("X_acceleration", rocket.accelX)
        logger.recordVariable("Y_acceleration", rocket.accelY)
        logger.recordVariable("resultant_acceleration", 0)
        logger.recordVariable("thrust", motor.currentThrust)
        logger.saveData(False)

    motor.update(time)
    rocket.addForce(motor.currentThrust, TVC.currentActuatorPosition * DEG_TO_RAD)
    rocket.update(dt)

    if rocket.posY < -1:
        break