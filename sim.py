import matplotlib.pyplot as plt
import numpy as np
import random
import pygame
from time import sleep

from navMath import orientation, PID, FSF
from physicsMath import DOF3, actuator
from motors import rocketMotor, motorType
from dataManagement import dataLogger
from dataVisualisation import dataVisualiser
from graphics import rectangle

DEG_TO_RAD = np.pi / 180 
RAD_TO_DEG = 180 / np.pi

#-------------------------------------------------------------------

timeStep = 500
simTime = 60

rocket = DOF3()
motor = rocketMotor(timeStep)

motor.maxIgnitionDelay = 0.7

motor.add_motor(motorType.e12, "ascent")

motor.add_motor(motorType.e12, "landing")

ori = orientation()

rocket.mmoi =  0.0404203354
rocket.drymass = 0.6
rocket.mass = rocket.drymass + motor.totalMotorMass
rocket.ori = 0
rocket.leverArm = 0.44
rocket.cpLocation = -0.3

apogee = False

PIDSpeed = 60 # HZ
dataLoggingSpeed = 40 #HZ

setpoint = 0.0

PIDDelay = 1 / PIDSpeed
logDelay = 1 / dataLoggingSpeed

PID_ori = PID(2.2, 0.7, 0.4, 0, 5, False)

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
logger.addDataPoint("prograde_direction")

logger.addDataPoint("X_acceleration")
logger.addDataPoint("Y_acceleration")
logger.addDataPoint("resultant_acceleration")

logger.addDataPoint("thrust")
logger.addDataPoint("mass")

logger.fileName = "data_out.csv"


oriPlot = dataVisualiser()
oriPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','Y_position','X_velocity','Y_velocity','resultant_velocity','velocity_direction','X_acceleration','Y_acceleration','resultant_acceleration','thrust','mass']

posPlot = dataVisualiser()
posPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','Y_position','X_velocity','Y_velocity','resultant_velocity','velocity_direction','X_acceleration','Y_acceleration','resultant_acceleration','thrust','mass']

accelPlot = dataVisualiser()
accelPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','Y_position','X_velocity','Y_velocity','resultant_velocity','velocity_direction','X_acceleration','Y_acceleration','resultant_acceleration','thrust','mass']

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

lastScreen = 0.0
screenDelay = 1 / 60

d90 = 90 * DEG_TO_RAD

d180 = 180 * DEG_TO_RAD

#colors for pygame

white = (255, 255, 255)
yellow = (255, 255, 102)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (50, 153, 213)

dis_width = 1280
dis_height = 720
 
# dis = pygame.display.set_mode((dis_width, dis_height))

rocketRect = rectangle(dis_width / 2, dis_height / 2, 120, 550)

rocket.oriRate = 0.01

lightAlt = 0.0

retroPeak = False
retroTime = 0.0

sinePercent = 0.0
pitchoverStart = 0.0
timeToImpact = 0.0
decelerationTime = 0.0
deltaVTotal = 21.0 # meters / second
deltaVSpike = 7.5 # over 0.5 secdons
deltaVafterRetroPeak = 7.75
sinMultiplier = 0.0
velAtRetroPeak = 0.0
apogeeAlt = 0.0
rocketTrueDV = 0.0

hopAlt = 0.0

#-------------------------------------------------------------------

while time < simTime:
    counter += 1
    time += dt

    windForce = (np.sin(time * 2) + randWind) * 0.03

    if time > 0.25 and pitchoverStart == 0.0:
        pitchoverStart = time

    if time > 0.25 and time < 1.75:
        setpoint = np.sin((time - pitchoverStart) * 4.2) * 10
        PID_ori.setSetpoint(setpoint)

    if apogee == True and lightAlt == 0.0:    
        timeToImpact = np.sqrt( 2 * rocket.posY / 9.83) + time
        lightAlt = 0.75 * rocket.posY
        print(timeToImpact)

    if apogee == True and rocket.posY < lightAlt and motor.isLit["landing"] == False:
        motor.ignite("landing", time)
        print(f'lighting at T+ {round(time, 2)}, {round(rocket.posY, 2)} Meters, and {round(rocket.vel, 2)} meters per second')

    if apogee == True and motor.currentThrust > 30 and retroPeak == False:
        retroPeak = True
        retroTime = time
        print(f'retro peak of {motor.ignitionDelays["landing"]}, at T+ {retroTime}')
        print(f'estimated delta v remaining: {motor.ignitionDelays["landing"] * 10}')
        deltaVafterRetroPeak = motor.ignitionDelays["landing"] * 10

    if sinMultiplier == 0.0 and apogee == True:
        PID_ori.setSetpoint(rocket.retrograde)

    if time > retroTime + 0.5 and time < retroTime + 1.2 and retroTime > 0 and sinMultiplier == 0.0:
        print(f'velocity ta sine wave divert calculation: {rocket.vel}')
        velAtRetroPeak = rocket.vel
        print((rocket.vel / deltaVafterRetroPeak))
        sinMultiplier = (((rocket.vel / deltaVafterRetroPeak) - 1) * 100)# / 0.63490941538)
        print(f'sin wave divert generated - % bleedoff required: {sinMultiplier}')
        if sinMultiplier < 1:
            print("aborting divert - coming in lower than expected")
            sinMultiplier = 0.001

    if time > retroTime + 0.5 and time < retroTime + 1.5 and retroTime > 0 and sinMultiplier != 0.0:
        setpoint = np.sin((time - retroTime + 0.5) * 6.3) * sinMultiplier
        PID_ori.setSetpoint(setpoint)

    if apogee == True and motor.currentThrust < 2 and motor.currentThrust > 0.1:
        rocketTrueDV = rocket.vel - velAtRetroPeak

    motor.update(time)
    rocket.mass = rocket.drymass + motor.totalMotorMass
    # if rocket.posY > 5 and motor.currentThrust < 6:
    #     motor.ignite('secondary_ascent', time)
    # rocket.addForce(windForce, d90 -rocket.ori, rocket.cpLocation)
    rocket.addForce(motor.currentThrust, TVC.currentActuatorPosition * DEG_TO_RAD / 4, rocket.leverArm)

    if rocket.vel != 0:
        rocket.addForce(0.65 * 0.004 * 1.225 * (rocket.vel * rocket.vel), rocket.retrograde + rocket.ori, rocket.cpLocation)
        # print(F'PGRAD: {round(np.arctan( rocket.velY / rocket.velX) * RAD_TO_DEG, 2)}, RGRAD: {round(np.arctan(rocket.velX / rocket.velY) * RAD_TO_DEG, 2)}')
        # print(f'PGRAD: {round(rocket.prograde * RsAD_TO_DEG, 2)}, RGRAD: {round((rocket.retrograde)  * RAD_TO_DEG, 2)}')
        # print("\n")

    if time > lastPID + PIDDelay:
        
        lastPID = time
        rocketRect.rotate(rocket.oriRate * PIDDelay)
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
        logger.recordVariable("prograde_direction", rocket.prograde * RAD_TO_DEG)
        logger.recordVariable("X_acceleration", rocket.accelX)
        logger.recordVariable("Y_acceleration", rocket.accelY)
        logger.recordVariable("resultant_acceleration", rocket.accel)
        logger.recordVariable("thrust", motor.currentThrust)
        logger.recordVariable("mass", rocket.mass)
        logger.saveData(False)
    
    if rocket.velY < -1 and apogee == False:
        apogee = True
        apogeeAlt = rocket.posY

    if rocket.velY > 0 and apogee == True:
        hopAlt = rocket.posY

    rocket.update(dt)

    if rocket.posY <= 0.1 and apogee == True:
        break

print(f"""
Time for some statistics!
the flight reached an apogee of {round(apogeeAlt, 2)} meters above ground

the percentage of the sine wave divert was {round(sinMultiplier, 2)}

the estimated delta-v after retro peak was {round(deltaVafterRetroPeak, 2)} M/s
and the actual delta-v was {abs(round(rocketTrueDV, 2))} (including the sine wave divert)
but the error in velocity lost versus desired velocity lost was {round(rocketTrueDV + velAtRetroPeak, 2)}

the hop altitude was {round(hopAlt, 2)} meters above ground

the ground impact velocity was {round(rocket.vel, 2)} M/s
the lateral impact velocity was {round(rocket.velX, 2)} M/s

and the orientation at impact was {round(rocket.ori * RAD_TO_DEG, 2)} degrees""")
plot_ori = oriPlot.graph_from_csv(['time', 'actuator_output', 'ori', 'ori_sensed', 'setpoint'])
plot_pos = posPlot.graph_from_csv(['time', 'Y_position', 'Y_velocity', 'X_position', 'X_velocity', 'thrust'])

# for dataLogg in plot_ori:
#     rocketRect.rotate(rocket.oriRate * dt)
    
#     dis.fill(black)
#     pygame.draw.rect(dis, white, [(dis_width / 2) - 360, (dis_height / 2) - 350, 700, 700])
#     pygame.draw.polygon(dis, black, [rocketRect.vertices[0], rocketRect.vertices[1], rocketRect.vertices[2], rocketRect.vertices[3]])
    
#     pygame.display.update()

#     sleep(dt)

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