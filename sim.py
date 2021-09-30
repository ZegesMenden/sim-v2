import matplotlib.pyplot as plt
import numpy as np
import random
import pygame
import time as t

from navMath import orientation, position, PID, FSF
from physicsMath import DOF3, actuator
from motors import rocketMotor, motorType
from dataManagement import dataLogger
from dataVisualisation import dataVisualiser
from graphics import rectangle

DEG_TO_RAD = np.pi / 180 
RAD_TO_DEG = 180 / np.pi

#-------------------------------------------------------------------

timeStep = 300
simTime = 60

rocket = DOF3()
motor = rocketMotor(timeStep)

motor.maxIgnitionDelay = 0.7

motor.add_motor(motorType.e12, "ascent")

motor.add_motor(motorType.e12, "landing")

ori = orientation()
pos = position()

pos.noiseMultiplier = 0.3 # +- 0.1 m/s?

rocket.mmoi =  0.0404203354
rocket.drymass = 0.6
rocket.mass = rocket.drymass + motor.totalMotorMass
rocket.ori = 0
rocket.leverArm = 0.44
rocket.cpLocation = -0.35

apogee = False

PIDSpeed = 30 # HZ
dataLoggingSpeed = 100 #HZ

setpoint = 0.0

PIDDelay = 1 / PIDSpeed
logDelay = 1 / dataLoggingSpeed

PID_ori = PID(3, 0.8, 0.45, 0, 3, False)


#-------------------------------------------------------------------

logger = dataLogger()

logger.addDataPoint("time")

logger.addDataPoint("ori")
logger.addDataPoint("ori_sensed")
logger.addDataPoint("ori_rate")
logger.addDataPoint("setpoint")
logger.addDataPoint("actuator_output")

logger.addDataPoint("X_position")
logger.addDataPoint("X_position_estimate")
logger.addDataPoint("Y_position")
logger.addDataPoint("Y_position_estimate")

logger.addDataPoint("X_velocity")
logger.addDataPoint("X_velocity_estimate")
logger.addDataPoint("Y_velocity")
logger.addDataPoint("Y_velocity_estimate")
logger.addDataPoint("resultant_velocity")
logger.addDataPoint("resultant_velocity_estimate")
logger.addDataPoint("prograde_direction")
logger.addDataPoint("prograde_direction_estimate")


logger.addDataPoint("X_acceleration")
logger.addDataPoint("X_acceleration_estimate")
logger.addDataPoint("Y_acceleration")
logger.addDataPoint("Y_acceleration_estimate")
logger.addDataPoint("resultant_acceleration")
logger.addDataPoint("resultant_acceleration_estimate")

logger.addDataPoint("thrust")
logger.addDataPoint("mass")

logger.fileName = "data_out.csv"


oriPlot = dataVisualiser()
oriPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','X_position_estimate','Y_position','Y_position_estimate','X_velocity','X_velocity_estimate','Y_velocity','Y_velocity_estimate','resultant_velocity','resultant_velocity_estimate','velocity_direction','velocity_direction_estimate','X_acceleration','X_acceleration_estimate','Y_acceleration','Y_acceleration_estimate','resultant_acceleration','resultant_acceleration_estimate','thrust','mass']

posPlot = dataVisualiser()
posPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','X_position_estimate','Y_position','Y_position_estimate','X_velocity','X_velocity_estimate','Y_velocity','Y_velocity_estimate','resultant_velocity','resultant_velocity_estimate','velocity_direction','velocity_direction_estimate','X_acceleration','X_acceleration_estimate','Y_acceleration','Y_acceleration_estimate','resultant_acceleration','resultant_acceleration_estimate','thrust','mass']

accelPlot = dataVisualiser()
accelPlot.allDataDescriptions = ['time','ori','ori_sensed','ori_rate','setpoint','actuator_output','X_position','X_position_estimate','Y_position','Y_position_estimate','X_velocity','X_velocity_estimate','Y_velocity','Y_velocity_estimate','resultant_velocity','resultant_velocity_estimate','velocity_direction','velocity_direction_estimate','X_acceleration','X_acceleration_estimate','Y_acceleration','Y_acceleration_estimate','resultant_acceleration','resultant_acceleration_estimate','thrust','mass']

time = 0.0
counter = 0
lastPID = 0.0

dt = 1 / timeStep

ori.trueOri = rocket.ori
ori.oriNoiseMultiplier = 1

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

dis_width = 1080
dis_height = 1080
 
dis = pygame.display.set_mode((dis_width, dis_height))

rocketRect = rectangle(dis_width / 2, dis_height / 2, 8, 50)

engine = rectangle(rocketRect.cx, rocketRect.cy, 4, 20)

rocket.oriRate = 0

lightAlt = 0.0

retroPeak = False
retroTime = 0.0

sinePercent = 0.0

velAtRetroPeak = 0.0
hopalt = 0.0

area = 0.006

lastPos = 0.0
posDelay = 1 / 500

t_start = t.time()

Timee = t.time()


#-------------------------------------------------------------------

while time < simTime:
    Timee = t.time() - t_start
    counter += 1
    time += dt

    windForce = (np.sin(time * 2) + randWind) 
    rocket.ori += windForce * DEG_TO_RAD * dt

    if time > 0.25 and time < 0.75:
        setpoint = 8
        PID_ori.setSetpoint(setpoint)
    
    if time > 0.75 and time < 1.25:
        setpoint = 0
        PID_ori.setSetpoint(setpoint)
    
    if time > 1.25 and time < 1.5:
        PID_ori.setSetpoint(-8)

    if apogee == True and lightAlt == 0.0:    

        lightAlt = pos.estimatedPosY * 0.869

    if apogee == True and pos.estimatedPosY < lightAlt and sinePercent == 0.0:
        motor.ignite("landing", time)
        PID_ori.setSetpoint(pos.estimatedRetrograde * RAD_TO_DEG)

    if apogee == True and motor.currentThrust > 5 and retroPeak == False:
        retroTime = time
        velAtRetroPeak = rocket.vel
        print(f'retro peak @ {retroTime}')
        sinePercent = (1 - (motor.ignitionDelays["landing"] * 7 / rocket.vel)) / 0.07
        print(f'sine percent: {sinePercent}')

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



    # if time > retroTime and time < retroTime + 0.5 and retroTime > 0:
    #     setpoint = np.sin((time - retroTime) * 6.3) * sinePercent
    #     PID_ori.setSetpoint(setpoint)
    
    # if time > retroTime + 0.5 and time < retroTime + 1 and retroTime > 0:
    #     if rocket.retrograde * RAD_TO_DEG < 10:
    #         PID_ori.setSetpoint((rocket.retrograde * RAD_TO_DEG))
    #     else:
    #         PID_ori.setSetpoint(10)
    #     if rocket.retrograde * RAD_TO_DEG > -10:
    #         PID_ori.setSetpoint((rocket.retrograde * RAD_TO_DEG))
    #     else:
    #         PID_ori.setSetpoint(-10)
        
    if time > retroTime and time < retroTime + 1.1 and retroTime > 0:
        PID_ori.setSetpoint((pos.estimatedRetrograde)* RAD_TO_DEG )

    if time > retroTime + 1.1 and retroTime > 0:
        PID_ori.setSetpoint(0)


    motor.update(time)
    rocket.mass = rocket.drymass + motor.totalMotorMass
    # if rocket.posY > 5 and motor.currentThrust < 6:
    #     motor.ignite('secondary_ascent', time)
    # rocket.addForce(windForce, d90 -rocket.ori, rocket.cpLocation)
    rocket.addForce(motor.currentThrust, TVC.currentActuatorPosition * DEG_TO_RAD / 4, rocket.leverArm)
 

    if rocket.vel != 0:
        rocket.addForce(0.65 * area * 1.225 * (rocket.vel * rocket.vel), rocket.retrograde + rocket.ori, rocket.cpLocation)

        # print(F'PGRAD: {round(np.arctan( rocket.velY / rocket.velX) * RAD_TO_DEG, 2)}, RGRAD: {round(np.arctan(rocket.velX / rocket.velY) * RAD_TO_DEG, 2)}')
        # print(f'PGRAD: {round(rocket.prograde * RsAD_TO_DEG, 2)}, RGRAD: {round((rocket.retrograde)  * RAD_TO_DEG, 2)}')
        # print("\n")

    

    if time > lastPID + PIDDelay:
        
        pos.estimatedPosY = (pos.estimatedPosY * 50 + rocket.posY) / 51        
        lastPID = time

        ori.update(rocket.ori, PIDDelay)
        if motor.currentThrust > 0.5:
            TVC.actuate(time, PID_ori.compute(ori.sensedOri * RAD_TO_DEG, PIDDelay))
        else:
            TVC.actuate(time, 0)

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
        logger.recordVariable("X_position_estimate", pos.estimatedPosX)
        logger.recordVariable("Y_position_estimate", pos.estimatedPosY)
        logger.recordVariable("X_velocity_estimate", pos.estimatedVelX)
        logger.recordVariable("Y_velocity_estimate", pos.estimatedVelY)
        logger.recordVariable("resultant_velocity_estimate", pos.estimatedVel)
        logger.recordVariable("prograde_direction_estimate", pos.estimatedPrograde * RAD_TO_DEG)
        logger.recordVariable("X_acceleration_estimate", pos.estimatedAccelX)
        logger.recordVariable("Y_acceleration_estimate", pos.estimatedAccelY)
        logger.saveData(False)


    # if t.time() > lastScreen + screenDelay:



        # lastScreen = time
    dis.fill(black)
    
    rocketRect.rotate(rocket.oriRate * dt)
    
    rocketRect.move((dis_width / 2) - (rocket.posX * 50), (dis_height - 940) + (rocket.posY * 50))
    
    engine.rotate((TVC.currentActuatorPosition - TVC.lastActuatorPosition + rocket.oriRate) * screenDelay)

    engine.move(dis_width - (rocketRect.vertices[1][0] + rocketRect.vertices[0][0]) / 2, dis_height - (rocketRect.vertices[1][1] + rocketRect.vertices[0][1]) / 2)
    pygame.draw.rect(dis, white, [0, 0, 1080, 1080])
    
    if motor.currentThrust > 0.1:
        pygame.draw.polygon(dis, yellow, [engine.vertices[0], engine.vertices[1], engine.vertices[2], engine.vertices[3]])

    pygame.draw.polygon(dis, black, [rocketRect.vertices[0], rocketRect.vertices[1], rocketRect.vertices[2], rocketRect.vertices[3]])
    
    pygame.display.update()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print("sad")

    if rocket.velY < -1:

        apogee = True
        apogeeAlt = rocket.posY

    if rocket.velY > 0 and apogee == True:
        hopAlt = rocket.posY

    pos.update(rocket.accelX, rocket.accelY, time)

    rocket.update(dt)

    if apogee == True and rocket.velY > 0:
        hopalt = rocket.posY

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
plot_pos = posPlot.graph_from_csv(['time', 'Y_position', 'Y_position_estimate', 'Y_velocity', 'Y_velocity_estimate', 'X_position', 'X_position_estimate', 'X_velocity', 'X_velocity_estimate', 'thrust'])

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

plt.figure(3)

for index, dataPoint in enumerate(plot_pos):
    if index > 0:   
        plt.plot(plot_pos[2], plot_pos[4])

plt.xlabel("x position")
plt.ylabel("y position")

plt.show()