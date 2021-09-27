import numpy as np
import random

global RAD_TO_DEG
global DEG_TO_RAD
DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

def calculateRotationalAcceleration(moment_arm, force, force_angle, MMOI):
    """Returns the rotational acceleration on one axis. USE RADIANS.""" 
    return force * (np.sin(force_angle) * moment_arm) / MMOI

def calculateVerticalAcceleration(mass, force, body_angle, actuator_angle):
    """calculates the vertical acceleration of the body based on the angle of the body and the force applied. USE RADIANS."""
    force_on_body = np.cos(actuator_angle) * force
    return ((np.cos(body_angle) * force_on_body) / mass)

def calculateLateralAcceleration(mass, force, body_angle, actuator_angle):
    """calculates the lateral acceleration of the body based on the angle of the body and the force applied. USE RADIANS."""
    force_on_body = np.cos(actuator_angle) * force
    return ((np.sin(body_angle) * force_on_body) / mass)

class DOF3:

    def __init__(self):
        
        self.mass = 0.0
        self.drymass = 0.0
        self.mmoi = 0.0
        self.leverArm = 0.0
        self.cpLocation = 0.0

        self.ori = 0.0
        self.oriRate = 0.0
        self.oriAccel = 0.0

        self.accel = 0.0
        self.accelX = 0.0
        self.accelY = 0.0

        self.vel = 0.0
        self.velX = 0.0
        self.velY = 0.0
        
        self.posX = 0.0
        self.posY = 0.0

        self.timeStep = 0


    def addForce(self, force, forceDir, leverArm):

        self.oriAccel += calculateRotationalAcceleration(leverArm, force, forceDir, self.mmoi)

        self.accelX += calculateLateralAcceleration(self.mass, force, self.ori, forceDir)
        self.accelY += calculateVerticalAcceleration(self.mass, force, self.ori, forceDir)

    def update(self, dt):
        
        self.velX += self.accelX * dt
        self.velY += (self.accelY - 9.83) * dt

        self.posX += self.velX * dt
        self.posY += self.velY * dt
        if self.posY < 0:
            self.posY = 0
            self.velY = 0

        self.oriRate += self.oriAccel * dt
        self.ori += self.oriRate * dt
        
        self.accel = np.sqrt( np.power(self.accelX, 2) + np.power(self.accelY, 2) )
        self.vel = np.sqrt( np.power(self.velX, 2) + np.power(self.velY, 2) )

        self.accelX = 0.0
        self.accelY = 0.0
        self.oriAccel = 0.0

class actuator:
    def __init__(self):
        self.actuatorSpeed = 0.0
        self.actuatorNoise = 0.0
        self.currentActuatorPosition = 0.0
        self.PIDLoop = 0.0
        self.lastActuatorTime = 0.0
        self.maxActuatorPosition = 0.0

    def actuate(self, time, command_angle):
        
        dt = time - self.lastActuatorTime
        
        if command_angle < self.currentActuatorPosition - (self.actuatorSpeed * dt):
            self.currentActuatorPosition -= self.actuatorSpeed * dt
        else:
            self.currentActuatorPosition = command_angle
        
        if command_angle > self.currentActuatorPosition + (self.actuatorSpeed * dt):
            self.currentActuatorPosition += self.actuatorSpeed * dt
        else:
            self.currentActuatorPosition = command_angle

        if self.currentActuatorPosition > abs(self.maxActuatorPosition):
            self.currentActuatorPosition = abs(self.maxActuatorPosition)

        if self.currentActuatorPosition < abs(self.maxActuatorPosition) * -1:
            self.currentActuatorPosition = abs(self.maxActuatorPosition) * -1

        self.currentActuatorPosition += (random.randint(-100, 100) / 100) * self.actuatorNoise
        self.lastActuatorTime = time