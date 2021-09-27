import numpy as np
import random

class orientation:
    def __init__(self):
        self.trueOri = 0.0
        self.sensedOri = 0.0
        self.oriError = 0.0
        self.oriNoiseMultiplier = 0.0
        self.lastDT = 0.0
    
    def update(self, newOri, dt):
        self.trueOri = newOri
        self.oriError += (random.randint(60, 100) / 100 * random.choice([-1, 1])) * self.oriNoiseMultiplier * dt
        self.sensedOri = self.oriError

class PID:
    def __init__(self, kP, kI, kD, setpoint, iMax, usePONM):
        
        #assigning gains to user inputs
        self.kP = kP 
        self.kI = kI
        self.kD = kD

        #the value that the PID controller wants the input to become
        self.setPoint = setpoint

        # you might not see this in your run-of-the-mill PID library, this is just a limit on how bit the integral can get
        self.iMax = iMax 

        #you are even less likely to see this in your run-of-the-mill PID library
        #this is basically a different implementation of the proportional gain, where instead of looking at the error
        #the proportional part of the controller looks at the change between the initial sensed value and the current sensed value
        
        self.usePONM = usePONM

        if usePONM == True:
            self.proportional = 0

        #this variable stores the most recent process variable, so that when update is called the PID controller can
        #know not only the current error, but also the speed at which it is approaching or departing from the setpoint!
        #this helps with reducing overshoot by feeding it into the derivitive value, which basically functions as a break on a car
        self.lastProcess = 0

        #this is the integral, which helps combat steady state error. Eg. your rocket is stable, and has canceled all rotation on
        #it's body, but its 10 degrees off target! the integral is crucial here as it increases over time with the error
        self.integral = 0
        self.error = 0

    def setSetpoint(self, setpoint):
        self.setPoint = setpoint

    def zeroIntegral(self):
        self.integral = 0

    def compute(self, process, dt):
        
        change = process - self.lastProcess

        self.lastProcess = process

        self.error = self.setPoint - process

        if self.usePONM == True:
            #whats happening here is the proportional changing with the process variable
            self.proportional -= change * self.kP
        else:
            proportional = self.error * self.kP

        #checking that the integral will not exceed the maximum value allowed by the user
        if abs(self.integral + self.error * self.kI * dt) < self.iMax:
            self.integral += self.error * self.kI * dt

        derivitive = change * self.kD / dt

        if self.usePONM == True:
            return self.proportional + self.integral - derivitive
        else:
            return proportional + self.integral - derivitive

class FSF:
    #gains
    fsf_gains = np.matrix([0,0,0,0])

    #setpoints (ori, rotVel, position, velocity)
    fsf_setpoint = np.array([0,0,0,0])

    lastOri = 0
    lastPos = 0

    I = 0
    output = 0

    def __init__(self, _gains, _setpoint, iGain):
        self.fsf_gains = _gains
        self.fsf_setpoint = _setpoint
        self.iGain = iGain

    def compute(self, pos, ori, dt):
        x = np.matrix([
            [ori - self.fsf_setpoint[0]],
            [((ori - self.lastOri) * dt) - self.fsf_setpoint[1]],
            [pos - self.fsf_setpoint[2]],
            [((pos - self.lastPos) * dt) - self.fsf_setpoint[3]]
        ])
        out = -self.fsf_gains * x
        self.output = np.sum(out)
        self.I += self.output * self.iGain
        self.lastOri = ori
        self.lastPos = pos
        self.output += self.I
    
    def setSetpoint(self, ori, oriRate, pos, vel):
        self.fsf_gains = np.array([ori, oriRate, pos, vel])
    
    def reset(self):
        self.output = 0
        self.lastPos = 0
        self.lastOri = 0
        self.I = 0