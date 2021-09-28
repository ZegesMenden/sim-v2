import pygame
import numpy as np

class rectangle():
    def __init__(self, center_x, center_y, width, height):
        self.cx = center_x
        self.cy = center_y

        self.width = width
        self.height = height

        self.half_width = width / 2
        self.half_height = height / 2

        self.hypotenuse = np.sqrt((self.half_height * self.half_height) + (self.half_width * self.half_width))

        self.standard_angle = np.arcsin(self.half_width / self.hypotenuse)

        self.staticvertices = [[self.cx - self.half_width, self.cy + self.half_height], [self.cx + self.half_width, self.cy + self.half_height], [self.cx + self.half_width, self.cy - self.half_height], [self.cx - self.half_width, self.cy - self.half_height]]
        self.vertices = [[0,0],[0,0],[0,0],[0,0]]
        self.newx = self.cx
        self.newy = self.cy
        self.changex = 0
        self.changey = 0

    def rotate(self, angle): # angle in radians

        count = 0
        for v in self.staticvertices:

            corner = v
            corner[0] -= self.cx
            corner[1] -= self.cy
            self.staticvertices[count] = [ corner[0]*np.cos(angle)-corner[1]*np.sin(angle) + self.cx, corner[0]*np.sin(angle)+corner[1]*np.cos(angle) + self.cy] 
            self.vertices[count][0] = self.staticvertices[count][0] - self.changex
            self.vertices[count][1] = self.staticvertices[count][1] - self.changey
            
            count += 1

    def move(self, newx, newy):
        self.newx = newx
        self.newy = newy

        self.changex = self.newx - self.cx
        self.changey = self.newy - self.cy
