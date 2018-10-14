# -*- coding: utf-8 -*-
"""
Created on Sat Oct 13 09:51:25 2018

@author: SimonLBS
"""
import os
import sys
import numpy as np
import math

def getMember(member, args):
    if member == "Triangle":
        return Triangle(*args)
    if member == "Bell":
        return Bell(*args)
    if member == "BellPiShape":
        return BellPiShape(*args)
    if member == "Trapezoid":
        return Trapezoid(*args)
    if member == "Cosine":
        return Cosine(*args)
    if member == "SigmoidDifference":
        return SigmoidDifference(*args)
    if member == "Rectangle":
        return Rectangle(*args)
    if member == "Gaussian":
        return Gaussian(*args)
    if member == "SigmoidProduct":
        return SigmoidProduct(*args)
    if member == "GaussianProduct":
        return GaussianProduct(*args)
    if member == "Spike":
        return Spike(*args)
    if member == "Binary":
        return Binary(*args)
    if member == "Ramp":
        return Ramp(*args)
    if member == "SShape":
        return SShape(*args)
    if member == "Concave":
        return Concave(*args)
    if member == "Sigmoid":
        return Sigmoid(*args)
    if member == "ZShape":
        return ZShape(*args)

class _member:
    def _membership(self, x):
        return x
    def membership(self, x):
        if isinstance(x, (list,)) or type(x) is np.ndarray:
            yList = [ self._membership(xVar) for xVar in x ]
            if type(x) is np.ndarray:
                return np.array(yList)
            else:
                return yList
        else:
            return self._membership(x)
            
class Triangle(_member):
    def __init__(self, start, center, end, hight=1, name = ""):
        self.start = start
        self.center = center
        self.end = end
        self.hight=hight
        self.name = name
    
    def _membership(self, x):
        if (x<self.start or x>self.end):
            return 0*self.hight
        elif (x==self.center):
            return 1*self.hight
        elif (x<self.center):
            return self.hight*((x-self.start)/(self.center-self.start))
        else:
            return self.hight*((self.end-x)/(self.end-self.center))
        
class Bell(_member):
    def __init__(self, start, center, width, slope, hight=1, name = ""):
        self.start = start
        self.center = center
        self.width = width
        self.slope = slope
        self.center = center
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        return self.hight/(1+(abs(x-self.center)/self.width)**(2*self.slope))

class BellPiShape(_member):
    def __init__(self, bottomLeft, topLeft, topRight, bottomRight, hight=1, name = ""):
        self.bottomLeft = bottomLeft
        self.topLeft = topLeft
        self.topRight = topRight
        self.bottomRight = bottomRight
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if x <= self.bottomLeft:
            return 0*self.hight
        elif x <= 0.5*(self.topLeft+self.bottomLeft):
            return 2*self.hight*((x-self.bottomLeft)/(self.topLeft-self.bottomLeft))**2
        elif x<self.topLeft:
            return self.hight*(1-2*((x-self.topLeft)/(self.topLeft-self.bottomLeft))**2)
        elif x<=self.topRight:
            return self.hight 
        elif x <= 0.5*(self.topRight+self.bottomRight):
            return self.hight*(1-2*((x-self.topRight)/(self.topRight-self.bottomRight))**2)
        elif x < self.bottomRight:
            return 2*self.hight*((x-self.bottomRight)/(self.bottomRight-self.topRight))**2
        else:
            return 0*self.hight 

class Trapezoid(_member):
    def __init__(self, firstVertex, secondVertex, thirdVertex, fourthVertex, hight=1, name = ""):
        self.firstVertex = firstVertex
        self.secondVertex = secondVertex
        self.thirdVertex = thirdVertex
        self.fourthVertex = fourthVertex
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if x<self.firstVertex or x>self.fourthVertex:
            return 0*self.hight
        elif x < self.secondVertex:
            return self.hight*min(1,(x-self.firstVertex)/(self.secondVertex-self.firstVertex))
        elif x <= self.thirdVertex:
            return 1*self.hight
        elif x < self.fourthVertex:
            return self.hight*(self.fourthVertex-x)/(self.fourthVertex-self.thirdVertex)
        else:
            return 0*self.hight
        
class Cosine(_member):
    def __init__(self, center, width, hight=1, name = ""):
        self.center = center
        self.width = width
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if x < (self.center-0.5*self.width) or x > (self.center+0.5*self.width):
            return 0*self.hight
        else:
            return 0.5*self.hight*(1+math.cos(2/self.width*math.pi*(x-self.center)))
        
        
class SigmoidDifference(_member):
    def __init__(self, left, rising, falling, right, hight=1, name = ""):
        self.left = left
        self.rising = rising
        self.falling = falling
        self.right = right
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        a = 1/(1+ math.exp(-self.rising*(x-self.left)))
        b = 1/(1+ math.exp(-self.falling*(x-self.right)))
        return self.hight*(a-b)
        
class Rectangle(_member):
    def __init__(self, start, end, hight=1, name = ""):
        self.start = start
        self.end = end
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if x>= self.start and x<= self.end:
            return 1*self.hight
        else:
            return 0*self.hight

class Gaussian(_member):
    def __init__(self, mean, standardDeviation, hight=1, name = ""):
        self.mean = mean
        self.standardDeviation = standardDeviation
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        return self.hight*math.exp(-(x-self.mean)**2/(2*self.standardDeviation**2))
    
class SigmoidProduct(_member):
    def __init__(self, left, rising, falling, right, hight=1, name = ""):
        self.left = left
        self.rising = rising
        self.falling = falling
        self.right = right
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        a = 1/(1+ math.exp(-self.rising*(x-self.left)))
        b = 1/(1+ math.exp(-self.falling*(x-self.right)))
        return self.hight*(a*b)
    
class GaussianProduct(_member):
    def __init__(self, meanA, standardDeviationA, meanB, standardDeviationB, hight=1, name = ""):
        self.meanA = meanA
        self.standardDeviationA = standardDeviationA
        self.meanB = meanB
        self.standardDeviationB = standardDeviationB
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        i = 1 if x <= self.meanA else 0
        j = 1 if x >= self.meanB else 0  
        return self.hight*((1-i)+i*math.exp(-(x-self.meanA)**2/(2*self.standardDeviationA**2)))*((1-j)+j*math.exp(-(x-self.meanB)**2/(2*self.standardDeviationB**2)))

class Spike(_member):
    def __init__(self, center, widht, hight=1, name = ""):
        self.center = center
        self.widht = widht
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        return self.hight*math.exp(-abs(10/self.widht*(x-self.center)))
    
class Binary(_member):
    def __init__(self, start, direction , hight=1, name = ""):
        self.start = start
        self.direction  = direction 
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if self.direction > 0:
            if self.start < x:
                return 1*self.hight
        else:
            if self.start > x:
                return 1*self.hight
        return 0*self.hight
    
class Ramp(_member):
    def __init__(self, start, end , hight=1, name = ""):
        self.start = start
        self.end = end 
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if x==self.end:
            return 1*self.hight
        elif self.start < self.end:
            if x <= self.start:
                return 0*self.hight
            elif x >= self.end:
                return 1*self.hight
            else:
                return self.hight*(x-self.start)/(self.end-self.start)
        elif self.start > self.end:
            if x >= self.start:
                return 0*self.hight
            elif x <= self.end:
                return 1*self.hight
            else:
                return self.hight*(self.start-x)/(self.start-self.end)
class SShape(_member):
    def __init__(self, start, end , hight=1, name = ""):
        self.start = start
        self.end = end 
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if x <= self.start:
            return 0*self.hight
        elif x <= 0.5*(self.start+self.end):
            return self.hight*(2*((x-self.start)/(self.end-self.start))**2)
        elif x < self.end:
            return self.hight*(1-2*((x-self.end)/(self.end-self.start))**2)
        else:
            return 1*self.hight
        
class Concave(_member):
    def __init__(self, inflection , end , hight=1, name = ""):
        self.inflection = inflection 
        self.end = end 
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if self.inflection <= self.end and x < self.end:
            return self.hight * (self.end-self.inflection)/(2*self.end-self.inflection-x)
        elif self.inflection > self.end and x > self.end:
            return self.hight * (self.inflection-self.end)/(-2*self.end+self.inflection+x)
        else:
            return 1*self.hight
        
class Sigmoid(_member):
    def __init__(self, inflection, slope, hight=1, name = ""):
        self.inflection = inflection 
        self.slope  = slope  
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        return self.hight/(1+math.exp(-self.slope*(x-self.inflection))) 
    
class ZShape(_member):
    def __init__(self, start, end, hight=1, name = ""):
        self.start = start 
        self.end  = end  
        self.hight = hight
        self.name = name
        
    def _membership(self, x):
        if x <= self.start:
            return 1*self.hight
        elif x<=0.5*(self.start+self.end):
            return self.hight*(1-2*((x-self.start)/(self.end-self.start))**2)
        elif x<self.end:
            return self.hight*(2*((x-self.end)/(self.end-self.start))**2)
        else:
            return 0*self.hight