# -*- coding: utf-8 -*-
"""
Created on Sat Oct 13 12:54:55 2018

@author: SimonLBS
"""
import numpy as np
import membership
import matplotlib.pyplot as plt

class term:
    def __init__(self, name, memberfunction, args):
        self.name = name
        self.memberfunction = membership.getMember(memberfunction, args)
def sRange(start, stop, step):
    number = start
    while number < stop:
        yield number
        number += step
class Variable:
    def __init__(self, name, enabled, _range, lockRange):
        self.enabled = enabled
        self._range = _range
        self.lockRange = lockRange
        self.name = name
        self.terms = []
        
    def addTerm(self, term):
        self.terms.append(term)
        
    def plot(self, loc='upper right', pre=0.01):
        plt.figure(num=self.name)
        x = np.arange(self._range[0]-pre,self._range[1]+pre,pre)
        for term in self.terms:
            y = term.memberfunction.membership(x)
            plt.plot(x, y, label=term.name)
        plt.legend(loc=loc)
        plt.title(self.name)
        plt.show()
            
        