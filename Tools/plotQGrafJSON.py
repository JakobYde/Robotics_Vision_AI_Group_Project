#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  3 11:48:24 2018

@author: simonlbs
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import json
import tkinter as tk
from tkinter.filedialog import askopenfilename
root = tk.Tk()
root.withdraw()

def movingAvg(x,w):
    return np.convolve(x, np.ones((w,))/w)[(w-1):]

def rooling(l, alfa):
    tl = [l[0]]
    for v in l[1:]:
        tl.append(alfa*v + (1-alfa)*tl[-1])
    return tl

file = askopenfilename(title = "JSON file")
if len(file) == 0:
    sys.exit()
    
with open(file) as f:
    data = json.load(f)

legend = []
plotString = "("
count = 0

windo = 500
alphaColor = 0.5
alfaRunning = 0.002
while True:
    if "xdata_{}".format(count) in data:
        if "legend_{}".format(count) in data:            
            #p = plt.plot(data["xdata_{}".format(count)][:-windo], movingAvg(data["ydata_{}".format(count)],windo)[:-windo], label=data["legend_{}".format(count)]+" - moving avg ")
            plt.plot(data["xdata_{}".format(count)], rooling(data["ydata_{}".format(count)],alfaRunning), label=data["legend_{}".format(count)]+" - EMA (alfa = {})".format(alfaRunning))#, color=p[0].get_color(), alpha=alphaColor)
            #plt.plot(data["xdata_{}".format(count)], data["ydata_{}".format(count)], label=data["legend_{}".format(count)], color=p[0].get_color(), alpha=alphaColor/2)
                
        else:
            p = plt.plot(data["xdata_{}".format(count)][:-windo], np.convolve(data["ydata_{}".format(count)],windo)[:-windo])
            plt.plot(data["xdata_{}".format(count)], data["ydata_{}".format(count)], color=p[0].get_color(), alpha=alphaColor)
             
        count += 1
    else:
        break
if "xlabel" in data: plt.xlabel(data["xlabel"])
if "ylabel" in data: plt.ylabel(data["ylabel"])
plt.legend(bbox_to_anchor=(1,0.5), loc="center left")
#plt.legend()
plt.title(data["titel"])
plt.show()


