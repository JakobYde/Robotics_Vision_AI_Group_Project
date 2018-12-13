#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 11 12:49:47 2018

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

count = 0

alfaRunning = 0.002
alphaColor = 0.5
plt.rcParams.update({'font.size': 20})

while True:
    if "xdata_{}".format(count) in data:
        ydata = data["ydata_{}".format(count)]
        ydata = np.array(ydata)
        y = np.mean(ydata,axis=0)
        
        x = data["xdata_{}".format(count)]

        label = data["legend_{}".format(count)]
        
        p = plt.plot(x, rooling(y,alfaRunning), label=label)
        #plt.plot(x, y, color=p[0].get_color(), alpha=alphaColor/2)#, label=data["legend_{}".format(count)]
        plt.errorbar(x,y,yerr=np.std(ydata, axis=0),errorevery=10000, color=p[0].get_color()+"1F",ecolor=p[0].get_color(),capsize = 5)
        if "xlabel" in data: plt.xlabel("Episode")
        if "ylabel" in data: plt.ylabel(data["ylabel"])
        count+=1
    else:
        break

#plt.legend()
plt.grid(color='grey', linestyle='--')
plt.title(data["titel"])

plt.show()   

# Plot a line between the means of each dataset
#plt.plot(x, y, 'b-')

# Save the default tick positions, so we can reset them...
#locs, labels = plt.xticks() 
#ydata = np.transpose(ydata)
#plt.boxplot(ydata, positions=data["xdata_0"])

# Reset the xtick locations.
#plt.xticks(locs, labels)
plt.show()

#legend = []
#plotString = "("
#count = 0
#
#windo = 500
#alphaColor = 0.5
#alfaRunning = 0.002
#plt.rcParams.update({'font.size': 20})
#while True:
#    if "xdata_{}".format(count) in data:
#        if "legend_{}".format(count) in data:    
#            y_data = data["ydata_{}".format(count)]
#            y_data = np.mean(y_data, axis=0)
#            p = plt.plot(data["xdata_{}".format(count)], rooling(y_data,alfaRunning), label=data["legend_{}".format(count)])
#            #greedy = float(data["legend_{}".format(count)].split()[2])
#            #plt.plot(data["xdata_{}".format(count)], rooling(data["ydata_{}".format(count)],alfaRunning))#, label="Greedy = "+str(greedy))#, color=p[0].get_color(), alpha=alphaColor)
#            #plt.plot(data["xdata_{}".format(count)], data["ydata_{}".format(count)], color=p[0].get_color(), alpha=alphaColor/2)#, label=data["legend_{}".format(count)]
#                
#        else:
#            p = plt.plot(data["xdata_{}".format(count)][:-windo], np.convolve(data["ydata_{}".format(count)],windo)[:-windo])
#            plt.plot(data["xdata_{}".format(count)], data["ydata_{}".format(count)], color=p[0].get_color(), alpha=alphaColor)
#             
#        count += 1
#    else:
#        break
#if "xlabel" in data: plt.xlabel("Episode")
#if "ylabel" in data: plt.ylabel(data["ylabel"])
##plt.legend(bbox_to_anchor=(1,0.5), loc="best")
#
#plt.legend()
#plt.grid(color='grey', linestyle='--')
#plt.title(data["titel"])
#
#plt.show()
