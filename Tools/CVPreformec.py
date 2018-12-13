#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 13 10:25:15 2018

@author: simonlbs
"""
from tabulate import tabulate
from operator import itemgetter
import sys
import numpy as np
import matplotlib.pyplot as plt
import json
import tkinter as tk
from tkinter.filedialog import askopenfilename
root = tk.Tk()
root.withdraw()

file = askopenfilename(title = "JSON file with pos")
if len(file) == 0:
    sys.exit()
    
with open(file) as f:
    data = json.load(f)
    
fileF = file+"_falsepos"
    
with open(fileF) as f:
    data2 = json.load(f)

 
maxDistenList = []
ch = "-1"
while ch != "q":
    ch = input("Max input (q for quit): ")
    if(ch != "q"):
        maxDistenList.append(float(ch))
    print(maxDistenList)

print("Is ", maxDistenList, "right?")
ch = input("T/F: ")
if ch != "T":
    exit()

for maxDisten in maxDistenList:       
    distList = []
    count = 0
    while True:
        if "xdata_{}".format(count) in data:
            
            ydata = list(reversed(data["ydata_{}".format(count)]))
            ydataF = list(reversed(data2["ydata_{}".format(count)]))
            xdata = list(reversed(data["xdata_{}".format(count)]))
            label = data["legend_{}".format(count)]
            
            yU = []
            yUF = []
            for i in range(len(xdata)):
                if(xdata[i] >= maxDisten):
                    break
                yU.append(ydata[i])
                yUF.append(ydataF[i])
                
            yMean = float(np.mean(yU))
            yStd = float(np.std(yU))
            yFMean = float(np.mean(yUF))
            yFStd = float(np.std(yUF))
            distList.append([label,yMean,yStd,yFMean,yFStd])
            
            count+=1
        else:
            break
    listOfBest = list(reversed(sorted(distList, key=itemgetter(1))))
    header = ['Parameter [0 m, {} m]'.format(maxDisten), 'Mean detected% ', 'std', 'Mean F detected% ', 'std F']
    print(tabulate(listOfBest, headers=header, tablefmt="grid"))
    
    
    import csv
    with open('{}_{}.csv'.format(file,maxDisten), 'w', newline='') as f:
        writer = csv.writer(f)
        
        writer.writerows(listOfBest)
