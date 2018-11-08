#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  8 12:08:25 2018

@author: simonlbs
"""

import matplotlib.pyplot as plt
import json
import tkinter as tk
from tkinter.filedialog import askopenfilename
root = tk.Tk()
root.withdraw()

file = askopenfilename(title = "JSON file")

with open(file) as f:
    data = json.load(f)


img = plt.imread("../robot_control/tmp.eps")
fig, ax = plt.subplots()
scale = 2.85
ax.imshow(img, extent=[-img.shape[1]/2/scale, img.shape[1]/2/scale, -img.shape[0]/2/scale, img.shape[0]/2/scale])

exec("ax."+data["plotType"]+"""(data["xdata"], data["ydata"])""")
plt.xlabel(data["xlabel"])
plt.ylabel(data["ylabel"])
plt.title(data["titel"])
fig.show()
