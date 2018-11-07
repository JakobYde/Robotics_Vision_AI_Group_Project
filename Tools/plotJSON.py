#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  6 16:12:31 2018
@author: SimonLBS
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

plt.figure()
exec("plt."+data["plotType"]+"""(data["xdata"], data["ydata"])""")
plt.xlabel(data["xlabel"])
plt.ylabel(data["ylabel"])
plt.title(data["titel"])
plt.show()
