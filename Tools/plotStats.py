# -*- coding: utf-8 -*-
"""
Created on Sun Nov 11 16:08:36 2018

@author: SimonLBS
"""
from graphviz import Digraph
import tkinter as tk
from tkinter.filedialog import askopenfilename
root = tk.Tk()
root.withdraw()
import json

file = askopenfilename(title = "JSON file")

with open(file) as f:
    data = json.load(f)

dot = Digraph(format='png')#format='svg', format='pdf',format='png'


for state in data["stats"]:
    name = state["name"]
    mean = state["mean"]
    stddev = state["stddev"]
    dot.node(name, label="{} \nmean={}, stddev={}".format(name, mean, stddev))
for state in data["stats"]:
    name = state["name"]
    for conn in state["conn"]:
        value = "{}".format(state["conn"][conn])
        dot.edge(name,conn, label=value)

dot.render("file_name", view=True)
