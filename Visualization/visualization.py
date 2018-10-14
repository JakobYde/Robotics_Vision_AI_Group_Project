# -*- coding: utf-8 -*-
"""
Created on Sat Oct 13 09:58:53 2018

@author: SimonLBS
"""
import variable
import re
import sys

def askForFile(titel):
    import tkinter as tk
    from tkinter.filedialog import askopenfilename
    root = tk.Tk()
    root.withdraw()
    
    return askopenfilename(title = titel)

filepath = askForFile("Fuzzy file") # filepath = input("Fuzzylite file: ")#filepath = "fuzzybugcontroller2.fll"

if filepath == '':
    sys.exit(1)
    
with open(filepath) as f:
    file = f.readlines()
file = [re.sub(' +', ' ',x.strip()).replace("\t", "").split(" ") for x in file]

lines = []
for line in file:
    g = []
    if line[0] != '#':
        for group in line:
            if group != '' and group[0] == '#':
                break
            if group != '':
                g.append(group)
        if len(g):
            lines.append(g)

varibles = []
var = {}
for line in lines:
    if line[0] == 'InputVariable:' or line[0] == 'OutputVariable:':
        if len(var):
            varibles.append(var)
            var = {}
        var['varType'] = line[0]
        var['varName'] = line[1]
        var['terms'] = []
    
    if line[0] == 'enabled:':
        var['enabled'] = line[1] == 'true'
    if line[0] == 'range:':
        var['range'] = [ float(x) for x in line[1:]]
    if line[0] == 'lock-range:':
        var['lock-range'] = line[1] == 'true'
    if line[0] == 'term:':
        term = [line[1], line[2], [ float(x) for x in line[3:]]]
        var['terms'].append(term) 
    if line[0] == 'RuleBlock:':
        if len(var):
            varibles.append(var)
            
fuzzyVaribles = []
for var in varibles:
    fVar = variable.Variable(var['varName'], var['enabled'], var['range'], var['lock-range'])
    for term in var['terms']:
        _term = variable.term(term[0],term[1], tuple(term[2]))
        fVar.addTerm(_term)
    fuzzyVaribles.append(fVar)

for fV in fuzzyVaribles:
    fV.plot(pre=0.0001)
