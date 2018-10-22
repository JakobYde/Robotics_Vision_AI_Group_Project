#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 10:39:41 2018

@author: simonlbs
"""
n = input("Writ n (nxn matrix): ")
topInputName = input("Write top input name: ")

topInputVars = []
while True:
    inputVar = input("Wirte {} input term from left to right (q to quit): ".format(topInputName))
    if inputVar == "q":
        break
    topInputVars.append(inputVar)
    
leftInputName = input("Write left input name: ")
leftInputVars = []
while True:
    inputVar = input("Wirte {} input term from top to buttom (q to quit): ".format(leftInputName))
    if inputVar == "q":
        break
    leftInputVars.append(inputVar)

outputVars = []
while True:
    outputVar = input("Output var (q to quit): ")
    if outputVar == "q":
        break
    outputVars.append(outputVar)

roules = []
for topV in topInputVars:
    for leftV in leftInputVars:
        rule = "if {} is {} and {} is {} then ".format(topInputName, topV, leftInputName, leftV)
        print("\n"+rule)
        outputGiven = False
        for i, outVar in enumerate(outputVars):
            ov = input("Write {} output (- to None): ".format(outVar))
            if ov is not "-":
                outputGiven = True
                if i is not len(outputVars)-1:
                    rule += "{} is {} and ".format(outVar, ov)
                else:
                    rule += "{} is {}".format(outVar, ov)
        if outputGiven:
            roules.append(rule+"\n")

outputFile = input("Write outputfile: ")
with open(outputFile, "w") as f:
    f.writelines(roules)