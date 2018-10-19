# -*- coding: utf-8 -*-
"""
Created on Fri Oct 19 13:17:57 2018

@author: SimonLBS
"""

import importlib
cv2_chek = importlib.util.find_spec('cv2')
cv2_found = cv2_chek is not None
if not cv2_found:
    print("Install cv2 before use")
tkinter_chek = importlib.util.find_spec('tkinter')
tkinter_found = tkinter_chek is not None
if not tkinter_found:
    print("Install tkinter before use")
skimage_chek = importlib.util.find_spec('skimage')
skimage_found = skimage_chek is not None
if not skimage_found:
    print("Install skimage before use")

if not (cv2_found and tkinter_found and skimage_found):
    exit(1)
else:
    print("cv2, tkinter and skimage is found")
    
import os
import sys
import cv2
import SThreader  

def askForDir(titel):
    import tkinter as tk
    from tkinter.filedialog import  askdirectory
    
    root = tk.Tk()
    root.withdraw()
    return askdirectory(title = titel) + "/"
 
def askForFile(titel):
    import tkinter as tk
    from tkinter.filedialog import askopenfilename
    root = tk.Tk()
    root.withdraw()
    
    return askopenfilename(title = titel)

def drawC(img, xy, r, collor):
    from skimage.draw import circle
    rr, cc = circle(xy[0],xy[1], r)
    img[rr, cc] = collor

videoFile = askForFile("Video")
outPutDir = askForDir("Frame output")
fileOut = lambda count: os.path.join(outPutDir,"frame{}.png".format(count))

def getXYR(shape):
    minSize = min(shape[0],shape[1])
    r = minSize/10
    return r, (r*1.25,shape[1]-r*1.25)

def imgSafe(args):
    cv2.imwrite(fileOut(args[1]), args[0]) 

threader = SThreader.SThreader(imgSafe)
threader.startThreda(5)

KEY_SPACE = 32
KEY_ESC = 27

RECODE_COLLOR = [0,0,255]
vidcap = cv2.VideoCapture(videoFile)

if (vidcap.isOpened()== False): 
    print("Error opening video file")
    exit(1)
recode = 0
count = 0
while(vidcap.isOpened()):
    success, frame = vidcap.read()
    frameShow = frame.copy()
    
    if success:
        if recode > 2:
            r, xy = getXYR(frameShow.shape)
            drawC(frameShow, xy, r, RECODE_COLLOR)
        
        cv2.imshow('Video',frameShow)
        pressedKey = cv2.waitKey(25)
        print(pressedKey)
        if(pressedKey == KEY_SPACE):
            args = [frame, count]
            threader.giveJob(args)
            recode = True
        elif(pressedKey == KEY_ESC):
            cv2.destroyAllWindows()    
            break
        else:
            recode = False
            
        count += 1
    else:
        break
    
threader.awaitThreads()
