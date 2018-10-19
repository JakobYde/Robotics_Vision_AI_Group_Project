# -*- coding: utf-8 -*-
"""
Created on Tue Jul 24 21:31:16 2018

@author: SimonLBS
"""
import threading
from queue import Queue

class SThreader:
    def __init__(self, f):
        # Create the queue and threader 
        self.q = Queue()
        self.funktion = f
        self.joinR = False
        self.kill = False
        
    def __threader(self):
        while True:
            if (self.q.empty() and self.joinR) or self.kill:
                break
            
            # gets an args from the queue
            args = self.q.get()
    
            # Run the example job with the avail args in queue (thread)
            self.funktion(args)
    
            # completed with the job
            self.q.task_done()
            
            
    
    def startThreda(self, numThreds):
        # how many threads are we going to allow for
        for x in range(numThreds):
             t = threading.Thread(target=self.__threader)
        
             # classifying as a daemon, so they will die when the main dies
             t.daemon = True
        
             # begins, must come after daemon definition
             t.start()
             
        print(self.funktion.__name__+" threads running")
             
    def giveJob(self, args):
        self.q.put(args)
             
    def awaitThreads(self):
        self.joinR = True
        self.q.join()
        print(self.funktion.__name__+" threads joind")
        
    def killThreads(self):
        self.kill = True
#    def __del__(self):