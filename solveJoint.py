import instanceGenerator
import models
import solution
import time
from pathlib import Path
import os
import io
import numpy
from os import walk


input1 = 10# int(sys.argv[1])  #the number of customer nodes to be selected
input2 = -1#int(sys.argv[2])    #the first instance index to be solved -1:7 = Clustered 8:18 = Random 19:26 = Random clustered
input3 = 26#int(sys.argv[3])    #the last instance index to be solved
input4 =  7200# int(sys.argv[4]) #time limit (in seconds)
inputCost =  3# int(sys.argv[5]) #relative cost of water level to street level logistics ->  both level minimizes vehicles first but relative cost ratio changes WL to SL

path = os.path.dirname(os.path.abspath(__file__))+"/data/json"
filenames = next(walk(path), (None, None, []))[2]


if inputCost ==1:
    input5 = 0.1 # water level cost is 10 times cheaper than street level  cost
    
elif inputCost ==2:
    input5 = 0.2 #water level cost is 5 times cheaper than street level  cost
    
elif inputCost ==3:
    input5 = 1 #Both are equally expensive
    
elif inputCost ==4:
    input5 = 10 #water level  cost is 10 times more expensive than street level  cost
    

count = input2
report = os.path.dirname(os.path.abspath(__file__)) +"/report"+str(input1)+str(input2)+str(input3)+"CostSetting"+str(input5)+".txt"
file1 = open(report,"w")
file1.close()
stats = os.path.dirname(os.path.abspath(__file__)) +"/statistics"+str(input1)+str(input2)+str(input3)+"CostSetting"+str(input5)+".txt"
file4 = open(stats,"w")
file4.close()

for instance_name in filenames:
    sys.stdout = open(os.path.dirname(os.path.abspath(__file__)) +"/console"+str(input1)+str(input2)+str(input3)+"CostSetting"+str(input5)+".txt", "w")
    start = time.time()
    count += 1
    instance_name = filenames[count]
    nofWP = input1
    WRS = input5
    timelimit = input4
    problem = instanceGenerator.readGenerate(instance_name, nofWP,WRS, timelimit)
    problem.modelStartTime = time.time()          
    
    problem.report = report
    problem.stats = stats
    
    
    models.JointSynchronized(problem) #solve joint model by BB
    problem.decomposedModel.startTime = time.time()
    #input("Press Enter to continue...")
    
    if count >= input3:
        sys.stdout.close()
        break
    sys.stdout.close()
