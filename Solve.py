#!/usr/bin/env python
# coding: utf-8

# In[1]:

import instanceGenerator
import integratedJointModel
import LBBDmodel
import os 
import sys
import time
import math
from os import walk


#default inputs
# python3 C:\Users\ckarademir\OneDrive - Delft University of Technology\Desktop\Paper Writing\Ejors first paper deterministic 2eVRP\Models\LBBD\LBBD Updated\SolveDecomposed.py 0 27 0 1 0 100
#input parameters
#input1 = First instance index to solve, input2 = Last index to solve...... 
index_start = 0#  int(sys.argv[1])  #the min index 0
index_end =  27 #int(sys.argv[2])# the max index 27 
index_end = min(index_end, 27)
size_start = 0# int(sys.argv[3]) #the min index 0  [10, 20,30,40,50,100]
size_end =  1#int(sys.argv[4])  # the max index 6
cost_scen =  0# int(sys.argv[5]) #max 3 [0.1, 0.2, 1, 10]
time_secs =  3600#int(sys.argv[6]) #time limit in secs
method = "LBBD"# sys.argv[7] #method To solve either "LBBD" or "Joint"
feas_option = 0#(sys.argv[8])
problem_type_scen = 0#int(sys.argv[9])#(0-4) = "ours","VRP", "MVRP","LRP", "2eBenchmark" #sys.argv[9]

problem_choices = ["2eVRP","VRP", "MVRP","2eLRP", "2eVRPBenchmark"]
cost_types = [0.1, 0.2, 1, 10]

    
sys.stdout = open(os.path.dirname(os.path.abspath(__file__)) +"/Hconsole1"+str(problem_choices[problem_type_scen])+method+str(feas_option)+"_"+str(index_start)+"_toinstance"+str(index_end)+"_sizes"+str(size_start)+"_"+str(size_end)+"_cost_scenario"+str(cost_scen)+".txt", "w")
    
for scen_x in range(1):    
    closeness = 0#round(0.1 + 0.1*scen_x,1)
    inputCost = cost_types[cost_scen]

    problem_sizes = [10, 20,30,40,50,100]
    problem_type = problem_choices[problem_type_scen]
    update_rule = [False, False] # First: large costs fixed at makespan, then relative ratios are normalized and multiplied with base costs  

    path = os.path.dirname(os.path.abspath(__file__))+"/data/json"
    filenames = next(walk(path), (None, None, []))[2]

    report = os.path.dirname(os.path.abspath(__file__)) +"/Computations/1604_report_instance_"+"_closeness"+str(closeness)+str(problem_type)+method+str(feas_option)+"_"+str(index_start)+"_toinstance"+str(index_end)+"_sizes"+str(size_start)+"_"+str(size_end)+"_cost_scenario"+str(cost_scen)+".txt"
    file1 = open(report,"w")
    file1.close()
    
    stats = os.path.dirname(os.path.abspath(__file__)) +"/Computations/1604statistics_instance_"+"_closeness"+str(closeness)+str(problem_type)+method+str(feas_option)+"_"+str(index_start)+"_toinstance"+str(index_end)+"_sizes"+str(size_start)+"_"+str(size_end)+"_cost_scenario"+str(cost_scen)+".txt"
    file4 = open(stats,"w")
    file4.close()

    for size in problem_sizes[size_start:size_end]:    
        for instance_name in filenames[index_start:index_end]:           
        
            problem = instanceGenerator.readGenerate(instance_name, size, time_secs, problem_type, closeness)#instance, size, WLS, time limit for solver
            
            minVessel = math.ceil(sum(x.demand for x in problem.nodes if x.index in problem.customers)/problem.VesselCapacity)
           
            #update costs
            problem.costScenario = inputCost
            problem.update_rule = update_rule                             
            problem.Set_cost_coefficients()

            #print("instance: ", instance_name, "size: ", size, "cost: ", problem.costScenario, "minVessel: ", minVessel, "penaltyStreet", problem.penaltyCars, "penaltyVessel", problem.penaltyVessels)
            
            
            #solution
            problem.modelStartTime = time.time()
            problem.report = report
            problem.stats = stats
            problem.onlyFeasCuts = False
            if feas_option == 1:
                problem.onlyFeasCuts = True
    
            if method=="LBBD":
                bestknown_solution = LBBDmodel.Solve_by_decomposition(problem) #solve by decompisition
                
            else:
                bestknown_solution = integratedJointModel.JointSynchronized(problem) #solve joint model by BB       

sys.stdout.close()  
        

            
            

            
            
            
        
