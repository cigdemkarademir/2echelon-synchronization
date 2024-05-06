#!/usr/bin/env python
# coding: utf-8

# In[1]:

import DecomposedModels
import solution
from json import load, dump
import os
import io
import numpy
import math 
import sys 
from gurobipy import *
from gurobipy import GRB
import pandas
import time
import matplotlib.pyplot as plt
from pathlib import Path
from os import walk
import random
from gurobipy import quicksum
from gurobipy import Model
from gurobipy import LinExpr

def Solve_by_decomposition(problem1):
    global problem 
    problem = problem1
    #solve by decomposing
    problem.bounded_by_four = False
    bestSolution = solution.Solution(False)
    bestSolution.synch_cost = 100000
    bestSolution.lowerFixed = 100000
    bestSolution.isFeasible = False
    
    problem.decomposedModel = DecomposedModel(bestSolution, problem) 
    problem.decomposedModel.masterProblem.model = DecomposedModels.MasterProblem(problem)
    problem.decomposedModel.masterProblem.model._transferDuals = []   
    
    problem.decomposedModel.bestSolution.lagrangian_bound = 100000  

    #solving       
    problem.decomposedModel.startTime = time.time()    
    problem.decomposedModel.onlyFeasCuts = problem.onlyFeasCuts    
    problem.decomposedModel.fixed_routing_solving = False
    problem.initial_heuristic_time = time.time() - problem.decomposedModel.startTime
    problem.initial_opt_cuts = len(problem.decomposedModel.feas_cuts)

    problem.decomposedModel.masterProblem.solveDecomposed(problem)   
    
    problem.decomposedModel.report_best_solution(problem)
    return 
class DecomposedModel:
    def __init__(self, bestSolution,problem1):
        problem = problem1
        self.feas_cuts = []
        self.inf_cuts = []
        self.exact_inf_cuts = []
        self.exact_feas_cuts = []
        self.onlyFeasCuts = False
        self.bestSolution = bestSolution 
        self.feasibleSolutions = [] 
        self.inFeasibleSolutions = []
        self.allSolutions = []
        self.highestLower = 0
        self.boundsOverTime = []
        self.startTime= time.time()
        self.timeLimit = 10800
        self.terminated = False
        self.routingSet = []
        self.transferSet = []
        self.streetSolutions = []        
        self.routingTime = 0
        self.nofInitialization =0
        self.masterProblem = MasterProblem(problem)
        self.routingIntegral = 0
        self.transferIntegral = 0
        self.optimalityAchieved = False
        self.optimalityGap = 1
        self.upperBound = math.inf
        self.improvements = []
        self.fixed= False
        self.gapAllowed = 0.15
        self.bestKnownLower = 0
        
    def report_best_solution(self, problem):
        self.check_found_solutions_IP()
                
        problem.decomposedModel.bestSolution.ReportStatisticsSolution(problem, problem.decomposedModel.bestSolution.travelStreet, problem.decomposedModel.bestSolution.travelWater*problem.travelcostwater)
        problem.decomposedModel.write_out_solution(problem)
        return
    def write_out_solution(self, problem):
        currentSolution = problem.decomposedModel.bestSolution
        instanceNameSplit = (problem.name).split(".")
        file4 = open(os.path.dirname(os.path.abspath(__file__)) +"/ResultsLBBD/" +instanceNameSplit[0]+"Nodes"+str(len(problem.customers))+str(problem.costScenario)+str(problem.fix_ref_scenario)+str(problem.normalized)+str(problem.decomposedModel.onlyFeasCuts)+str(int(currentSolution.objVal))+"BestRoutesLBBD.txt","w")
        file4.write("Problem and solution setting"+"\n")
        if currentSolution.isFeasible:
            file4.write("Solution feasibility"+"\t"+str(currentSolution.isFeasible)+"\n")
            file4.write("Best solution objective"+"\t"+str(currentSolution.objVal)+"\n") 
            file4.write("Best solution nof vessels"+"\t"+str(currentSolution.nVessel)+"\n")
            file4.write("Best solution nof LEFVs"+"\t"+str(currentSolution.nCars )+"\n")
            file4.write("Best solution travel water"+"\t"+str(currentSolution.travelWater )+"\n")
            file4.write("Best solution travel street"+"\t"+str(currentSolution.travelStreet)+"\n")
            file4.write("Best solution arcs routing"+"\n")
            for [i,j] in currentSolution.xij:
                file4.write(str(i) + " " + str(j)+"\n")            
            
            file4.write("Best solution transfers"+"\n")
            for i in currentSolution.vi:
                file4.write(str(i)+"\n")
            file4.write("Best solution assigned hubs"+"\n")
            for i in currentSolution.hubs:
                file4.write(str(i)+"\n")
        else:
            file4.write("No solution found")            

        file4.close()
        return 
    def isFound_solution(self,usedArcs, transferPoints):
        usedArcs.sort()
        transferPoints.sort()
        found = False      
        
        
        for current in self.allSolutions:
            found = True
            #check lengths
            if len(transferPoints) != len(current.vi):
                found = False
                continue
            if len(usedArcs) != len(current.xij):
                found = False
                continue
            #check one by one
            for v in transferPoints:
                if(v not in current.vi):
                    found=False
                    break
            if not found:
                continue 
            for v in usedArcs:
                if(v not in current.xij):
                    found = False
                    break
            if found:
                prevSolution = current
                break       
        
        if not found:
            prevSolution = solution.Solution(False)
            prevSolution.xij = usedArcs
            prevSolution.vi = transferPoints            
            prevSolution = DecomposedModels.Subproblem_bounded(problem, prevSolution)
            self.allSolutions.append(prevSolution)
        
        prevSolution.foundBefore = found

        return prevSolution 
    def check_found_solutions_IP(self):
        xxx = [solutionF for solutionF in problem.decomposedModel.allSolutions if not solutionF.foundInregral]
        problem.decomposedModel.allSolutions =  sorted(problem.decomposedModel.allSolutions, key=lambda x: len(x.xij), reverse=False)
        for solutionF in xxx:
            if not solutionF.foundInregral:
                if solutionF.lowerFixed > problem.decomposedModel.bestSolution.objVal:                    
                    solutionF.foundInregral = True
                    
                else:
                    solutionF = DecomposedModels.Solve_integral_synchronization_subproblem(problem, solutionF)
            #if solutionF.objVal > problem.decomposedModel.bestSolution.objVal:
            #    solutionF.isFeasible = False  
            if solutionF.isFeasible:
                try:
                    currentSolution = solutionF
                    sumDualsR = sum(rc for [i,rc] in currentSolution.piT)
                    sumDualsE = sum(rc for [i,rc] in currentSolution.piT_upper)
                    
                    sum_relaxed = currentSolution.synch_cost - sumDualsR 
                    sum_exact = currentSolution.exact_synch_cost - sumDualsE 
                    difference = sum_exact - sum_relaxed

                    print(str(currentSolution.objVal)+ "\t"+ str(currentSolution.lowerFixed )+ "\t"+ str(currentSolution.lowerBoundRoot )+ "\t"
                        + str(sumDualsE)+"\t"+ str(sumDualsR)+ "\t"
                        +str(currentSolution.exact_synch_cost)+ "\t"+ str(currentSolution.synch_cost )+ "\t")
                except:
                    pass
        return
    
    
# In[8]:
class MasterProblem:
    def __init__(self, problem):
        self.routingVars = []
        self.transferVars = []
        self.model = Model("MasterProblem") 
        self.initialSolutions = []
        self.multipliers = []
        self.problem = problem
     
    def feedSolutions(self,modelIn, solutionsIn):
        if not solutionsIn:
            return
        print("an incumbent solution exists")
        print(len(solutionsIn))                
        modelIn.NumStart =min(1,  len(solutionsIn))
        solutionsIn =  sorted(solutionsIn, key=lambda x: x.lowerFixed, reverse=False)
        
        # iterate over all MIP starts
        for s in range(modelIn.NumStart):
            solutionF = solutionsIn[s]
            # set StartNumber
            modelIn.params.StartNumber = s
            print("feasible cuts" +str(s+1)+"\t"+str(solutionF.objVal)+"\t"+str(solutionF.lowerBoundRoot))
            #initials for binary variables                   
            for [i,j] in self.routingVars:      
                if [i,j] in solutionF.xij:
                    modelIn.getVarByName("x."+str(i)+"."+str(j)).Start = 1
                else:
                    modelIn.getVarByName("x."+str(i)+"."+str(j)).Start = 0
                
            for i in self.transferVars:      
                if i in solutionF.vi:                    
                    modelIn.getVarByName("v."+str(i)).Start = 1
                else:
                    modelIn.getVarByName("v."+str(i)).Start = 0
            
        modelIn.update()
        return 
    def generate_arcs(self, routes):
        arcs = []

        for route in routes:
            arcs.append([0,route[0]])

            for ii, ind in enumerate(route):
                if ii < len(route) - 1 :
                    arcs.append([ind, route[ii+1]])
                else:
                    arcs.append([ind, 0])
        
        return arcs
    def Solve_Fixed_Routing(self, problem, solutionIn):
        if True:
            print("Solving fixed routing for transfer optimization")
            
            copy = self.model.copy() 
            copy.NumStart = 1
            copy.params.StartNumber= 0  
            copy.update()
            for [i,j] in self.routingVars:      
                if [i,j] in solutionIn.xij:
                    copy.getVarByName("x."+str(i)+"."+str(j)).LB = 1
                else:
                    copy.getVarByName("x."+str(i)+"."+str(j)).UB = 0
                
            for i in self.transferVars:      
                if i in solutionIn.vi:                    
                    copy.getVarByName("v."+str(i)).Start = 1
                else:
                    copy.getVarByName("v."+str(i)).Start = 0
            copy.setParam("LazyConstraints", 1)
            copy.getVarByName("zObj").UB = problem.decomposedModel.bestSolution.objVal           
            
            problem.decomposedModel.fixed_routing_solving = True
            copy.optimize(callbackLBBD)
            problem.decomposedModel.fixed_routing_solving = False
            try:                
                objt = self.model.getObjective()
                objVal = round(objt.getValue(),2)  
                solutionIn.objVal_lower_fixed = objVal
                solutionIn.isFeasible = True

            except:
                solutionIn.isFeasible = False
                #copy.write("model.lp")

        return solutionIn
    def get_optimal_master(self):
        
        master_Feasible =  not (self.model.status == 3  or self.model.status == 4 or  self.model.status == 6)
        if self.model.status == GRB.INFEASIBLE or self.model.status == GRB.INF_OR_UNBD:
            master_Feasible = False
        try:            
            objt = self.model.getObjective()
            objVal = round(objt.getValue(),2)  
        except:
            master_Feasible = False
            self.status =   self.model.status
            self.objVal = 100000
            self.cplex_gap = 1

        if master_Feasible:            
            print(objVal)
            self.status =   self.model.status
            self.objVal = objVal
            self.cplex_gap = self.model.MIPGap
            binary_vars  = [v for v in self.model.getVars() if v.vType == GRB.BINARY]
            
            k = min(1,self.model.SolCount)
            k_best_solutions = []
            for i in range(k):
                self.model.setParam('SolutionNumber', i)
                
                transfer_decisions = []
                arc_decisions = []
                for v in self.model.getVars():
                    if v.x > 0.5:
                        name = v.varName.split(".")            
                        if(name[0] == "x" ):
                            arc_decisions.append([int(name[1]), int(name[2])])
                    
                        if(name[0] == "v"): 
                            transfer_decisions.append(int(name[1]))
                            
                    
                k_best_solutions.append([arc_decisions, transfer_decisions, self.model.getVarByName("zObj").x])
            k_best_solutions.sort(key=lambda x: x[2])
            #get street level decisions
            for solutionM in k_best_solutions:
                if solutionM[2] > problem.decomposedModel.bestSolution.objVal:
                    continue
                arc_decisions = solutionM[0]
                transfer_decisions = solutionM[1]
                
                currentSolution = problem.decomposedModel.isFound_solution(arc_decisions, transfer_decisions)
                if currentSolution.foundBefore:
                    continue
                currentSolution.lowerBoundRoot = solutionM[2]
                problem.decomposedModel.check_found_solutions_IP() 
                
                
                
        return problem.decomposedModel.bestSolution
    
    def solveDecomposed(self, problem1):        
        #set uo solving process for lazy constraints to be checked        
        self.model._incumbent_variables = [] 
        self.model._incumbent_values = [] 
        self.cplex_gap = -1
        self.status = -1
        self.objVal = -1
        # Set up solution pool to store multiple solutions       
        
        #self.model.setParam('Nodelimit', 10)
        self.model.setParam('Timelimit', 50)  
        if len(problem.nodes)>50:
            self.model.setParam('Timelimit', 200)
        self.model.setParam('OutputFlag', 0)
        self.model._initialsols = []
        self.model.optimize(callbackWarmUp)
        self.model._initialsols.sort(key=lambda x: x[2])
        for [arcs, transfers, obj] in self.model._initialsols:
            if obj <= problem.decomposedModel.bestSolution.objVal:
                solution_k = problem.decomposedModel.isFound_solution(arcs, transfers)
                problem.decomposedModel.check_found_solutions_IP()  
        #warm up results
        initial_best_feaasible_objVal = problem.decomposedModel.bestSolution.objVal
        initial_best_feaasible_nCars = problem.decomposedModel.bestSolution.nCars
        initial_best_feaasible_travelStreet = problem.decomposedModel.bestSolution.travelStreet
        initial_best_feaasible_travelWater = problem.decomposedModel.bestSolution.travelWater
        initial_best_feaasible_nVessel = problem.decomposedModel.bestSolution.nVessel
        initial_best_masterObj = problem.decomposedModel.bestSolution.lowerBoundRoot
        #master_solution = self.get_optimal_master()
        problem.decomposedModel.terminated = True
        
        while problem.decomposedModel.terminated:#   
            
            problem.decomposedModel.nofInitialization +=1            
            problem.decomposedModel.bestSolution.usedAsSolution = False
            problem.decomposedModel.terminated = False       
            

            problem.decomposedModel.allSolutions.sort(key=lambda x: x.objVal)
                           
            self.feedSolutions(self.model, problem.decomposedModel.allSolutions)

            if problem.decomposedModel.bestSolution.isFeasible and not problem.decomposedModel.onlyFeasCuts:
                cuts = addCut(problem.decomposedModel.bestSolution, self.model,
                                    problem.decomposedModel.masterProblem.routingVars, problem.decomposedModel.masterProblem.transferVars, "ciko3") 
                for cut in cuts:
                    self.model.addConstr(cut) 
                self.model.addConstr(self.model.getVarByName("zObj") <= problem.decomposedModel.bestSolution.objVal)
            print("master routing started")            
            startRouting = time.time()   
            timeRemain = problem.timeLimit - round(time.time() - problem.decomposedModel.startTime,2) 
            if timeRemain < 5 :
                break
            
            self.model.setParam('Nodelimit', math.inf)
            self.model.setParam('Timelimit', timeRemain)

            self.model.optimize(callbackLBBD)
            self.model.update()            
            
            master_solution = self.get_optimal_master()

            #problem.decomposedModel.allSolutions.sort(key=lambda x: x.lowerBoundRoot, reverse=False)
            #for solution in problem.decomposedModel.allSolutions:              
            #    self.Solve_Fixed_Routing(problem, solution)     
            
            
            problem.decomposedModel.check_found_solutions_IP()   
            
            endRouting = time.time()
            problem.decomposedModel.routingTime += round(endRouting - startRouting,2)
            
            print("Rebuilding"+"Terminated"+str(problem.decomposedModel.terminated)+"Status"+"\t"+str(self.model.status)+"IncumbetFeas"+str(problem.decomposedModel.bestSolution.isFeasible))
            print("11new incumbent to master problem" +"\t"+ str(problem.decomposedModel.bestSolution.objVal))
            print("best solution lower bound"+"\t"+str(problem.decomposedModel.bestSolution.lowerFixed))
            print("Global lower bound "+"\t"+str(problem.decomposedModel.highestLower))
            
               
        instanceNameSplit = (problem.name).split(".")
        
        problem.decomposedModel.bestSolution.isFeasible = True
        if problem.decomposedModel.bestSolution.isFeasible: # If optimal solution is found
            problem.decomposedModel.feas_cuts = sorted(problem.decomposedModel.feas_cuts, key=lambda x: x.lowerFixed, reverse=False)
            
            count_best_cuts = min(5, len( problem.decomposedModel.feas_cuts))
            average_LB = 0
            if count_best_cuts > 0:
                average_LB = sum(x.lowerFixed for x in problem.decomposedModel.feas_cuts[0:count_best_cuts])/count_best_cuts
            if len(problem.decomposedModel.improvements)>0:
                averageImp = round( sum(problem.decomposedModel.improvements) / len(problem.decomposedModel.improvements),1)
            else:
                averageImp = 0
            if problem.onlyFeasCuts:
                file1 = open(problem.report,"a")
                file1.write(instanceNameSplit[0]+"\t"+str(len(problem.customers))+"\t"+str(problem.costScenario)+"\t"+str(problem.fix_ref_scenario)+"\t"+str(problem.normalized)
                            +"DecomposedTwoIndex"+"\t"+"final"+"\t"+str(round(problem.decomposedModel.bestSolution.objVal,2))+"\t"+ str(round(problem.decomposedModel.bestSolution.nCars,2))
                            +"\t"+str(round(problem.decomposedModel.bestSolution.travelStreet,2))+"\t"+str(round(problem.decomposedModel.bestSolution.nVessel,2))
                            +"\t"+str(round(problem.decomposedModel.bestSolution.travelWater,2))+"\t"+str(self.cplex_gap)+"\t"+ str(self.status)
                            +"\t"+str(round(problem.decomposedModel.bestSolution.travelWater+problem.decomposedModel.bestSolution.travelStreet,2))
                            +"\t"+str(round(time.time() - problem.decomposedModel.startTime,2))+"\t"+"IncumbentFoundTime"+"\t"+str(problem.bestFoundTime)
                            +"\t"+str(round(problem.decomposedModel.bestSolution.lowerFixed,2))
                            +"\t"+str(round(problem.decomposedModel.bestSolution.lowerBoundRoot,2))+"\t"#+str(round(problem.decomposedModel.bestSolution.optimalityBound,2))
                            +"\t"+str(round(self.objVal,2))
                            +"\n" )
                file1.close() 
                return
            
                        
            file1 = open(problem.report,"a")
            file1.write(instanceNameSplit[0]+"\t"+str(len(problem.customers))+"\t"+str(problem.costScenario)+"\t"+str(problem.fix_ref_scenario)+"\t"+str(problem.normalized)
                        +"DecomposedTwoIndex"+"\t"+"final"+"\t"+str(round(problem.decomposedModel.bestSolution.objVal,2))+"\t"+ str(round(problem.decomposedModel.bestSolution.nCars,2))
                        +"\t"+str(round(problem.decomposedModel.bestSolution.travelStreet,2))+"\t"+str(round(problem.decomposedModel.bestSolution.nVessel,2))
                        +"\t"+str(round(problem.decomposedModel.bestSolution.travelWater,2))+"\t"+str(self.cplex_gap)+"\t"+ str(self.status)
                        +"\t"+str(round(problem.decomposedModel.bestSolution.travelWater+problem.decomposedModel.bestSolution.travelStreet,2))
                        +"\t"+str(round(time.time() - problem.decomposedModel.startTime,2))+"\t"+"IncumbentFoundTime"+"\t"+str(problem.bestFoundTime)
                        +"\t"+str(round(problem.decomposedModel.bestSolution.lowerFixed,2))
                        +"\t"+str(round(problem.decomposedModel.bestSolution.lowerBoundRoot,2))+"\t"#+str(round(problem.decomposedModel.bestSolution.optimalityBound,2))
                        +"\t"+str(round(self.objVal,2))     
                        +"\t"+"initial"+"\t"+str(round(initial_best_feaasible_objVal,2)) 
                        +"\t"+str(round(initial_best_masterObj,2))
                        +"\t"+str(round(initial_best_feaasible_nCars,2)) 
                        +"\t"+str(round(initial_best_feaasible_travelStreet,2)) 
                        +"\t"+str(round(initial_best_feaasible_nVessel,2)) 
                        +"\t"+str(round(initial_best_feaasible_travelWater,2)) 
                        +"\t"+str(round(problem.initial_heuristic_time,2))
                        +"\t"+str(problem.initial_opt_cuts)
                        +"\t"+str(problem.decomposedModel.bestSolution.synch_cost)  
                        +"\t"+str(round(problem.decomposedModel.bestSolution.fixed_arcs_cost + problem.decomposedModel.bestSolution.synch_cost,2)) 
                        +"\t"+str(round(sum(rc for [i,rc] in problem.decomposedModel.bestSolution.piT),2))    
                         +"\t"+str(round(problem.decomposedModel.bestSolution.fixed_arcs_cost))        
                        +"\t"+"Average best 5 bounds"+"\t"+str(round(average_LB, 2))
                        +"\t"+"Number of solutions found"+"\t"+str(len(problem.decomposedModel.feasibleSolutions))
                        +"\t"+"Routing Time"+"\t"+str(round(problem.decomposedModel.routingTime,2))                 
                        +"\t"+"nof initializations"+"\t"+str(problem.decomposedModel.nofInitialization)
                        +"\t"+str(averageImp)
                        +"\t"+str(problem.nodes[0].latest)
                        
                        +"\n" )
            file1.close()            
            
            
        else:
            file1 = open(problem.report,"a")
            file1.write(instanceNameSplit[0]+"\t"+str(len(problem.customers))+"\t"+str(problem.costScenario)+"\t"+str(problem.fix_ref_scenario)+"\t"+str(problem.normalized)
                        +"DecomposedTwoIndex"+"\t"+"No solution"+"\n" )
            file1.close()    

def callbackLBBD(model, where):   
    if where == GRB.Callback.MIPSOL:
        objVar = model.getVarByName("zObj")
        synch_cost = model.getVarByName("synch_cost")
        relaxedObj = model.cbGetSolution(objVar)   
        synch_cost = model.cbGetSolution(synch_cost)

        if relaxedObj > problem.decomposedModel.bestSolution.objVal + 0.001 :
            return   
        
        # MIP solution callback
        binary_vars  = [v for v in model.getVars() if v.vType == GRB.BINARY]
        values =  model.cbGetSolution(binary_vars)

        transfer_decisions = []
        arc_decisions = []
        #get street level decisions
        for k, v in enumerate(binary_vars):
            if values[k] > 0.5:
                name = v.varName.split(".")            
                if(name[0] == "x" ):
                    arc_decisions.append([int(name[1]), int(name[2])])
            
                if(name[0] == "v"): 
                    transfer_decisions.append(int(name[1]))
        
        currentSolution = problem.decomposedModel.isFound_solution(arc_decisions, transfer_decisions)
        
        problem.decomposedModel.check_found_solutions_IP()  #final check for integer solution     

        currentSolution.lowerBoundRoot = relaxedObj
        currentSolution.variables = binary_vars
        currentSolution.values = values      
        
        
        if  currentSolution in problem.decomposedModel.feas_cuts and not problem.decomposedModel.onlyFeasCuts :#and currentSolution.isFeasible: #optimality cut:                         
            cuts = addCut(currentSolution, model,
                            problem.decomposedModel.masterProblem.routingVars, problem.decomposedModel.masterProblem.transferVars, "ciko3") #optimality cut
            for cut in cuts:
                model.cbLazy(cut)  
            
        
        if not currentSolution.isFeasible: #feas1                    
            cut = addCut(currentSolution, model,
                            problem.decomposedModel.masterProblem.routingVars, problem.decomposedModel.masterProblem.transferVars, "feas1") #feas1
            model.cbLazy(cut) 
        best_bound = model.cbGet(GRB.Callback.MIPSOL_OBJBND)
        if best_bound >= problem.decomposedModel.bestSolution.lowerFixed :
            model.terminate()
        elif where == GRB.Callback.MIPNODE :
            if model.cbGet(GRB.Callback.MIPNODE_STATUS) == GRB.OPTIMAL:    #called after MIPSOL  
                for solutionF in problem.decomposedModel.feas_cuts:            
                    model.cbSetSolution(solutionF.variables, solutionF.values)
                    objValue1 = model.cbUseSolution()
  
        
def callbackWarmUp(model, where):   
    if where == GRB.Callback.MIPSOL:
        objVar = model.getVarByName("zObj")
        synch_cost = model.getVarByName("synch_cost")
        relaxedObj = model.cbGetSolution(objVar)   
        synch_cost = model.cbGetSolution(synch_cost)
    
        # MIP solution callback
        binary_vars  = [v for v in model.getVars() if v.vType == GRB.BINARY]
        values =  model.cbGetSolution(binary_vars)

        transfer_decisions = []
        arc_decisions = []
        #get street level decisions
        for k, v in enumerate(binary_vars):
            if values[k] > 0.5:
                name = v.varName.split(".")            
                if(name[0] == "x" ):
                    arc_decisions.append([int(name[1]), int(name[2])])
            
                if(name[0] == "v"): 
                    transfer_decisions.append(int(name[1]))
        model._initialsols.append([arc_decisions, transfer_decisions, relaxedObj])       

        return
  

def addCut(solution, modelIn, routingVars, transferVars, cutType):
    
    if cutType == "feas1":
        multiplication = 1
        #if problem.decomposedModel.bestSolution.isFeasible:
        #    multiplication = problem.decomposedModel.bestSolution.lowerFixed 
        exp5 = LinExpr()
        exp5 = (quicksum((multiplication*modelIn.getVarByName("x."+str(i)+"."+str(j))) for [i, j ] in routingVars if [i,j] not in solution.xij) +
                quicksum(multiplication*(1- modelIn.getVarByName("v."+str(i))) for i in transferVars if i in solution.vi)+
                quicksum(multiplication*modelIn.getVarByName("v."+str(i)) for i in transferVars if i not in solution.vi) >= multiplication)
        return exp5
    
    elif cutType == "ciko3": #globally binding cut: relaxed over all decisions of the master problem limited to the selected transfer decisions
        synch_cost_solution =  solution.synch_cost # 
        all_arcs_from_garage = [j for [i,j] in solution.xij if i==0 ]        
        all_arcs_from_customers_no_transfers =  [[i,j] for [i,j] in solution.xij if i not in solution.vi]
        
        arcs_transfers_duals = []
        for [i,rc] in solution.piT:
            j = next(j for [k,j] in solution.xij if k==i)
            arcs_transfers_duals.append([i,j,rc])
        #lower bound for given solution
        boundValue = (synch_cost_solution  + sum(problem.travelcoststreet*problem.DistMatrix[i][j] for [i,j] in all_arcs_from_customers_no_transfers if [i,j] in solution.xij)
                      + sum((problem.penaltyCars) for j in all_arcs_from_garage if [0,j] in solution.xij))
        
        boundValue2 = (sum(problem.travelcoststreet*problem.DistMatrix[i][j] for [i,j] in all_arcs_from_customers_no_transfers if [i,j] in solution.xij)
                      + sum((problem.travelcoststreet*problem.DistMatrix[0][j] + problem.penaltyCars) for j in all_arcs_from_garage if [0,j] in solution.xij)
                      + sum(rc for [i,rc] in solution.piT))

        solution.optimalityBound = boundValue
        solution.lagrangian_bound = boundValue2
        
        cuts = []
         
        cutCiko = (modelIn.getVarByName("zObj") >= problem.penaltyCars*quicksum(modelIn.getVarByName("x."+str(0)+"."+str(j)) for j in all_arcs_from_garage) + solution.synch_cost 
                   + quicksum(modelIn.getVarByName("cij."+str(i)+"."+str(j)) for [i,j] in all_arcs_from_customers_no_transfers)                                                                                
                                                + quicksum( rc*(modelIn.getVarByName("x."+str(i)+"."+str(j)) + modelIn.getVarByName("v."+str(i)) - 2) for [i,j,rc] in arcs_transfers_duals) )
                                                               
                   
        cuts.append(cutCiko)    

        return cuts

