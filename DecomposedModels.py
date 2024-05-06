import os
import io
import numpy
import math 
from gurobipy import *
from gurobipy import GRB
import solution
import time
from gurobipy import quicksum
from gurobipy import Model
from gurobipy import LinExpr
def MasterProblem(problem):
        #sets
    SetCd = []
    SetCd.append(0)
    SetCd.extend(problem.customers) 
    
    Customers = problem.customers
    Satellites = problem.satellites
    CDC = problem.CDC
    arcList = []
    
    model = Model("2EMVRPTWSS")
    
## ----------- STREET PROBLEM-----------------------------------------
    #region
    # ---- Variables ----
    #street level
    xij = {} #street route
    mi = {} #load on car
    hi = {} #service start at wp
    arcStreetCost = {} #arc cost if traversed
    vi = {} #transfer after node i
    nofCars = model.addVar(lb=1,ub=len(Customers), vtype=GRB.INTEGER, name = "nofCars")

    #Define Variables
    for i in Customers:
        vi[i] = model.addVar (vtype=GRB.BINARY, name="v."+str(i))
        if problem.initial_solution:  #if exist any initial solution 
            if i in problem.initial_solution.vi: 
                vi[i].Start = 1 
            else:
                vi[i].Start = 0

        hi[i] = model.addVar(lb=problem.nodes[i].earliest,ub=problem.nodes[i].latest, vtype=GRB.CONTINUOUS, 
                           name="h["+str(i)+"]")
        mi[i] = model.addVar(lb=problem.nodes[i].demand,ub=problem.CarCapacity, vtype=GRB.CONTINUOUS, 
                           name="m["+str(i)+"]")
    hi[0] = model.addVar(lb=problem.nodes[0].earliest,ub=problem.nodes[0].latest, vtype=GRB.CONTINUOUS, 
                           name="u["+str(0)+"]")
    

    for i in SetCd:
        for j in SetCd:
            if i==j:
                continue   
            if problem.nodes[i].earliest + problem.nodes[i].serviceTime + problem.DistMatrix[i][j]<=problem.nodes[j].latest:
                xij[i,j]=model.addVar (vtype=GRB.BINARY, name="x."+str(i)+"."+str(j))                
                arcList.append([i,j])
                
                arcStreetCost[i,j] = model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS, 
                           name="arcCostS["+str(i)+"."+str(j)+"]")
                
                if problem.initial_solution:
                    if [i,j] in problem.initial_solution.xij: #if exist any initial solution 
                        xij[i,j].Start = 1 
                    else:
                        xij[i,j].Start = 0 

    #---Part 1----- Street level routing Problem Constraints 
    #assignment const
    for i in Customers:
        model.addConstr(quicksum(xij[i,j] for j in SetCd if [i,j] in arcList)==1, "outflow."+str(i)) #Const2
        model.addConstr(quicksum(xij[j,i] for j in SetCd if [j,i] in arcList)==1,"inflow."+str(i)) #Const2
    
    #fleet size and  #flow balance at the depot (garage for street vehicles)
    model.addConstr(quicksum(xij[i,0] for i in Customers) == nofCars) #Const3 
    model.addConstr(quicksum(xij[i,0] - xij[0, i] for i in Customers) == 0) #Const3 

    #last transfer assignment before depot
    for i in Customers:
        model.addConstr(vi[i] >= xij[i,0]) #Const4
    
    #load on the cars #Const5
    for i in Customers:
        for j in Customers:
            if [i,j] in arcList:
                model.addConstr(mi[j] - mi[i] >= problem.nodes[j].demand - problem.CarCapacity*(1 - xij[i,j] +vi[i]))

                
    #service start time binding with depot times #const7
    for i in Customers:
        model.addConstr(float(problem.nodes[0].earliest + problem.DistMatrix[0][i]) <= hi[i]) 
        model.addConstr(float(problem.nodes[0].latest - problem.DistMatrix[i][0] - problem.nodes[i].serviceTime - problem.TransferDuration ) >= hi[i])
        
    
    #service start times flowing #Const8
    for i in Customers:
        for j in SetCd:
            if [i,j] in arcList:
                maxTravel = max(problem.DistMatrix[i][p] + problem.DistMatrix[p][j] for p in Satellites)
                Mij = problem.nodes[i].latest + problem.nodes[i].serviceTime + problem.TransferDuration + maxTravel  - problem.nodes[j].earliest 
                model.addConstr(hi[i] + problem.nodes[i].serviceTime+problem.TransferDuration*vi[i]
                                +  arcStreetCost[i,j] <= hi[j] + Mij - Mij*xij[i,j])
        
    #arc costs #Const 9
    for [i,j] in arcList:
        model.addConstr(arcStreetCost[i,j]>=problem.DistMatrix[i][j]*xij[i,j]) #objCoeff
        
        if i in Customers:
            minTravel = min(problem.DistMatrix[i][p] + problem.DistMatrix[p][j] - problem.DistMatrix[i][j] for p in Satellites)
            model.addConstr(arcStreetCost[i,j]>= problem.DistMatrix[i][j]*xij[i,j] + minTravel*(xij[i,j]+vi[i] - 1)) #objCoeff

    #endregion
        
    #alternative systems VRPTW vs MVRPTW on the streets
    # the only change in the master problem is to set the number of transfers to the number of vehicles for VRP case, only trucks
    if not problem.multitripsOnStreets: #VRPTW
        model.addConstr(quicksum(xij[i,0] - vi[i] for i in Customers) == 0) #each truck only transfers to depot at the end of the single trip, route

        #(A) Objectives
    #--------------------------------------
    # Define objective components
    FleetStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetStreet") #number of cars 
    TravelStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelStreet") 
    estimatedWatercost=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="synch_cost")
    fixedcost=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="fixed_cost")

    
    model.addConstr(nofCars*problem.penaltyCars<= FleetStreet)

    cij = {}
    synch_ij = {}
    exp3 = LinExpr()
    exp3.clear() #street movements             
    for [i,j] in arcList:
        cij[i,j] = model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="cij."+str(i)+"."+str(j))
        if i !=0:
            synch_ij[i,j] = model.addVar(lb=0,ub=math.inf,  vtype=GRB.CONTINUOUS,name="synch_ij."+str(i)+str(j))
            model.addConstr(cij[i,j] >= problem.travelcoststreet * problem.DistMatrix[i][j] * (xij[i,j] - vi[i])) #objCoeff * xij[i,j])
            model.addConstr(synch_ij[i,j] >= problem.travelcoststreet * min(problem.DistMatrix[i][p] + problem.DistMatrix[p][j]  for p in Satellites )* (xij[i,j] + vi[i] - 1)) #objCoeff * xij[i,j])
            
        else:
            model.addConstr(cij[i,j] >= problem.travelcoststreet * problem.DistMatrix[i][j] * xij[i,j]) #objCoeff * xij[i,j])
        
        exp3.addTerms(1, cij[i,j])                
                
    model.addConstr(exp3<=TravelStreet)    

           
    #lower bound on estimated cost
    # Set the type of objective
    zObj=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="zObj")
    zwater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="zwater")    
    
    sumDemand = sum(problem.nodes[iix].demand for iix in Customers)
    lowerVessels = math.ceil (sumDemand / problem.VesselCapacity )
    minTr = min(2*problem.DistMatrix[CDC][siix] for siix in Satellites)
    model.addConstr(zwater>=problem.travelcostwater*minTr*lowerVessels + lowerVessels*problem.penaltyVessels)
    
    model.addConstr(estimatedWatercost >= zwater +  quicksum(synch_ij[i,j] for [i,j] in arcList if i !=0))
    model.addConstr(fixedcost >= FleetStreet + TravelStreet)
    model.addConstr(fixedcost+ estimatedWatercost  <= zObj)
    model.setObjective(zObj, GRB.MINIMIZE)

    problem.decomposedModel.masterProblem.routingVars.extend(arcList)
    problem.decomposedModel.masterProblem.transferVars.extend(Customers) 

    model.setParam('OutputFlag', 0)
    model.setParam("LazyConstraints", 1)
    model.setParam("PreCrush", 1)
    model.setParam("MIPGap", 0.0)
    return model

def get_optimal_solution(problem, currentSolution):
    model = currentSolution.model
    arcList = currentSolution.xij
    transferSet = currentSolution.vi
    nofCars = sum(1 for [i,j] in arcList if i == 0)
    #sets
    SetCd = []
    SetCd.append(0)
    SetCd.extend(problem.customers) 

    feasible_exists = not(model.status == 3  or model.status == 4 or  model.status == 6)
    
    try:
        objt = model.getObjective()
        objVal = round(objt.getValue(), 4)        
        print(objVal)        
    except:
        feasible_exists = False
    #################### optimized #######################################################
    if not feasible_exists:
        currentSolution.timeFound = round( time.time() - problem.decomposedModel.startTime, 4)        
        problem.decomposedModel.inFeasibleSolutions.append(currentSolution)
        problem.decomposedModel.exact_inf_cuts.append(currentSolution)
        currentSolution.isFeasible = False
    else: 
        problem.decomposedModel.exact_feas_cuts.append(currentSolution) 
        currentSolution.hubs = []        
        currentSolution.detours= []
        satellite_assignments = []
        for i in currentSolution.vi:
            currentSolution.detours.append(round(model.getVarByName("detour["+str(i)+"]").x,2))
            for p in problem.satellites:                        
                if (model.getVarByName("vip."+str(i)+"."+str(p)).x > 0.5):
                        currentSolution.hubs.append(p)       
                        satellite_assignments.append([i,p])                 
                
        currentSolution.isFeasible = True
        currentSolution.objVal = round(model.getVarByName("zObj").x,6)
        currentSolution.exact_synch_cost =  round(model.getVarByName("synch_cost").x,6)
        currentSolution.addedBound = False            
        currentSolution.nCars = nofCars        
        currentSolution.travelStreet = round(model.getVarByName("TravelStreet").x/problem.travelcoststreet,2)
        currentSolution.fleetStreet = currentSolution.nCars*problem.penaltyCars
        currentSolution.nVessel = int(model.getVarByName("nofVessels").x+0.1)        
        currentSolution.fleetWater = problem.penaltyVessels*currentSolution.nVessel 
        if problem.twoechelon:
            currentSolution.travelWater = round(model.getVarByName("TravelWater").x/problem.travelcostwater,2)
        else:
            currentSolution.travelWater = round(model.getVarByName("TravelWater").x,2)        
       
        
        for i in SetCd:            
            for j in SetCd:
                if i==j:
                    continue   
                if [i,j] in arcList:
                    if i not in transferSet:
                        currentSolution.arcCostS.append([[i,j], problem.DistMatrix[i][j] ])
                    else:
                        currentSolution.arcCostS.append([[i,j], round(model.getVarByName("detour["+str(i)+"]").x,2)])
        
        #update best solution
        problem.decomposedModel.feasibleSolutions.append(currentSolution)
        if currentSolution.objVal <= problem.decomposedModel.bestSolution.objVal - 0.001:
            problem.bestFoundTime = round( time.time() - problem.decomposedModel.startTime, 4)
            currentSolution.timeFound = round( time.time() - problem.decomposedModel.startTime, 4) 
            problem.decomposedModel.bestSolution = currentSolution
            problem.decomposedModel.bestSolution.usedAsSolution = False
            currentSolution.isBest = True
            print("new best solution, terminate BB")  
        
        
        if currentSolution.nofCars + currentSolution.nVessel <= problem.decomposedModel.bestSolution.nofCars + problem.decomposedModel.bestSolution.nVessel:
            if currentSolution.nofCars < problem.decomposedModel.bestSolution.nofCars and currentSolution.nVessel <= problem.decomposedModel.bestSolution.nVessel:
                problem.bestFoundTime = round( time.time() - problem.decomposedModel.startTime, 4)
                currentSolution.timeFound = round( time.time() - problem.decomposedModel.startTime, 4) 
                problem.decomposedModel.bestSolution = currentSolution
                problem.decomposedModel.bestSolution.usedAsSolution = False
                currentSolution.isBest = True
                print("new best solution, terminate BB")  
            elif currentSolution.nofCars <= problem.decomposedModel.bestSolution.nofCars and currentSolution.nVessel < problem.decomposedModel.bestSolution.nVessel:
                problem.bestFoundTime = round( time.time() - problem.decomposedModel.startTime, 4)
                currentSolution.timeFound = round( time.time() - problem.decomposedModel.startTime, 4) 
                problem.decomposedModel.bestSolution = currentSolution
                problem.decomposedModel.bestSolution.usedAsSolution = False
                currentSolution.isBest = True
                print("new best solution, terminate BB")  
             
        
        
        
        
        if not problem.decomposedModel.onlyFeasCuts:
            fixed = model.fixed()
            fixed.optimize()

            currentSolution.piT_upper = []
            for i in transferSet:
                try:
                    cHA = fixed.getConstrByName("hubAssignment"+str(i))
                    lagrange = cHA.Pi
                    currentSolution.piT_upper.append([i, lagrange])
                                        
                    problem.nodes[i].available_saving_upper = max(0, lagrange) 
                except:
                    currentSolution.piT_upper.append([i,0])                 
            #currentSolution.piT = currentSolution.piT_upper
    return currentSolution     

def Solve_integral_synchronization_subproblem(problem, currentSolution): 
    model = currentSolution.model
    model.getVarByName("zObj").UB = problem.decomposedModel.bestSolution.objVal #+ 0.001
    model.setParam("OutputFlag", 0)
    
    model.optimize()

    currentSolution = get_optimal_solution(problem, currentSolution)
    currentSolution.foundInregral = True 
    
    currentSolution.model = [] 
    return currentSolution

def Subproblem_bounded(problem, currentSolution): #fixed routing and transfer set
    #check water level arc costs at line 780
    
    arcList = currentSolution.xij
    transferSet = currentSolution.vi
    #sets
    SetCd = []
    SetCd.append(0)
    SetCd.extend(problem.customers) 
    
    Customers = problem.customers
    Satellites = problem.satellites
    CDC = problem.CDC
    
    
    model = Model("2EMVRPTWSS")
    
## ----------- STREET PROBLEM-----------------------------------------
    #region
    # ---- Variables ----
    #street level
    
    mi = {} #load on car
    hi = {} #service start at wp
    vi = {} #transfer after node i
    nofCars = sum(1 for [i,j] in arcList if i == 0)
    
    #Define Variables
    for i in Customers:        
        hi[i] = model.addVar(lb=problem.nodes[i].earliest,ub=problem.nodes[i].latest, vtype=GRB.CONTINUOUS, 
                           name="h["+str(i)+"]")
        mi[i] = 0
    hi[0] = model.addVar(lb=problem.nodes[CDC].earliest,ub=problem.nodes[CDC].latest, vtype=GRB.CONTINUOUS, 
                           name="h["+str(0)+"]")
    
    
    #get the trip loads
    loads_trips = [] 
    for i in transferSet:
        problem.nodes[i].tripAssigned = i
        load_current =  problem.nodes[i].demand
        next_cust = i 
        incoming = next(j for [j,k] in arcList if k == next_cust)
        while  incoming != 0 and incoming not in transferSet:                    
            load_current += problem.nodes[incoming].demand
            problem.nodes[incoming].tripAssigned = i
            next_cust = incoming
            incoming = next(j for [j,k] in arcList if k == next_cust)
        mi[i] = load_current
        loads_trips.append(load_current)
          
    #service start time binding with depot times
    for i in Customers:
        model.addConstr(float(problem.nodes[0].earliest + problem.DistMatrix[0][i]) <= hi[i]) #5
        model.addConstr(problem.nodes[i].earliest<=hi[i] )
        model.addConstr(hi[i]<=problem.nodes[i].latest ) 
#------------------End of street level constraints------------------    
    #endregion
    #----------- SYNCHRONIZATION PROBLEM-----------------------------------------
    #region        
    vip = {} #trip decision 
    ui = {} #service start at transfer task
    detour = {} #amount of detour for street vehicles to visit a satellite
    
    for i in transferSet:
        ui[i] = model.addVar(lb=problem.nodes[CDC].earliest,ub=problem.nodes[CDC].latest, vtype=GRB.CONTINUOUS, 
                           name="u["+str(i)+"]")
        detour[i] = model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS, 
                           name="detour["+str(i)+"]")
        for p in Satellites:            
            vip[i,p] = model.addVar (vtype=GRB.BINARY,name="vip."+str(i)+"."+str(p)) #transfer decisions           

            

#---Part 2----- Synchronozation Constraints------
    #arc costs updated based on hub assignments
    for i in transferSet:
        model.addConstr(quicksum(vip[i,p] for p in Satellites)==1, name = "hubAssignment"+str(i)) #10 satellite assignment
       
    
    #transfer times bounds
    for i in transferSet:        
        model.addConstr(ui[i] >= hi[i] + problem.nodes[i].serviceTime + quicksum(problem.DistMatrix[i][p]*vip[i,p] for p in Satellites), name = "transferCustomerLower" + str(i)) #11 lower bound on transfer starts
        model.addConstr(problem.nodes[CDC].earliest + quicksum(problem.DistMatrix[CDC][p]* vip[i,p]  for p in Satellites) <= ui[i], name = "depotLower" + str(i)) #13 depot time windows lower starts 
        model.addConstr(problem.nodes[CDC].latest >= ui[i] + problem.TransferDuration + quicksum(problem.DistMatrix[p][CDC]* vip[i,p] for p in Satellites), name = "depotUpper" + str(i)) #13 depot time windows upper ends 
        
        j = next(j for [k,j] in currentSolution.xij if k==i)        
        model.addConstr(quicksum((problem.DistMatrix[i][p] +  problem.DistMatrix[p][j] )*vip[i,p]    for p in Satellites) <= detour[i], name = "detourC" + str(i)) #14 extra travel with satellite assignments
        
        if j!=0:
            maxTravel = max(problem.DistMatrix[i][p] + problem.DistMatrix[p][j] for p in Satellites)
            Mij = problem.nodes[i].latest + problem.nodes[i].serviceTime + problem.TransferDuration + maxTravel  - problem.nodes[j].earliest
            
            model.addConstr(ui[i] + problem.TransferDuration +  quicksum(problem.DistMatrix[p][j]*vip[i,p] for p in Satellites) 
                            <= hi[j] + Mij*(1 - quicksum(vip[i,p]for p in Satellites)), name = "nextDelayed" + str(i))     #12 arrival to next customer delayed by transfer time

    #service start times flowing
    for [i,j] in arcList:
        if i not in transferSet and j!=0:                  
            model.addConstr(hi[i] + problem.nodes[i].serviceTime +   problem.DistMatrix[i][j] <= hi[j], name = "timeDependency" + str(i) + str(j))   
#------------------End of synchronization constraints------------------    
    #endregion
    #-----------   WATER LEVEL PROBLEM-----------------------------------------
    #region
    y = {} #water route
    li = {} #load on the vessel
    arcWaterCost = {} #arc cost if traversed
    fij = {} #single transfer variable
    nofVessels = model.addVar(lb=0,ub=len(transferSet), vtype=GRB.INTEGER, name = "nofVessels")    
        
    
    setCw = []
    setCw.append(CDC)
    setCw.extend(transferSet) 
    for i in transferSet:
        li[i] = model.addVar(lb=problem.nodes[i].demand,ub=problem.VesselCapacity, vtype=GRB.CONTINUOUS, 
                           name="l["+str(i)+"]")
        for j in transferSet:
            if i==j:
                continue
            fij[i,j] = model.addVar(lb=0,ub=problem.nodes[CDC].latest, vtype=GRB.CONTINUOUS, name="fij["+str(i)+","+str(j)+"]") 
    
    
    
    for i in setCw:
        for j in setCw:
            if i!=j:
                if i ==CDC or j == CDC:                    
                    y[i,j] = model.addVar (vtype=GRB.BINARY, name="Y["+str(i)+","+str(j)+"]") # movements between transfer tasks
                                        
                else:                    
                    y[i,j] = model.addVar (vtype=GRB.BINARY, name="Y["+str(i)+","+str(j)+"]") # movements between transfer tasks
                    
                
                arcWaterCost[i,j] = model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS, 
                           name="arcCostW["+str(i)+"."+str(j)+"]")
                            

    
#---------------------water level constraints-----------------------    


    #routing
    #assignment const
    for i in transferSet:
        model.addConstr(quicksum(y[i,j] for j in setCw if i != j)==quicksum(vip[i,p]  for p in Satellites)) #11
        model.addConstr(quicksum(y[j,i] for j in setCw if i != j)== quicksum(vip[i,p]  for p in Satellites)) #12
   
    # inflow-outflow
    model.addConstr(quicksum((y[i,CDC] -y[CDC,i]) for i in transferSet)==0)

    #loads  /  fleet size /
    sumDemand = 0
    for i in Customers:
        sumDemand = sumDemand + problem.nodes[i].demand
    for i in transferSet:        
        model.addConstr(li[i] >= mi[i]) #15
        for j in transferSet:
            if i!= j:
                model.addConstr(li[j] - li[i]  >= mi[j] - problem.VesselCapacity*(1 - y[i,j])) #16

    
    model.addConstr(quicksum(y[i,CDC] for i in transferSet)<=nofVessels) #13
    
    #service start times for vessel routing
    for i in transferSet:
        for j in transferSet:
            if i!=j:
                model.addConstr(ui[i] + problem.TransferDuration + arcWaterCost[i,j]<= ui[j] 
                                + problem.nodes[CDC].latest - problem.nodes[CDC].latest*y[i,j]) #time delay in nodes
                
    
    #arc cost water level if traversed
    for i in transferSet:
        if i not in setCw:
            continue
        model.addConstr(arcWaterCost[i,CDC] >= quicksum(problem.DistMatrix[p][CDC]*(y[i,CDC] + vip[i,p] -1) for p in Satellites))
        model.addConstr(arcWaterCost[CDC,i] >= quicksum(problem.DistMatrix[CDC][p]*(y[CDC,i] + vip[i,p] -1) for p in Satellites))
        for p in Satellites:
            model.addConstr(arcWaterCost[i,CDC] >= problem.DistMatrix[p][CDC]*(y[i,CDC] + vip[i,p] -1))
            model.addConstr(arcWaterCost[CDC,i] >= problem.DistMatrix[CDC][p]*(y[CDC,i] + vip[i,p] -1))
            #continue
            for j in transferSet:
                if i!=j and j in setCw: 
                    for r in Satellites:
                            model.addConstr(arcWaterCost[i,j] >= problem.DistMatrix[p][r]*(y[i,j] + vip[i,p]+ vip[j,r] -2))
 
        for j in Customers:
            if i!=j and j in setCw:
                model.addConstr(arcWaterCost[i,j] >= quicksum(problem.DistMatrix[p][r]*(y[i,j] + vip[i,p]+ vip[j,r] -2) for p in Satellites for r in Satellites))
                
    exp3 = LinExpr()
    # ------------
    #single transfers at hubs ----
    if problem.singleTransfer:
        for i in transferSet:
            for j in transferSet:
                exp3.clear() #load on vessels
                if i<j:
                    model.addConstr(ui[i] - ui[j] <= fij[i,j])
                    model.addConstr(ui[j] - ui[i] <= fij[i,j])
                    model.addConstr(fij[j,i] == fij[i,j])
                    if i!=j:
                        for p in Satellites:
                            model.addConstr(fij[i,j] >= problem.TransferDuration*(vip[i,p]+ vip[j,p] -1))
    #endregion


    #-----------    Objective ---------------------------
    #region
    # Define objective components
    FleetStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetStreet") #number of cars 
    TravelStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelStreet") 
    FleetWater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetWater")
    TravelWater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelWater") 
    #fixed by the solution
    arcs_from_garage = [j for [i,j] in arcList if i==0 ]        
    arcs_transfers_with =  [[i,j] for [i,j] in arcList if i in transferSet]
    arcs_no_transfers = [[i,j] for [i,j] in arcList if i not in transferSet]
    
    fixed_cost = sum(problem.DistMatrix[i][j]*problem.travelcoststreet for [i,j] in arcs_no_transfers)
    min_extra = sum(min(problem.DistMatrix[i][p] + problem.DistMatrix[p][j] for p in Satellites)*problem.travelcoststreet for [i,j] in arcs_transfers_with)
    model.addConstr(len(arcs_from_garage)*problem.penaltyCars<= FleetStreet)
    model.addConstr(fixed_cost + quicksum( problem.travelcoststreet*detour[i] for i in transferSet )  <= TravelStreet)
    
    #water level
    model.addConstr(nofVessels*problem.penaltyVessels <= FleetWater)    
    exp3.clear() #water movements
    totalDemand = 0
    for i in setCw:
        totalDemand = totalDemand + problem.nodes[i].demand
        for j in setCw:
            if i!=j:
                exp3.addTerms(problem.travelcostwater, arcWaterCost[i,j])    
    model.addConstr(exp3<=TravelWater)


    #lower bounds
    #lower bound on estimated cost
   
    sumDemand = sum(problem.nodes[iix].demand for iix in Customers)
    lowerVessels = math.ceil (sumDemand / problem.VesselCapacity )
    currentSolution.lowerVessels = lowerVessels
    minTr = min(2*problem.DistMatrix[CDC][siix] for siix in Satellites)
    
    
    #model.addConstr(nofVessels>= (quicksum( mi[i]*vip[i,p] for i in transferSet for p in Satellites) / problem.VesselCapacity) )
    #model.addConstr(TravelWater>=problem.travelcostwater*minTr*(quicksum( mi[i]*vip[i,p] for i in transferSet for p in Satellites) / problem.VesselCapacity))
    model.addConstr(TravelWater>= problem.travelcostwater*minTr*lowerVessels )
    model.addConstr(nofVessels>= lowerVessels)
    min_water_expected = problem.travelcostwater*minTr*lowerVessels  + lowerVessels*problem.penaltyVessels
    #13
    
    
    #CHECK STATIONARY BARGE INTERMOVEMENTS!!! REMOVED HERE
    if problem.stationaryBarges: #stationary system, limiting the vessels go work at a chosen satellite
        z = {}
        for p in Satellites:
            z[p] = model.addVar (vtype=GRB.BINARY, name="z["+str(p)+"]") #barge location decisions
        nofBarges = math.ceil(totalDemand/problem.VesselCapacity)
        model.addConstr(nofVessels == nofBarges)
        model.addConstr(quicksum(z[p] for p in Satellites) ==nofBarges)
        for i in transferSet:
            for p in Satellites:
                model.addConstr(vip[i,p] <=z[p])
        for i in setCw:
            for j in setCw:
                if i!=j:
                    if i !=CDC and j != CDC:                    
                        model.addConstr(arcWaterCost[i,j] ==0)  # no movement is allowed from the satellite                 
    # Set the type of objective
    zObj=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="zObj")
    model.addConstr(FleetStreet + FleetWater + TravelStreet + TravelWater<=zObj)
    model.addConstr(FleetStreet + FleetWater + TravelStreet + TravelWater>=0)
    
    synch_cost=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="synch_cost") 
    model.addConstr(synch_cost >= quicksum( problem.travelcoststreet*detour[i] for i in transferSet ) + TravelWater +  FleetWater) 

    #bounding      
    if problem.decomposedModel.bestSolution.isFeasible:
        zObj.UB = problem.decomposedModel.bestSolution.objVal - 0.0001# + 0.0001
    model.setObjective(zObj,GRB.MINIMIZE)

    #model.setObjective(zObj,GRB.MINIMIZE)
    #-----------------END---------------------------
    #endregion
    
    #--------- SOLVE RELAXATION--------------------
    #region
    model.setParam("Presolve", 0)
    model.setParam("TimeLimit", 100)
    model.setParam('Outputflag', 0)   
    
    model.setParam("MIPGap", 0.00)
    model.update() 

    currentSolution.fixed_arcs_cost = fixed_cost + len(arcs_from_garage)*problem.penaltyCars

    fixed = model.copy()
    fixed.relax()
    modelVars = fixed.getVars()        
    for v in modelVars:
        if(v.VType!=GRB.CONTINUOUS):
            v.VType = GRB.CONTINUOUS 
    print("relax sub started")
    fixed.optimize()
    print("relax sub ended")
        
    #endregion

    
    #update model values
    #region
    feasible_exists = not (fixed.status == 3  or fixed.status == 4 or  fixed.status == 6)
    
    #3 inf 6 cutoff 4 unbd 4 inf/unb (maybe duality)
    try:
        objt = fixed.getObjective()
        objVal = objt.getValue()       
    except:
        feasible_exists = False

    if not feasible_exists:        
        currentSolution.timeFound = round( time.time() - problem.decomposedModel.startTime, 4)        
        problem.decomposedModel.inFeasibleSolutions.append(currentSolution)
        problem.decomposedModel.inf_cuts.append(currentSolution)
        currentSolution.isFeasible = False
        currentSolution.foundInregral = True
        return currentSolution 
    #get dual multipliers
    #region

    currentSolution.lowerFixed = round(fixed.getVarByName("zObj").x,4)    
    currentSolution.synch_cost = round(fixed.getVarByName("synch_cost").x,4)
    currentSolution.nofCars = nofCars

    if not problem.decomposedModel.onlyFeasCuts:
        #currentSolution.optimality_bound_bounds = calculate_bound(arcs_no_transfers, arcs_from_garage, objVal, problem)
        currentSolution.optimality_bound_max= 0#max(currentSolution.optimality_bound_bounds)

            
        problem.decomposedModel.feas_cuts.append(currentSolution)   
        currentSolution.min_extra = min_extra
        currentSolution.min_water_expected = min_water_expected
        currentSolution.detours_lower= []
        currentSolution.piA = []
        currentSolution.duals_modified = []
        currentSolution.piT = []
        for i in transferSet:
            try:
                cHA = fixed.getConstrByName("hubAssignment"+str(i))
                lagrange = cHA.Pi
                currentSolution.piT.append([i, lagrange])
                
                problem.nodes[i].available_saving = max(0, round(lagrange,2))
                

                detour_i = round(fixed.getVarByName("detour["+str(i)+"]").x,2)
                currentSolution.detours_lower.append([i, detour_i])
                
                j = next(j for [k,j] in currentSolution.xij if k==i)    
                extra_than_best = round(problem.travelcoststreet*(detour_i  - min(problem.DistMatrix[i][s] + problem.DistMatrix[s][j] - problem.DistMatrix[i][j] for s in Satellites)),2)    
                currentSolution.piA.append([i,j,extra_than_best])

                currentSolution.duals_modified.append([i, extra_than_best])            
            except:
                currentSolution.piT.append([i,0])  

        for i in Customers:
            if i not in transferSet:
                try:
                    problem.nodes[i].available_saving =  problem.nodes[problem.nodes[i].tripAssigned].available_saving
                except:
                    pass
    #endregion
    print(currentSolution.piT)
    currentSolution.isFeasible = True
    currentSolution.model = model    
    #endregion 

    return currentSolution
    

def calculate_bound(all_arcs_from_customers_no_transfers, all_arcs_from_garage, synch_cost_solution, problem):

    if not problem.decomposedModel.bestSolution.isFeasible:
        return [0]
    bounds = []
    for solution in problem.decomposedModel.feas_cuts:    
        arcs_transfers_duals = []
        for [i,rc] in solution.piT:
            j = next(j for [k,j] in solution.xij if k==i)
            arcs_transfers_duals.append([i,j,rc])
        #lower bound for given solution
        boundValue = (synch_cost_solution + sum(problem.travelcoststreet*problem.DistMatrix[i][j] for [i,j] in all_arcs_from_customers_no_transfers if [i,j] in solution.xij)
                        + sum((problem.travelcoststreet*problem.DistMatrix[0][j] + problem.penaltyCars) for j in all_arcs_from_garage if [0,j] in solution.xij))
        bounds.append(boundValue)
        
    max_boundValue = max(bounds)
    return bounds 

