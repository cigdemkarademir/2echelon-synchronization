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

def JointSynchronized(problem): 
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
    nofCars = model.addVar(lb=0,ub=len(Customers), vtype=GRB.INTEGER, name = "nofCars")

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
        #continue
        if i in Customers:
            minTravel = min(problem.DistMatrix[i][p] + problem.DistMatrix[p][j] - problem.DistMatrix[i][j] for p in Satellites)
            model.addConstr(arcStreetCost[i,j]>= problem.DistMatrix[i][j]*xij[i,j] + minTravel*(xij[i,j]+vi[i] - 1)) #objCoeff
        


#------------------End of street level constraints------------------   
    #endregion
         
    #Synchronization Problem----------------
    #region
    vip = {} #trip decision 
    ui = {} #service start at transfer task    
    for i in Customers:
        ui[i] = model.addVar(lb=problem.nodes[CDC].earliest,ub=problem.nodes[CDC].latest, vtype=GRB.CONTINUOUS, 
                           name="u["+str(i)+"]")
        for p in Satellites:
            vip[i,p] = model.addVar (vtype=GRB.BINARY,name="vip."+str(i)+"."+str(p)) #transfer decisions
            
            


#---Part 2----- Synchronozation Constraints------
    # Satellite assignments #Const 12
    for i in Customers:
        model.addConstr(quicksum(vip[i,p] for p in Satellites)==vi[i])        
    
    #transfer times bounds
    for i in Customers:        
        model.addConstr(ui[i] >= hi[i] + problem.nodes[i].serviceTime + quicksum(problem.DistMatrix[i][p]*vip[i,p] for p in Satellites)) #Const13
        model.addConstr(problem.nodes[CDC].earliest + quicksum(problem.DistMatrix[CDC][p]* vip[i,p] for p in Satellites) <= ui[i]) #Const15
        model.addConstr(problem.nodes[CDC].latest >= ui[i] +  quicksum((problem.TransferDuration + problem.DistMatrix[p][CDC])* vip[i,p] for p in Satellites)) #Const15
        for j in SetCd:
            if [i,j] in arcList:
                model.addConstr(quicksum((problem.DistMatrix[i][p] +  problem.DistMatrix[p][j])*(xij[i,j] + vip[i,p] - 1) for p in Satellites) <=arcStreetCost[i,j])
                
                maxTravel = max(problem.DistMatrix[i][p] + problem.DistMatrix[p][j] for p in Satellites)
                Mij = problem.nodes[i].latest + problem.nodes[i].serviceTime + problem.TransferDuration + maxTravel  - problem.nodes[j].earliest 
                model.addConstr(ui[i] + quicksum((problem.TransferDuration +  problem.DistMatrix[p][j])*vip[i,p] for p in Satellites) <= hi[j]
                                + Mij - Mij*xij[i,j]) #Const14
                
    
#------------------End of synchronization constraints------------------    
    #endregion
    
    #    WATER LEVEL Problem----------------------
    #region
    y = {} #water route
    li = {} #load on the vessel
    arcWaterCost = {} #arc cost if traversed
    fij = {} #single transfer variable
    nofVessels = model.addVar(lb=0,ub=len(Customers), vtype=GRB.INTEGER, name = "nofVessels")

    if problem.stationaryBarges:
        z = {}
        for p in Satellites:
            z[p] = model.addVar (vtype=GRB.BINARY, name="z["+str(p)+"]") #barge location decisions
    
    setCw = []
    setCw.append(CDC)
    setCw.extend(problem.customers) 
    for i in Customers:
        li[i] = model.addVar(lb=problem.nodes[i].demand,ub=problem.VesselCapacity, vtype=GRB.CONTINUOUS, 
                           name="l["+str(i)+"]")
        for j in Customers:
            if i==j:
                continue
            fij[i,j] = model.addVar(lb=0,ub=problem.nodes[CDC].latest, vtype=GRB.CONTINUOUS, name="fij["+str(i)+","+str(j)+"]") 
    
    for i in setCw:
        for j in setCw:
            if i!=j:                
                y[i,j] = model.addVar (vtype=GRB.BINARY, name="Y["+str(i)+","+str(j)+"]") # movements between transfer tasks                
                
                arcWaterCost[i,j] = model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS, 
                           name="arcCostW["+str(i)+"."+str(j)+"]")
                            

    
#---------------------water level constraints-----------------------    


    #routing
    #assignment const
    for i in Customers:
        model.addConstr(quicksum(y[i,j] for j in setCw if i != j)== vi[i]) #Const18
        model.addConstr(quicksum(y[j,i] for j in setCw if i != j)== vi[i]) #Const18
   
    # depot inflow-outflow
    model.addConstr(quicksum((y[i,CDC] -y[CDC,i]) for i in Customers)==0) #Const19

    #loads    
    for i in Customers:
        model.addConstr(li[i] >= mi[i]) #Const20
        for j in Customers:
            if i!= j:
                model.addConstr(li[j] - li[i]  >= mi[j] - problem.VesselCapacity*(1 - y[i,j])) ##Const21

    #fleet size and requirements
    sumDemand = sum(problem.nodes[i].demand for i in Customers)
    lowerVessels = math.ceil (sumDemand / problem.VesselCapacity )
    model.addConstr(quicksum(y[i,CDC] for i in Customers)==nofVessels)  
    model.addConstr(nofVessels>=lowerVessels) 
    

    #service start times for vessel routing
    for i in Customers:
        for j in Customers:
            if i!=j:
                MijWater = (problem.nodes[i].latest + problem.nodes[i].serviceTime + max(problem.DistMatrix[i][p] for p in Satellites) +  problem.TransferDuration 
                            +  max(problem.DistMatrix[p][r] for p in Satellites for r in Satellites) 
                            - (problem.nodes[j].earliest + problem.nodes[j].serviceTime +  min(problem.DistMatrix[j][p] for p in Satellites) ))
                model.addConstr(ui[i] + problem.TransferDuration + arcWaterCost[i,j]<= ui[j] 
                                + MijWater - MijWater*y[i,j]) #time delay in nodes #Const22
                
    
    #arc cost water level if traversed
    for i in Customers: 
        model.addConstr(arcWaterCost[CDC,i] >= quicksum(problem.DistMatrix[CDC][p]*(y[CDC,i] + vip[i,p] -1) for p in Satellites)) #Const23 
        model.addConstr(arcWaterCost[i,CDC] >= quicksum(problem.DistMatrix[p][CDC]*(y[i,CDC] + vip[i,p] -1) for p in Satellites)) #Const24
        for p in Satellites:
            for j in Customers:
                if i!=j: 
                    for r in Satellites:
                        if p!=r: 
                            model.addConstr(arcWaterCost[i,j] >= problem.DistMatrix[p][r]*(y[i,j] + vip[i,p]+ vip[j,r] -2)) #Const24  #the most expensive part of the formulation (N*N*P*P many constraints)
                

    # ------------
    #single transfers at hubs ----
    if problem.singleTransfer:
        for i in Customers:
            for j in Customers:
                if i<j:
                    model.addConstr(ui[i] - ui[j] <= fij[i,j])
                    model.addConstr(ui[j] - ui[i] <= fij[i,j])
                    model.addConstr(fij[j,i] == fij[i,j])
                    if i!=j:
                        for p in Satellites:
                            model.addConstr(fij[i,j] >= problem.TransferDuration*(vip[i,p]+ vip[j,p] -1)) #transfers must be separated by at least a transfer duration

     #--------------------------------------
    #endregion
                            
    #(A) Objectives------------------------
    #region
    #--------------------------------------
    # Define objective components
    FleetStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetStreet") #number of cars 
    TravelStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelStreet") 
    FleetWater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetWater")
    TravelWater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelWater") 
    
    model.addConstr(nofVessels*problem.penaltyVessels == FleetWater)
    model.addConstr(nofCars*problem.penaltyCars== FleetStreet)   
    
    exp3 = LinExpr()#street movements
    for i in SetCd:
        for j in SetCd:            
            if [i,j] in arcList:
                exp3.addTerms(problem.travelcoststreet, arcStreetCost[i,j])
    model.addConstr(exp3==TravelStreet)

    exp3.clear() #water movements
    for i in setCw:
        for j in setCw:
            if i!=j:
                exp3.addTerms(problem.travelcostwater, arcWaterCost[i,j])    
    model.addConstr(exp3==TravelWater)
    
    #minimum bound on travel time
    minTr = math.inf
    for s in Satellites:
        if problem.DistMatrix[CDC][s]<minTr:
            minTr = problem.DistMatrix[CDC][s]
        
    model.addConstr(TravelWater>=problem.travelcostwater*minTr*lowerVessels)

    #CHECK STATIONARY BARGE INTERMOVEMENTS!!! REMOVED HERE
    if problem.stationaryBarges: # 2E-LRP : fixed the number of facilities to open
        nofBarges = math.ceil(sumDemand/problem.VesselCapacity)
        model.addConstr(FleetWater==problem.penaltyVessels*nofBarges)
        model.addConstr(quicksum(z[p] for p in Satellites) <=nofBarges)
        for i in Customers:
            for p in Satellites:
                model.addConstr(vip[i,p] <=z[p])
        for i in setCw:
            for j in setCw:
                if i!=j:
                    if i !=CDC and j != CDC:                    
                        model.addConstr(arcWaterCost[i,j] == 0)

                            
    # Set the type of objective
    zObj=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="zObj")
    model.addConstr(FleetStreet + FleetWater + TravelStreet + TravelWater<=zObj)
    model.addConstr(FleetStreet + FleetWater + TravelStreet + TravelWater>=0)
    model.setObjective(zObj,GRB.MINIMIZE)
    #-----------------END of the LP---------------------------
    #endregion

    spl = problem.name.split(".")
    problem.name = spl[0]
    #model.write("model"+spl[0]+".lp")
    # Run Gurobi
    
    model.setParam('OutputFlag', 0)
    model.setParam('Timelimit', problem.timeLimit)
    model.setParam("LazyConstraints", 1)

    problem.modelStartTime = time.time()    
    
    incumbentTime = round(time.time(),2)  
    model._incumbenttime = 0
    model._incumbentValue = math.inf
    
    try:
        model.optimize(callBackSolTime)
    except:
        print("model solve stopped")
    incumbentTime =   model._incumbenttime
    if model.status == GRB.Status.INFEASIBLE:
        model.computeIIS()
        spl = problem.name.split(".")
        model.write("model"+spl[0]+".ilp")
        file1 = open(problem.report,"a")
        file1.write(str(problem.name)+"\t"+str(len(problem.customers))+"\t"+"IntegratedMILP"+"\t"+ "Infeasible")
        file1.close()

        print(" Infeasible model check the printed minimum infeasible constraint set!!!!")        
        return
    try:
    
        #-----------------------------------------------------------------------------
        # --- Print results ---
        #-----------------------------------------------------------------------------
        objt = model.getObjective()
        objVal = round(objt.getValue(),2)  

        instanceNameSplit = (problem.name).split(".")
    except:
        file1 = open(problem.report,"a")
        file1.write(str(problem.name)+"\t"+str(len(problem.customers)) +"\t"+"IntegratedMILP No feasible solution found"+ "\t"+ "Model status"+ "\t"+ str(model.status))
        file1.close()
        return
    if model.status != GRB.Status.INFEASIBLE: # If optimal solution is found	
        currentSolution = solution.Solution(False)
        
        objt = model.getObjective()
        objVal = round(objt.getValue(),2)  
        currentSolution.isFeasible = True
        currentSolution.objVal = objVal           
        currentSolution.nVessel = int(model.getVarByName("nofVessels").x+0.1)
        currentSolution.nCars = int(model.getVarByName("nofCars").x+0.1)
        currentSolution.travelWater = model.getVarByName("TravelWater").x
        currentSolution.fleetWater = model.getVarByName("FleetWater").x
        currentSolution.travelStreet = model.getVarByName("TravelStreet").x
        currentSolution.fleetStreet = model.getVarByName("FleetStreet").x
            
        file1 = open(problem.report,"a")
        nodecnt = model.NodeCount
        if problem.penaltyVessels == 0:
            file1.write(instanceNameSplit[0]+"\t"+str(len(problem.customers))+"\t"+str(problem.costScenario)+"\t"+str(problem.fix_ref_scenario)+"\t"+str(problem.normalized)+"\t"+"IntegratedMILPTwoIndex"+"\t"+str(round(objVal,2))+"\t"+str(round(model.MIPGap,2))+"\t"+str(model.status)+"\t"+str(round(model.Runtime,2))+"\t"+"IncumbentFoundTime"+"\t"+str(incumbentTime)
                        +"\t"+"Number of nodes explored"+"\t"+str(round(nodecnt,0))
                        +"\t"+"Number of solutions found"+"\t"+str(round(model.SolCount,0))+"\t"+"Number of simplex iters"+"\t"+str(round(model.IterCount,0))+"\n" )
            file1.close()
        else:   
            file1.write(instanceNameSplit[0]+"\t"+str(len(problem.customers))+"\t"+str(problem.costScenario)+"\t"+str(problem.fix_ref_scenario)+"\t"+str(problem.normalized)+"\t"+"IntegratedMILPTwoIndex"+"\t"+str(round(objVal,2))+"\t"+ str(round(FleetStreet.x,0))+"\t"+str(round(TravelStreet.x/problem.travelcoststreet,2))
                        +"\t"+ str(round(FleetWater.x,0))+ "\t"+ str(round(TravelWater.x/problem.travelcostwater,2))+"\t"+str(round(model.MIPGap,2))+"\t"+str(model.status)+"\t"+str(round(model.Runtime,2))+"\t"+"IncumbentFoundTime"+"\t"+str(incumbentTime)
                        +"\t"+"Number of nodes explored"+"\t"+str(round(nodecnt,0))
                        +"\t"+"Number of solutions found"+"\t"+str(round(model.SolCount,0))+"\t"+"Number of simplex iters"+"\t"+str(round(model.IterCount,0))+"\n" )
            file1.close()
        
        file4 = open(os.path.dirname(os.path.abspath(__file__)) +"/ResultsJoint/" +instanceNameSplit[0]+"Nodes"+str(len(problem.customers))+str(problem.costScenario)+str(problem.fix_ref_scenario)+str(problem.normalized)+str(int(objVal))+"OptimalRoutesTwoIndexModelJoint.txt","w")
        file4.write(str(round(objVal,2))+"\n")
        file4.write(instanceNameSplit[0]+"\t"+str(len(problem.customers))+"\t"+str(problem.costScenario)+"\t"+str(problem.fix_ref_scenario)+"\t"+str(problem.normalized)+"\t"+"IntegratedMILPTwoIndex"+"\t"+str(round(objVal,2))+"\t"+str(round(model.MIPGap,2))+"\t"+str(model.status)+"\t"+str(round(model.Runtime,2))+"\t"+"IncumbentFoundTime"+"\t"+str(incumbentTime)
                        +"\t"+"Number of nodes explored"+"\t"+str(round(nodecnt,0))
                        +"\t"+"Number of solutions found"+"\t"+str(round(model.SolCount,0))+"\t"+"Number of simplex iters"+"\t"+str(round(model.IterCount,0))+"\n" )
        costStreet = 0
        usedArcs = []
        transferPoints = []
        hubAssignments = []
        for [i,j] in arcList: # print all variables that are greater than 1	            
            if i!=j:
                if (xij[i,j].x > 0.01): 	
                    usedArcs.append([i,j])
                    costStreet +=arcStreetCost[i,j].x
                    file4.write('%s %g ' % (xij[i,j].varName, xij[i,j].x)+"\t")	
                    file4.write('%s %g ' % (arcStreetCost[i,j].varName, arcStreetCost[i,j].x)+"\t")	
                    if i!=0:
                        file4.write('%s %g' % ( mi[i].varName, mi[i].x)+"\t")	
                        file4.write('%s %g' % ( hi[i].varName, hi[i].x)+"\t")	
                        for p in Satellites:
                            if (vip[i,p].x > 0.01):	
                                transferPoints.append(i)
                                hubAssignments.append(p)
                                file4.write('%s %g' % (vip[i,p].varName, vip[i,p].x)+"\t")	
                                file4.write('%s %g' % (ui[i].varName, ui[i].x)+"\t")
                                file4.write('%s %g' % (li[i].varName, li[i].x)+"\t")
                    file4.write("\n")
        currentSolution.xij = usedArcs
        currentSolution.vi = transferPoints
        currentSolution.hubs = hubAssignments
        
        file4.write("\n")
        costWater = 0
        for i in setCw:
            for j in setCw:
                if i!=j:
                    if (y[i,j].x > 0.01): 	
                            file4.write('%s %g ' % (y[i,j].varName, y[i,j].x)+"\t")	
                            if i==CDC :
                                costWater += problem.DistMatrix[CDC][hubAssignments[transferPoints.index(j)]]
                            elif j==CDC:
                                costWater += problem.DistMatrix[hubAssignments[transferPoints.index(i)]][CDC]
                            else:
                                costWater += problem.DistMatrix[hubAssignments[transferPoints.index(i)]][hubAssignments[transferPoints.index(j)]]

        file4.write("\n")
        file4.write("Street travel"+"\t"+ str(round(costStreet,2)) + "\t"+"Water travel"+"\t"+ str(round(costWater,2)))
        file4.close()
        currentSolution.ReportStatisticsSolution(problem, costStreet, costWater)
        return currentSolution
    
def callBackSolTime(model, where):
    if where == GRB.Callback.MIPSOL:
        cur_obj = model.cbGetSolution(model.getVarByName("zObj"))        
        if cur_obj < model._incumbentValue - 0.3:
            model._incumbentValue = cur_obj
            model._incumbenttime = round(model.cbGet(GRB.Callback.RUNTIME),1)
       
