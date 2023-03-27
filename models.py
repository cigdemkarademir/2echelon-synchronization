import os
import io
import numpy
import math 
from gurobipy import *
from gurobipy import GRB
import solution
import time

incumbentVal = math.inf
incumbentTime = time.time()

#if joint model will solve the problem, then do not set subproblem argument
#else set it True to return joint model to use as subproblem
def JointSynchronized(problem, subproblem = False): 
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
    # ---- Variables ----
    #street level
    xij = {} #street route
    mi = {} #load on car
    hi = {} #service start at wp
    arcStreetCost = {} #arc cost if traversed
    vi = {} #transfer after node i
    nofCars = model.addVar(lb=1,ub=len(Customers), vtype=GRB.CONTINUOUS, name = "nofCars")
    priorities = 1
    #Define Variables
    for i in Customers:
        vi[i] = model.addVar (vtype=GRB.BINARY, name="v."+str(i))

        hi[i] = model.addVar(lb=problem.nodes[i].earliest,ub=problem.nodes[i].latest, vtype=GRB.CONTINUOUS, 
                           name="h["+str(i)+"]")
        mi[i] = model.addVar(lb=problem.nodes[i].demand,ub=problem.CarCapacity, vtype=GRB.CONTINUOUS, 
                           name="m["+str(i)+"]")
    hi[0] = model.addVar(lb=problem.nodes[CDC].earliest,ub=problem.nodes[CDC].latest, vtype=GRB.CONTINUOUS, 
                           name="u["+str(0)+"]")
    for i in SetCd:
        for j in SetCd:
            if i==j:
                continue   
            if problem.nodes[i].earliest + problem.nodes[i].serviceTime + problem.DistMatrix[i][j]<=problem.nodes[j].latest:
                xij[i,j]=model.addVar (vtype=GRB.BINARY, name="x."+str(i)+"."+str(j))                
                arcList.append([i,j])
                maxCost = 0
                for p in Satellites:
                    cccc = problem.DistMatrix[i][p] + problem.DistMatrix[p][j] #upper bound on arcCost
                    if cccc > maxCost:
                        maxCost = cccc
                arcStreetCost[i,j] = model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS, 
                           name="arcCostS["+str(i)+"."+str(j)+"]")
    #---Part 1----- Street level Constraints 
    #assignment const
    for i in Customers:
        model.addConstr(quicksum(xij[i,j] for j in SetCd if [i,j] in arcList)==1, "outflow."+str(i)) #1
        model.addConstr(quicksum(xij[j,i] for j in SetCd if [j,i] in arcList)==1,"inflow."+str(i)) #2
    
    #fleet size and  #flow balance at the depot
    model.addConstr(quicksum(xij[i,0] for i in Customers) == nofCars) #3 
   
    exp3 = LinExpr()
    for i in Customers:
        exp3.addTerms(1,xij[i,0]) #3
        exp3.addTerms(-1,xij[0,i])#3
    model.addConstr(exp3==0)    
    
    #load on the cars #4
    for i in Customers:
        for j in Customers:
            if [i,j] in arcList:
                model.addConstr(mi[j] - mi[i] >= problem.nodes[j].demand - problem.CarCapacity*(1 - xij[i,j] +vi[i]))

                
    #service start time binding with depot times
    for i in Customers:
        model.addConstr(float(problem.nodes[0].earliest + problem.DistMatrix[0][i]) <= hi[i]) #5
        model.addConstr(problem.nodes[i].earliest<=hi[i] )
        model.addConstr(hi[i]<=problem.nodes[i].latest )
    
    #service start times flowing
    for i in Customers:
        for j in SetCd:
            if [i,j] in arcList:
                bigMT = problem.nodes[i].latest + problem.nodes[i].serviceTime + problem.DistMatrix[i][j]- problem.nodes[j].earliest
                model.addConstr(hi[i] + problem.nodes[i].serviceTime +  problem.DistMatrix[i][j] <= hi[j]
                                + bigMT - bigMT*xij[i,j])
                maxipj = 0
                for p in Satellites:
                    if problem.DistMatrix[i][p] + problem.DistMatrix[p][j] >= maxipj:
                        maxipj = problem.DistMatrix[i][p] + problem.DistMatrix[p][j]
                bigMtrans = problem.nodes[i].latest + problem.nodes[i].serviceTime + maxipj + problem.TransferDuration - problem.nodes[j].earliest
                model.addConstr(hi[i] + problem.nodes[i].serviceTime+problem.TransferDuration*vi[i]
                                +  arcStreetCost[i,j] <= hi[j]
                                + bigMtrans - bigMtrans*xij[i,j])
        
    #last transfer assignment before depot
    for i in Customers:
        model.addConstr(vi[i] >= xij[i,0]) #7

        #arc costs relaxed based on closest hub assignment
    for [i,j] in arcList:
        model.addConstr(arcStreetCost[i,j]>=problem.DistMatrix[i][j]*xij[i,j]) #objCoeff
        additionalMin = 1000000
        for p in Satellites:
            diff = max(0,problem.DistMatrix[i][p] +  problem.DistMatrix[p][j] - problem.DistMatrix[i][j])
            if(diff <=additionalMin):
                additionalMin = diff
        if i in Customers:
            model.addConstr(additionalMin*(xij[i,j] + vi[i]-1) + problem.DistMatrix[i][j]*xij[i,j] <=arcStreetCost[i,j]) #objCoeff


#------------------End of street level constraints------------------    
    #synchronization
    vip = {} #trip decision 
    ui = {} #service start at transfer task
    priorities +=1
    for i in Customers:
        ui[i] = model.addVar(lb=problem.nodes[CDC].earliest,ub=problem.nodes[CDC].latest, vtype=GRB.CONTINUOUS, 
                           name="u["+str(i)+"]")
        for p in Satellites:
            vip[i,p] = model.addVar (vtype=GRB.BINARY,name="vip."+str(i)+"."+str(p)) #transfer decisions
            
            


#---Part 2----- Synchronozation Constraints------
    #arc costs updated based on hub assignments
    for i in Customers:
        model.addConstr(quicksum(vip[i,p] for p in Satellites)==vi[i]) #8
        for j in SetCd:
            if [i,j] in arcList:
                for p in Satellites:
                    diff = problem.DistMatrix[i][p] +  problem.DistMatrix[p][j]
                    model.addConstr(diff*(xij[i,j] + vip[i,p] - 1) <=arcStreetCost[i,j]) #objCoeff
    
    #transfer times bounds
    for i in Customers:
        for p in Satellites:
            model.addConstr(ui[i] >= hi[i] + problem.nodes[i].serviceTime + problem.DistMatrix[i][p]*vip[i,p]) #9
            model.addConstr(problem.nodes[CDC].earliest + problem.DistMatrix[CDC][p]* vip[i,p] <= ui[i]) #16
            model.addConstr(problem.nodes[CDC].latest >= ui[i] + problem.TransferDuration + problem.DistMatrix[p][CDC]* vip[i,p]) #17
        for j in SetCd:
            if [i,j] in arcList:
                model.addConstr(quicksum((problem.DistMatrix[i][p] +  problem.DistMatrix[p][j])*(xij[i,j] + vip[i,p] - 1) for p in Satellites) <=arcStreetCost[i,j]) #objCoeff

                for p in Satellites:
                    model.addConstr(ui[i] + (problem.TransferDuration +  problem.DistMatrix[p][j])*vip[i,p] <= hi[j]
                                + problem.nodes[0].latest - problem.nodes[0].latest*xij[i,j]) #10

    
#------------------End of synchronization constraints------------------    
    #    WATER LEVEL
    y = {} #water route
    li = {} #load on the vessel
    arcWaterCost = {} #arc cost if traversed
    fij = {} #single transfer variable
    nofVessels = model.addVar(lb=1,ub=len(Customers), vtype=GRB.CONTINUOUS, name = "nofVessels")

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
    priorities+=1  
    maxCost = 0
    for p in Satellites:
        for r in Satellites:
            if problem.DistMatrix[p][r]>maxCost:
                maxCost = problem.DistMatrix[p][r]
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
    for i in Customers:
        model.addConstr(quicksum(y[i,j] for j in setCw if i != j)== vi[i]) #11
        model.addConstr(quicksum(y[j,i] for j in setCw if i != j)== vi[i]) #12
   
    # inflow-outflow
    model.addConstr(quicksum((y[i,CDC] -y[CDC,i]) for i in Customers)==0)

    #loads  /  fleet size /
    sumDemand = 0
    for i in Customers:
        sumDemand = sumDemand + problem.nodes[i].demand
        model.addConstr(li[i] >= mi[i]) #15
        for j in Customers:
            if i!= j:
                model.addConstr(li[j] - li[i]  >= mi[j] - problem.VesselCapacity*(1 - y[i,j])) #16

    lowerVessels = math.ceil (sumDemand / problem.VesselCapacity )
    model.addConstr(quicksum(y[i,CDC] for i in Customers)==nofVessels) #13
    model.addConstr(nofVessels>=lowerVessels) #13
    

    #service start times for vessel routing
    for i in Customers:
        for j in Customers:
            if i!=j:
                model.addConstr(ui[i] + problem.TransferDuration + arcWaterCost[i,j]<= ui[j] 
                                + problem.nodes[CDC].latest - problem.nodes[CDC].latest*y[i,j]) #time delay in nodes
                
    
    #arc cost water level if traversed
    for i in Customers:
        if i not in setCw:
            continue
        model.addConstr(arcWaterCost[i,CDC] >= quicksum(problem.DistMatrix[p][CDC]*(y[i,CDC] + vip[i,p] -1) for p in Satellites))
        model.addConstr(arcWaterCost[CDC,i] >= quicksum(problem.DistMatrix[CDC][p]*(y[CDC,i] + vip[i,p] -1) for p in Satellites))
        for p in Satellites:
            model.addConstr(arcWaterCost[i,CDC] >= problem.DistMatrix[p][CDC]*(y[i,CDC] + vip[i,p] -1))
            model.addConstr(arcWaterCost[CDC,i] >= problem.DistMatrix[CDC][p]*(y[CDC,i] + vip[i,p] -1))
            for j in Customers:
                if i!=j and j in setCw: 
                    for r in Satellites:
                            model.addConstr(arcWaterCost[i,j] >= problem.DistMatrix[p][r]*(y[i,j] + vip[i,p]+ vip[j,r] -2))
 
        for j in Customers:
            if i!=j and j in setCw:
                model.addConstr(arcWaterCost[i,j] >= quicksum(problem.DistMatrix[p][r]*(y[i,j] + vip[i,p]+ vip[j,r] -2) for p in Satellites for r in Satellites))
                

    # ------------
    #single transfers at hubs ----
    if problem.singleTransfer:
        for i in Customers:
            for j in Customers:
                exp3.clear() #load on vessels
                if i<j:
                    model.addConstr(ui[i] - ui[j] <= fij[i,j])
                    model.addConstr(ui[j] - ui[i] <= fij[i,j])
                    model.addConstr(fij[j,i] == fij[i,j])
                    if i!=j:
                        for p in Satellites:
                            model.addConstr(fij[i,j] >= problem.TransferDuration*(vip[i,p]+ vip[j,p] -1))

     #--------------------------------------
    #(A) Objectives
    #--------------------------------------
    # Define objective components
    FleetStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetStreet") #number of cars 
    TravelStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelStreet") 
    FleetWater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetWater")
    TravelWater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelWater") 
    
    model.addConstr(nofVessels*problem.penaltyVessels == FleetWater)
    model.addConstr(nofCars*problem.penaltyCars== FleetStreet)
    
    exp3.clear()
    for i in Customers:
        exp3.addTerms(1, xij[i,0])
        exp3.addTerms(-1, y[i,CDC])
    #model.addConstr(exp3>=0)
    
    exp3.clear() #street movements
    for i in SetCd:
        for j in SetCd:            
            if [i,j] in arcList:
                exp3.addTerms(problem.travelcoststreet, arcStreetCost[i,j])
    model.addConstr(exp3==TravelStreet)

    exp3.clear() #water movements
    totalDemand = 0
    for i in setCw:
        totalDemand = totalDemand + problem.nodes[i].demand
        for j in setCw:
            if i!=j:
                exp3.addTerms(problem.travelcostwater, arcWaterCost[i,j])
    
    model.addConstr(exp3==TravelWater)
    nofTransferLB = math.ceil(totalDemand/problem.CarCapacity)
    model.addConstr(quicksum(vi[i] for i in Customers) >= nofTransferLB)
    minTr = math.inf
    for s in Satellites:
        if problem.DistMatrix[CDC][s]<minTr:
            minTr = problem.DistMatrix[CDC][s]
        
    model.addConstr(TravelWater>=problem.travelcostwater*minTr*lowerVessels)

    #CHECK STATIONARY BARGE INTERMOVEMENTS!!! REMOVED HERE
    if problem.stationaryBarges:
        nofBarges = math.ceil(totalDemand/problem.VesselCapacity)
        model.addConstr(FleetWater==problem.penaltyVessels*nofBarges)
        model.addConstr(quicksum(z[p] for p in Satellites) ==nofBarges)
        for i in Customers:
            for p in Satellites:
                model.addConstr(vip[i,p] <=z[p])
                            
    # Set the type of objective
    zObj=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="zObj")
    model.addConstr(FleetStreet + FleetWater + TravelStreet + TravelWater<=zObj)
    model.addConstr(FleetStreet + FleetWater + TravelStreet + TravelWater>=0)
    model.setObjective(zObj,GRB.MINIMIZE)
    #-----------------END---------------------------
    #model.setParam("Threads", 4)
    #model.setParam('MIPGap', 0.03)
    spl = problem.name.split(".")
    #model.write("model"+spl[0]+".lp")
    # Run Gurobi
    if subproblem:            
        problem.decomposedModel.subProblem.routingVars.extend(arcList)        
        problem.decomposedModel.subProblem.transferVars.extend(Customers)
        model.update()
        return model
    else:
        model.setParam('OutputFlag', 0)
        model.setParam('Timelimit', problem.timeLimit)
    
        model.setParam("LazyConstraints", 1)

        problem.modelStartTime = time.time()
        
        global incumbentTime
        incumbentTime = round(time.time(),2)        
        model.optimize(callBackSolTime)
        incumbentTime =   round( incumbentTime - problem.modelStartTime, 2)  
    if model.status == GRB.Status.INFEASIBLE:
        model.computeIIS()
        spl = problem.name.split(".")
        model.write("model"+spl[0]+".ilp")
        file1 = open(problem.report,"a")
        file1.write(str(problem.name)+"\t"+str(len(problem.customers))+"\t"+"IntegratedMILP"+"\t"+ "Infeasible")
        file1.close()

        print("INFEASIBLEEEEE!!!!")        
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
        file1.write(str(problem.name)+"\t"+str(len(problem.customers))+"\t"+"IntegratedMILP"+"\t"+ "No solution found")
        file1.close()
        return
    if model.status != GRB.Status.INFEASIBLE: # If optimal solution is found	
        currentSolution = solution.Solution(False)
        
        objt = model.getObjective()
        objVal = round(objt.getValue(),2)  
        currentSolution.isFeasible = True
        currentSolution.objVal = objVal
        currentSolution.addedBound = False            
        currentSolution.nVessel = int(model.getVarByName("nofVessels").x+0.1)
        currentSolution.nCars = int(model.getVarByName("nofCars").x+0.1)
        currentSolution.travelWater = model.getVarByName("TravelWater").x
        currentSolution.fleetWater = model.getVarByName("FleetWater").x
        currentSolution.travelStreet = model.getVarByName("TravelStreet").x
        currentSolution.fleetStreet = model.getVarByName("FleetStreet").x
            
        file1 = open(problem.report,"a")
        nodecnt = model.NodeCount
        if problem.penaltyVessels == 0:
            file1.write(instanceNameSplit[0]+"\t"+str(len(problem.customers))+"\t"+"IntegratedMILPTwoIndex"+"\t"+str(round(objVal,2))+"\t"+str(round(model.MIPGap,2))+"\t"+str(round(model.Runtime,2))+"\t"+"IncumbentFoundTime"+"\t"+str(incumbentTime)
                        +"\t"+"Number of nodes explored"+"\t"+str(round(nodecnt,0))
                        +"\t"+"Number of solutions found"+"\t"+str(round(model.SolCount,0))+"\t"+"Number of simplex iters"+"\t"+str(round(model.IterCount,0))+"\n" )
            file1.close()
        else:   
            file1.write(instanceNameSplit[0]+"\t"+str(len(problem.customers))+"\t"+"IntegratedMILPTwoIndex"+"\t"+str(round(objVal,2))+"\t"+ str(round(FleetStreet.x,0))+"\t"+str(round(TravelStreet.x/problem.travelcoststreet,2))
                        +"\t"+ str(round(FleetWater.x,0))+ "\t"+ str(round(TravelWater.x/problem.travelcostwater,2))+"\t"+str(round(model.MIPGap,2))+"\t"+str(round(model.Runtime,2))+"\t"+"IncumbentFoundTime"+"\t"+str(incumbentTime)
                        +"\t"+"Number of nodes explored"+"\t"+str(round(nodecnt,0))
                        +"\t"+"Number of solutions found"+"\t"+str(round(model.SolCount,0))+"\t"+"Number of simplex iters"+"\t"+str(round(model.IterCount,0))+"\n" )
            file1.close()
        
        file4 = open(os.path.dirname(os.path.abspath(__file__)) +"/Results/" +instanceNameSplit[0]+"Nodes"+str(len(problem.customers))+"OptimalRoutesTwoIndexModelJoint.txt","w")
        file4.write(str(round(objVal,2))+"\n")
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
def callBackSolTime(model, where):
    if where == GRB.Callback.MIPSOL:
        cur_obj = model.cbGetSolution(model.getVarByName("zObj"))
        global incumbentVal
        if cur_obj < incumbentVal:
            incumbentVal = cur_obj
            global incumbentTime
            incumbentTime = round(time.time(),2)

def StreetModel(problem, isInitial= False): 
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
    # ---- Variables ----
    #street level
    xij = {} #street route
    mi = {} #load on car
    hi = {} #service start at wp
    arcStreetCost = {} #arc cost if traversed
    vi = {} #transfer after node i
    
    nofCars = model.addVar(lb=1,ub=len(Customers), vtype=GRB.CONTINUOUS, name = "nofCars")
    nofVessels = model.addVar(lb=1,ub=len(Customers), vtype=GRB.CONTINUOUS, name = "nofVessels")
    #Define Variables
    for i in Customers:       
        vi[i] = model.addVar (vtype=GRB.BINARY, name="v."+str(i))
                
        hi[i] = model.addVar(lb=problem.nodes[i].earliest,ub=problem.nodes[i].latest, vtype=GRB.CONTINUOUS, 
                           name="h["+str(i)+"]")
        mi[i] = model.addVar(lb=problem.nodes[i].demand,ub=problem.CarCapacity, vtype=GRB.CONTINUOUS, 
                           name="m["+str(i)+"]")
        
    hi[0] = model.addVar(lb=problem.nodes[CDC].earliest,ub=problem.nodes[CDC].latest, vtype=GRB.CONTINUOUS, 
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
                
    #---Part 1----- Street level Constraints 
    #assignment const
    for i in Customers:
        model.addConstr(quicksum(xij[i,j] for j in SetCd if [i,j] in arcList)==1, "outflow."+str(i)) #1
        model.addConstr(quicksum(xij[j,i] for j in SetCd if [j,i] in arcList)==1,"inflow."+str(i)) #2
    
    #fleet size and  #flow balance at the depot
    model.addConstr(quicksum(xij[0,i] for i in Customers if [0,i] in arcList) == nofCars) #3 
   
    exp3 = LinExpr()
    for i in Customers:
        if [i,0] in arcList:
            exp3.addTerms(1,xij[i,0]) #3
        if [0,i] in arcList:
            exp3.addTerms(-1,xij[0,i])#3
    model.addConstr(exp3==0)    
    
    #load on the cars #4
    for i in Customers:
        for j in Customers:
            if [i,j] in arcList:
                model.addConstr(mi[j] - mi[i] >= problem.nodes[j].demand - problem.CarCapacity*(1 - xij[i,j] +vi[i]))

                
    #service start time binding with depot times
    for i in Customers:
        model.addConstr(float(problem.nodes[0].earliest + problem.DistMatrix[0][i]) <= hi[i]) #5
        model.addConstr(problem.nodes[i].earliest<=hi[i] )
        model.addConstr(hi[i]<=problem.nodes[i].latest)
    
    #service start times flowing
    for i in Customers:
        for j in SetCd:
            if [i,j] in arcList:            
                bigMT = problem.nodes[i].latest + problem.nodes[i].serviceTime + problem.DistMatrix[i][j]- problem.nodes[j].earliest
                model.addConstr(hi[i] + problem.nodes[i].serviceTime +  problem.DistMatrix[i][j] <= hi[j]
                                + bigMT - bigMT*xij[i,j])
                maxipj = 0
                for p in Satellites:
                    if problem.DistMatrix[i][p] + problem.DistMatrix[p][j] >= maxipj:
                        maxipj = problem.DistMatrix[i][p] + problem.DistMatrix[p][j]
                bigMtrans = problem.nodes[i].latest + problem.nodes[i].serviceTime + maxipj + problem.TransferDuration - problem.nodes[j].earliest
                model.addConstr(hi[i] + problem.nodes[i].serviceTime+problem.TransferDuration*vi[i]
                                +  arcStreetCost[i,j] <= hi[j]
                                + bigMtrans - bigMtrans*xij[i,j])
        
        
    #last transfer assignment before depot
    for i in Customers:
        if [i,0] in arcList:
            model.addConstr(vi[i] >= xij[i,0])#7

        #arc costs relaxed based on closest hub assignment
    for [i,j] in arcList:
        model.addConstr(arcStreetCost[i,j]>=problem.DistMatrix[i][j]*xij[i,j] ) #objCoeff
       
        additionalMin = 1000000
        for p in Satellites:
            diff = problem.DistMatrix[i][p] +  problem.DistMatrix[p][j]
            if(diff <=additionalMin):
                additionalMin = diff
        if i in Customers:
            model.addConstr(additionalMin*(xij[i,j] + vi[i] - 1) <=arcStreetCost[i,j] ) #objCoeff
            model.addConstr(arcStreetCost[i,j]>= problem.DistMatrix[i][j]*(xij[i,j] - vi[i]) + additionalMin*(xij[i,j] + vi[i]-1) )
            
        additionalMin = 1000000
        for p in Satellites:
            diff = max(0,problem.DistMatrix[i][p] +  problem.DistMatrix[p][j] - problem.DistMatrix[i][j])
            if(diff <=additionalMin):
                additionalMin = diff
        if i in Customers:
            model.addConstr(additionalMin*(xij[i,j] + vi[i]-1) + problem.DistMatrix[i][j]*xij[i,j] <=arcStreetCost[i,j]) #objCoeff


    #--------------------------------------
    #(A) Objectives
    #--------------------------------------
    # Define objective components
    FleetStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetStreet") #number of cars 
    TravelStreet=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelStreet") 
    FleetWater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="FleetWater")
    TravelWater=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="TravelWater") 
    
       

    exp3.clear() #street movements
    totalDemand = 0
    for i in SetCd:
        totalDemand = totalDemand + problem.nodes[i].demand
        for j in SetCd:            
            if [i,j] in arcList:
                exp3.addTerms(problem.travelcoststreet, arcStreetCost[i,j])
    model.addConstr(exp3<=TravelStreet)
    
    nofTransferLB = math.ceil(totalDemand/problem.CarCapacity)
    model.addConstr(quicksum(vi[i] for i in Customers ) >= nofTransferLB)

    
    # add lower bounds on water cost considering perfect solution assigning all transfers to the closest hub
    lowerVessel = math.ceil(totalDemand/problem.VesselCapacity)
    
    
    model.addConstr(nofVessels >= lowerVessel)
    minTr = math.inf
    for s in Satellites:
        if problem.DistMatrix[CDC][s]<minTr:
            minTr = problem.DistMatrix[CDC][s]
        
    model.addConstr(TravelWater>=problem.travelcostwater*minTr*lowerVessel)
    
    
    model.addConstr(nofVessels*problem.penaltyVessels <= FleetWater)
    model.addConstr(nofCars*problem.penaltyCars<= FleetStreet)
    
    # Set the type of objective
    zObj=model.addVar(lb=0,ub=math.inf, vtype=GRB.CONTINUOUS,name="zObj")
    model.addConstr(FleetStreet + FleetWater + TravelStreet + TravelWater<=zObj)
    model.setObjective(zObj,GRB.MINIMIZE)  
    #-----------------END---------------------------
    model.setParam('OutputFlag', 0)
    model.setParam('Timelimit', problem.timeLimit)
    instanceNameSplit = (problem.name).split(".")
    spl = problem.name.split(".")    
    #model.write("model"+spl[0]+".lp")
    # Run Gurobi   
    
    print("11Decomposed model started")                    
    if isInitial:
        problem.decomposedModel.masterProblem.routingVars.extend(arcList)
        problem.decomposedModel.masterProblem.transferVars.extend(Customers) 
        return model
