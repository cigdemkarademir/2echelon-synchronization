from json import load, dump
import os
import io
import numpy
import math 
import time
from pathlib import Path
from os import walk
import sklearn.cluster

class Node:
    def __init__(self, t, index, xcor,ycor,demand,earliest,latest,serviceTime):
        self.type = t
        self.index = index
        self.xcor = xcor
        self.ycor = ycor
        self.demand = demand 
        self.earliest = earliest
        self.latest = latest
        self.serviceTime = serviceTime

def ReadSolomonInstance(instance_name, N):
    #based on Solomon test data for VRPTW stored in json files in data folder
    #Solomon, M. M. (1987). Algorithms for the vehicle routing and scheduling
    #problems with time window constraints. Operations research, 35(2), 254-265.
    
    projectPath = os.path.dirname(os.path.abspath(__file__))
    json_file = projectPath+"/data/json/"+instance_name

    with io.open(json_file, 'rt', encoding='utf-8', newline='') as file_object:
            instance = load(file_object)    
    
    nodes = []
    for i in range(N):
        ss = instance[f'customer_{i+1}']
        nodes.append(Node("C", (i+1), ss['coordinates']['x'], ss['coordinates']['y'], ss['demand'], ss['ready_time'],ss['due_time'],
                         ss['service_time']))  
    
    ss = instance['depart']
    nodes.insert(0,Node("D", 0, ss['coordinates']['x'], ss['coordinates']['y'], ss['demand'], ss['ready_time'],ss['due_time'],
                         ss['service_time'])) 
    distMatrix = instance['distance_matrix']
    Capacity = instance['vehicle_capacity']
    
    return nodes, distMatrix, Capacity

class instance:
    def __init__(self, name, nodes):
        self.name = name
        self.nodes = nodes
        self.satellites = []
        self.customers = []
        self.nofSatellites = 4
        self.VesselCapacity = 250
        self.CarCapacity = 50
        self.TransferDuration = 0  
        self.penaltyVessels = 1000
        self.penaltyCars = 1000
        self.travelcostwater = 1
        self.travelcoststreet = 1  
        self.problem_to_solve = "2eVRP" #default is our problem 
        self.twoechelon = True    # False for VRP and MVRP, True for 2e-VRP and 2e-LRP
        self.isBenchmark = False # False for the problems solved, True for locating satellites at the central as suggested by Grangier and many
        self.stationaryBarges = False #True for 2e-LRP only
        self.multitripsOnStreets = True  #True for: MVRP, 2e-VRP and 2e-LRP , False for VRP
        self.singleTransfer = True   #we assume it is true for all problems solved in the paper but if False, it removes one-to-one transfers at the satellites for alternatives 
        self.locate_by_kmeans = False #locating satellite by rule or kmeans            
        self.closeness = 0 #locating satellites for 2e-VRP default at the outskirts, 0.5 means locating all at the centre
        self.isDelivery = False  #if pickup there is no need to move customer time windows, only update vehicles time windows 
        self.report = "report.txt"
        self.stats = "stats.txt"        
        self.bestObjectve = 10000000
        self.bestFoundTime = 0
        self.modelStartTime = 0
        self.lowerFound = False
        self.lowerBoundRoot = 0
        self.arcList = []
        self.minVessels = 0
        self.closestHubtoCDC = 0
        self.optimal = False
        self.initial_solution = None
        
    def AddCDC(self):
    #The layout used is -50/150 for CDC
        #based on Grangier et al (2016) CDC definition
        #Grangier, P., Gendreau, M., Lehuédé, F., & Rousseau, L. M. (2016).
        #An adaptive large neighborhood search for the two-echelon multiple-trip
        #vehicle routing problem with satellite synchronization. European journal of operational research, 254(1), 80-91.
        minx = 1000000
        miny = 1000000
        maxx = -100000
        maxy = -100000

        totalDemand = 0
        sumService = 0
        countC = 0 
        for i in range(len(self.nodes)):
            current = self.nodes[i]
            if current.type !="C":
                continue
            self.customers.append(current.index)
            totalDemand = totalDemand + current.demand
            sumService = sumService + current.serviceTime
            countC +=1
            if current.xcor < minx:
                minx = current.xcor
            if current.xcor > maxx:
                maxx = current.xcor
            if current.ycor < miny:
                miny = current.ycor
            if current.ycor > maxy:
                maxy = current.ycor  
        average = sumService / countC
        self.TransferDuration = math.ceil( average * 2) #default transfer duration is constant and twice of the average customer service duration
        xcorCDC = minx - (float)((maxx - minx) / 2) # 50% left to the demand area
        ycorCDC = maxy + (float)((maxy - miny) / 2) # 50% up to the demand area -50/150 CDC location scheme used in Grangier et al.

        if self.isBenchmark:
            xcorCDC = minx + (float)((maxx - minx) / 2) # 50% right to the demand area, middle centre on x axis, closer to the city than our assumption
            self.TransferDuration = 0
            self.CarCapacity = self.benchmark_vehicle_capacity*(0.25)
            self.VesselCapacity = self.benchmark_vehicle_capacity*(2)

        self.nodes.append(Node("CDC", len(self.nodes), xcorCDC, ycorCDC, 0, self.nodes[0].earliest,
                                  self.nodes[0].latest, 0))
        self.CDC = len(self.nodes) - 1 #index of the CDC
        
        #add lower bound on the number of vessels
        lowerVessels = math.ceil (totalDemand / self.VesselCapacity )
        self.minVessels = lowerVessels #only lower bound on the number of vehicles based on the capacity
        
        
    def AddSatellites(self):
        
        minx = 1000000
        miny = 1000000
        maxx = -100000
        maxy = -100000
        xCoord = []
        yCoord = []
        for i in range(len(self.nodes)):
            current = self.nodes[i]
            if current.type == "C":
                xCoord.append(current.xcor)
                yCoord.append(current.ycor)
                if current.xcor < minx:
                    minx = current.xcor
                if current.xcor > maxx:
                    maxx = current.xcor
                if current.ycor < miny:
                    miny = current.ycor
                if current.ycor > maxy:
                    maxy = current.ycor 
        
        if not self.isBenchmark:
            if self.twoechelon:
                if self.locate_by_kmeans:
                    data = list(zip(xCoord, yCoord))
                    kmeans = sklearn.cluster.KMeans(n_clusters=4)
                    kmeans.fit(data)

                        
                    centroids = kmeans.cluster_centers_
                    self.centroids = centroids
                        
                    for iii in range(4):
                        self.nodes.append(Node("S", len(self.nodes), round(centroids[iii][0],2), round(centroids[iii][1],2), 0, self.nodes[0].earliest, self.nodes[0].latest, 0))
                        self.satellites.append(len(self.nodes) -1)
                        
                else: #default assumption in the paper
                    #locate staellites around the city/ at midpoints of the rectangle that contains all the demand nodes
                    listCoor = []
                    perSide = int (self.nofSatellites / 4)
                    xIncrement = (maxx - minx) / (perSide+1)
                    YIncrement = (maxy - miny) / (perSide+1)
                    for i in range(4): #number of sides
                        for p in range(perSide):
                            if i == 0 or i == 1:
                                xCor = minx + xIncrement*(p+1)
                                if i==0:
                                    yCor = miny + (maxy - miny)*self.closeness
                                else:
                                    yCor = maxy - (maxy - miny)*self.closeness
                            else:
                                yCor = miny + YIncrement*(p+1)
                                if i==2:
                                    xCor = minx + (maxx - minx)*self.closeness
                                else:
                                    xCor = maxx - (maxx - minx)*self.closeness
                            self.nodes.append(Node("S", len(self.nodes), xCor, yCor, 0, self.nodes[0].earliest,
                                            self.nodes[0].latest, 0))
                            self.satellites.append(len(self.nodes) -1)
                            listCoor.append([xCor,yCor])
            else:
                #locate satellites at the central depot (CDC), e.g. like work stations to perform transfers for vehicle trips
                for i in range(self.nofSatellites):
                    self.nodes.append(Node("S", len(self.nodes), self.nodes[self.CDC].xcor, self.nodes[self.CDC].ycor, 0, self.nodes[self.CDC].earliest,
                                          self.nodes[self.CDC].latest, 0))
                    self.satellites.append(len(self.nodes) -1)
        else:
            #satellites are located at the city center based on benchmark in the literature by Grangier et al
            #transfer durations or any handling time is assumed zero in Grangier paper/reference paper.
            #in total 8 satellites are located for 100 customers
            self.TransferDuration = 0
            xIncrement = (maxx - minx) / 4
            YIncrement = (maxy - miny) / 4

            for i in range(3):
                for p in range(3):
                    if i == 0 or i == 2:
                        xCor = minx + xIncrement*(p+1)
                        if i==0:
                            yCor = miny + YIncrement
                        else:
                            yCor = maxy - YIncrement
                    else:
                        if p==1:
                            continue
                        yCor = miny + YIncrement*2
                        xCor = minx + xIncrement*(p+1) 
                    self.nodes.append(Node("S", len(self.nodes), xCor, yCor, 0, self.nodes[0].earliest,
                                      self.nodes[0].latest, 0))
                    self.satellites.append(len(self.nodes) -1)  
        
        for node in self.nodes:
            if node.type == "S":
                node.serviceTime = self.TransferDuration
    def CreateDistMatrix(self): #use Euclidean distance to calculate distances
        self.DistMatrix = numpy.zeros( (len(self.nodes), len(self.nodes)) ) 
        for i in range(len(self.nodes)):
            for j in range(len(self.nodes)):
                ii = self.nodes[i]
                jj = self.nodes[j]
                cost =round( math.sqrt((ii.xcor - jj.xcor)*(ii.xcor - jj.xcor) + (ii.ycor - jj.ycor)*(ii.ycor - jj.ycor)), 2)
                self.DistMatrix[i][j] = cost
        
        for i in range(len(self.nodes)): #triangle inequality 
            for j in range(len(self.nodes)):
                for k in range(len(self.nodes)):
                    cc = self.DistMatrix[i][k] + self.DistMatrix[k][j] 
                    if cc<=self.DistMatrix[i][j]:
                        self.DistMatrix[i][j] = cc

    def UpdateTWs(self):
        if self.isDelivery: 
        #moving time windows to accomodate extra time needed to supply vehicles from CDC and vessels to return 
            addition = math.ceil(self.DistMatrix[self.CDC][0])  + self.TransferDuration    #the best is assumed that vessels bring goods to the depot and transfer them  
            for i in range(len(self.nodes)):
                current = self.nodes[i]
                if current.type == "C":                
                    current.latest = current.latest +  addition
                    current.earliest = current.earliest + addition

            #update depot, CDC and satellites for feasibly serving given demand        
            max_return = float(max(self.nodes[ii].latest +self.nodes[ii].serviceTime +self.DistMatrix[ii][pp] 
                                  +self.TransferDuration
                                  + self.DistMatrix[pp][self.CDC] for ii in self.customers for pp in self.satellites ))
            for i in range(len(self.nodes)):
                current = self.nodes[i]
                if current.type != "C":                
                    current.latest = max_return
                    
        else:  #if pickup there is no need to move customer time windows, only update vehicles time windows         
            max_return = float(max(self.nodes[ii].latest +self.nodes[ii].serviceTime +self.DistMatrix[ii][pp] 
                                  +self.TransferDuration
                                  + self.DistMatrix[pp][self.CDC] for ii in self.customers for pp in self.satellites ))            
        
            for i in range(len(self.nodes)):
                current = self.nodes[i]
                if current.type != "C":                
                    current.latest = max_return       
        

        self.closestHubtoCDC =  round(min( self.DistMatrix[self.CDC][p] + self.DistMatrix[p][self.CDC] for p in self.satellites),2)
        
        
    def GenerateProblem(self):        
        self.AddCDC()
        self.AddSatellites()
        self.CreateDistMatrix()
        self.UpdateTWs()

    
    def Set_cost_coefficients(self): # given reference values, the coefs are set according to fixed cost scenario
        self.fix_ref_scenario = self.update_rule[0] #large costs fixed at makespan
        self.normalized = self.update_rule[1] #relative ratios are normalized and multiplied with base costs  

        if self.costScenario ==0.1: # WLS = 1:10
            wcost = 1
            scost = 10       
                    
        elif self.costScenario ==0.2:  # WLS = 1:5
            wcost = 1
            scost = 5
            
        elif self.costScenario == 1: #WLS = 1:1
            wcost = 1
            scost = 1
            
        elif self.costScenario == 10: #WLS = 10:1
            wcost = 10
            scost = 1
        
        if self.normalized:
            WRS = round(wcost/(wcost+scost), 2)
            self.travelcostwater *= WRS
            self.travelcoststreet *= (1- WRS)
        else:
            WRS = round(wcost/scost,2) #used in the paper for the final results
            self.travelcostwater *= WRS

        if self.fix_ref_scenario : #fixed costs are very bog 
            self.penaltyVessels = self.nodes[0].latest       
            self.penaltyCars = self.nodes[0].latest
        else:
            fixed_vehicle_based_costs = float(round(max(self.travelcoststreet*(self.DistMatrix[0][c] + self.DistMatrix[c][p] + self.DistMatrix[p][0]) 
                                            + self.travelcostwater*(self.DistMatrix[self.CDC][p]  + self.DistMatrix[p][self.CDC])
                            for c in self.customers for p in self.satellites),2))
            
            self.penaltyVessels = fixed_vehicle_based_costs*WRS 
            self.penaltyCars = fixed_vehicle_based_costs 
        if not self.twoechelon:
            self.penaltyVessels = 0
            self.penaltyCars = 0
    
def readGenerate(instance_name, size, timeLimit, problem_type = None, closeness = 0):
    VRPTWinstance = ReadSolomonInstance(instance_name, size)
    problem = instance(instance_name, VRPTWinstance[0])
    problem.benchmark_vehicle_capacity = VRPTWinstance[2]
    problem.closeness = closeness #max 0.5 as all satellites are located exactly at the centre of the demand rectangle
    
    problem.problem_to_solve = problem_type

    #problem to solve: VRP, MVRP, 2e-VRP, 2e-LRP, benchmark(approximately Grangier)
    #single echelon only for VRP and MVRP   
    if problem_type == "VRP":
        problem.twoechelon = False   
        problem.isBenchmark = False        
        problem.stationaryBarges = False        
        problem.multitripsOnStreets = False  #only type without multi trips on streets
        problem.singleTransfer = True
        problem.CarCapacity = problem.VesselCapacity #increase it to truck size equal to vessel size

    elif problem_type == "MVRP":
        problem.twoechelon = False   
        problem.isBenchmark = False        
        problem.stationaryBarges = False        
        problem.multitripsOnStreets = True  #only type without multi trips on streets
        problem.singleTransfer = True

    #two-echelon only for 2e-VRP and 2e-LRP, benchmark
    elif problem_type == "2eLRP":
        problem.twoechelon = True   
        problem.isBenchmark = False        
        problem.stationaryBarges = True        
        problem.multitripsOnStreets = True  #only type without multi trips on streets
        problem.singleTransfer = True

    elif problem_type == "2eVRPBenchmark":
        problem.twoechelon = True   
        problem.isBenchmark = True        
        problem.stationaryBarges = False        
        problem.multitripsOnStreets = True  #only type without multi trips on streets
        problem.singleTransfer = False
        problem.TransferDuration = 0
    else:#ours
        problem.twoechelon = True   
        problem.isBenchmark = False        
        problem.stationaryBarges = False        
        problem.multitripsOnStreets = True  #only type without multi trips on streets
        problem.singleTransfer = True


    problem.GenerateProblem()
    
    problem.timeLimit = timeLimit
    return problem 


#WLS = scenario
#reported results are fix_ref and not normalized.

    

