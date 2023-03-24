from json import load, dump
import os
import io
import numpy
import math 
import pandas
import time
from pathlib import Path
from os import walk


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
    def __init__(self, name, nodes,decomposedModel=None):
        self.name = name
        self.nodes = nodes
        self.satellites = []
        self.customers = []
        self.nofSatellites = 4
        self.VesselCapacity = 250
        self.CarCapacity = 50
        self.TransferDuration = 150  
        self.penaltyVessels = 1000
        self.penaltyCars = 1000
        self.travelcostwater = 1
        self.travelcoststreet = 1
        self.singleTransfer = True
        self.stationaryBarges = False
        self.multitripsOnStreets = False
        self.report = "report.txt"
        self.stats = "stats.txt"
        self.isBenchmark = False
        self.twoechelon = True
        self.isDelivery = False
        self.bestObjectve = 10000000
        self.bestFoundTime = 0
        self.modelStartTime = 0
        self.lowerFound = False
        self.lowerBoundRoot = 0
        self.arcList = []
        self.minVessels = 0
        self.closestHubtoCDC = 0
        self.optimal = False
        self.nofRemoval = 3
        
    def AddCDC(self):
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
        self.TransferDuration = math.ceil( average * 2)
        xcorCDC = minx - (float)((maxx - minx) / 2)
        ycorCDC = maxy + (float)((maxy - miny) / 2)

        self.nodes.append(Node("CDC", len(self.nodes), xcorCDC, ycorCDC, 0, self.nodes[0].earliest,
                                  self.nodes[0].latest, 0))
        self.CDC = len(self.nodes) - 1
        
        #add lower bound on the number of vessels
        lowerVessels = math.ceil (totalDemand / self.VesselCapacity )
        self.minVessels = lowerVessels
        
    def AddSatellites(self):
        
        minx = 1000000
        miny = 1000000
        maxx = -100000
        maxy = -100000

        for i in range(len(self.nodes)):
            current = self.nodes[i]
            if current.type == "C":
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
                #locate staellites around the city/ at midpoints of the rectangle that contains all the demand nodes
                perSide = int (self.nofSatellites / 4)
                xIncrement = (maxx - minx) / (perSide+1)
                YIncrement = (maxy - miny) / (perSide+1)
                for i in range(4):
                    for p in range(perSide):
                        if i == 0 or i == 1:
                            xCor = minx + xIncrement*(p+1)
                            if i==0:
                                yCor = miny
                            else:
                                yCor = maxy
                        else:
                            yCor = miny + YIncrement*(p+1)
                            if i==2:
                                xCor = minx
                            else:
                                xCor = maxx
                        self.nodes.append(Node("S", len(self.nodes), xCor, yCor, 0, self.nodes[0].earliest,
                                          self.nodes[0].latest, 0))
                        self.satellites.append(len(self.nodes) -1)
            else:
                #locate satellites at the central depot (CDC), e.g. like work stations to perform transfers for vehicle trips
                for i in range(4):
                    self.nodes.append(Node("S", len(self.nodes), self.nodes[self.CDC].xcor, self.nodes[self.CDC].ycor, 0, self.nodes[self.CDC].earliest,
                                          self.nodes[self.CDC].latest, 0))
                    self.satellites.append(len(self.nodes) -1)
        else:
            #satellites are located at the city center based on benchmark in the literature by Grangier et al
            #transfer durations or any handling time is assumed zero in Grangier paper/reference paper.
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
        
    def CreateDistMatrix(self): 
        self.DistMatrix = numpy.zeros( (len(self.nodes), len(self.nodes)) )
        for i in range(len(self.nodes)):
            for j in range(len(self.nodes)):
                ii = self.nodes[i]
                jj = self.nodes[j]
                cost =round( math.sqrt((ii.xcor - jj.xcor)*(ii.xcor - jj.xcor) + (ii.ycor - jj.ycor)*(ii.ycor - jj.ycor)),2)
                self.DistMatrix[i][j] = cost
        
        for i in range(len(self.nodes)): #triangle inequality 
            for j in range(len(self.nodes)):
                for k in range(len(self.nodes)):
                    cc = self.DistMatrix[i][k] + self.DistMatrix[k][j] 
                    if cc<=self.DistMatrix[i][j]:
                        self.DistMatrix[i][j] = cc

    def UpdateTWs(self):
        #moving time windows to accomodate extra time needed to supply vehicles from CDC
        addition = math.ceil(self.DistMatrix[0][self.CDC])        
        for i in range(len(self.nodes)):
            current = self.nodes[i]
            if current.type == "C":                
                current.latest = current.latest +  addition
                current.earliest = current.earliest + addition
        
        maxipt = 0
        offset = 0
        for i in self.customers:
            for p in self.satellites:
                maxxO = math.ceil(self.nodes[i].latest +self.nodes[i].serviceTime +self.DistMatrix[i][p] 
                                  +self.TransferDuration
                                  + self.DistMatrix[p][self.CDC])
                if maxxO>=offset:
                    offset = maxxO
                
                tttt = math.ceil(self.DistMatrix[0][i] +self.DistMatrix[i][p]+self.DistMatrix[p][0])
                if tttt>= maxipt:
                    maxipt = tttt
       
        for i in range(len(self.nodes)):
            current = self.nodes[i]
            if current.type != "C":                
                current.latest = max(current.latest + addition, offset)
        
        #cost parameters' reference values 
        #create arc list and related costs
        for i in range(len(self.nodes)):
            for j in range(len(self.nodes)):
                if i != j and self.nodes[i].type != "S" and self.nodes[j].type != "S" and self.nodes[i].type != "CDC" and self.nodes[j].type != "CDC":
                    if self.nodes[i].earliest + self.nodes[i].serviceTime + self.DistMatrix[i][j]<=self.nodes[j].latest:
                        additionalMin = 1000000
                        for p in self.satellites:
                            diff = self.DistMatrix[i][p] +  self.DistMatrix[p][j]
                            if(diff <=additionalMin):
                                additionalMin = diff

                        
                        self.arcList.append([[i,j], additionalMin])

        minDistCdc = math.inf
        maxDistCdc = 0
        for p in self.satellites:
            trav = self.DistMatrix[self.CDC][p] + self.DistMatrix[p][self.CDC]
            if trav < minDistCdc:
                minDistCdc = trav
            if trav >= maxDistCdc:
                maxDistCdc = trav

        self.closestHubtoCDC = minDistCdc
        
        self.penaltyVessels = maxDistCdc
        self.travelcostwater = 1
        self.penaltyCars = maxipt
        self.travelcoststreet = 1
        
    def GenerateProblem(self):
        self.AddCDC()
        self.AddSatellites()
        self.CreateDistMatrix()
        self.UpdateTWs()
    
def readGenerate(instance_name, nofWP):
    VRPTWinstance = ReadSolomonInstance(instance_name, nofWP)
    problem = instance(instance_name, VRPTWinstance[0])
    problem.GenerateProblem()
    return problem 


