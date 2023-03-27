import os
import io
import numpy
import math 

class Solution:
    def __init__(self, IsFeasible):
        self.isFeasible = IsFeasible 
        self.index = 0 #the order of the solution found in BB process
        self.objVal = 100000000
        self.nVessel = 0
        self.nCars = 0
        self.travelWater = 0
        self.travelStreet = 0
        self.fleetWater = 0
        self.fleetStreet = 0
        self.xij = [] #list of used arcs
        self.yij = [] #list of used arcs
        self.vi = [] #list of transfer task
        self.hubs = []
        self.variables = []
        self.values = []
        self.timeFound = 0.000
        self.addedBound = False
        self.usedAsSolution = False
        self.mipGap = 0.000000
        self.lowerBound = 0
        self.lowerBoundstreet = 0
        self.streetSequence = []
        self.waterSequence = []                
        self.waterLowerBound = 0
        self.lowerBoundRoot = 0
        self.cuttOFF = False
        self.isBest = False
        self.foundBefore = False
        self.foundTransferSet = False
        self.foundInregral = False
        self.dualFeas = False
        self.reducedVars = []
        self.arcCostS = []
        self.reducedCosts = []
        self.piA = -1
        self.piT = -1
        self.ts = False
        self.solvedRouting = False
        self.optimizedHubs = False
    def UpdateSolution(problem):
        return

    def ReportStatisticsSolution(self, problem, street, water=0):
        usedArcs = self.xij
        transferPoints = self.vi
        hubs = self.hubs
        objVal = self.objVal
        nofVessels = self.nVessel
        
        file4 = open(problem.stats,"a")
        if len(usedArcs) == 0: #no feasible solution found
            file4.write("No feasible solution found"+"\n")
            file4.close()
            return
        #find routes
        routes = []
        indexRoute = 0
        for pair in usedArcs:
            if pair[0]==0:
                foundBefore = False
                for r in routes:
                    if r[0] == pair [1]:
                        foundBefore = True
                        break
                if not foundBefore:
                    routes.append([pair[1]])
        for k in range(len(routes)):
            lastnode = routes[k][0]
            completed = False
            while not completed:
                for pair in usedArcs:
                    if pair[0]==lastnode:
                        if pair[1] != 0:
                            routes[k].append(pair[1])
                            lastnode = pair[1]
                        else:
                            completed = True
        
        cost = objVal
        streetCost = 0
        weightedLoad = 0
        loadedTravelTime = 0
        totalTransferLoad = 0
        currentLoad = 0
        nofCars = len(routes)
        nofTransfers = len(transferPoints)
        totalDemand = 0
        
        for r in routes:
            print(r)
            i = r[0]
            streetCost += problem.DistMatrix[0][i]
            for s in range(len(r)):
                i = r[s]
                currentLoad +=problem.nodes[i].demand
                totalDemand+=problem.nodes[i].demand
                if s == len(r) - 1 :
                    j = 0
                else:
                    j = r[s+1]
                    
                if i in transferPoints:
                    hub = hubs[transferPoints.index(i)]
                    weightedLoad+=currentLoad*problem.DistMatrix[i][hub]
                    loadedTravelTime+= problem.DistMatrix[i][hub]
                    streetCost += problem.DistMatrix[i][hub] + problem.DistMatrix[hub][j]
                    totalTransferLoad += currentLoad
                    currentLoad = 0
                else:
                    weightedLoad+=currentLoad*problem.DistMatrix[i][j]
                    loadedTravelTime+= problem.DistMatrix[i][j]
                    streetCost += problem.DistMatrix[i][j]

                
            
        minimumNofTransfers = math.ceil(totalDemand/problem.CarCapacity)
        averageTransferLoad = round(100*((totalTransferLoad/nofTransfers)/problem.CarCapacity),2)
        averagePercentLoadCars = round(100*((weightedLoad/loadedTravelTime)/problem.CarCapacity),2)
        averageLoadCars = round((weightedLoad/loadedTravelTime),2)
        modalShare = round(100*(streetCost/(street+water)),2)
        
        nofHubsUsed = numpy.unique(hubs)
        
        file4.write(str(problem.name)+"\t"+str(len(problem.customers))+"\t"+"StreetLevel nofCars"+"\t"+str(nofCars)+"\t"
                    +"Street Travel Time"+"\t"+str(round(streetCost,2))+"\t"+str(round(street,2))+"\t"+"AverageLoaded"+"\t"+str(averageLoadCars)+"\t"
                    +"Average PercentLoad"+"\t"+str(averagePercentLoadCars)+"\t"+"ModalShare"+"\t"+str(modalShare)+"\t"
                    +"Minimum required nofTransfers"+"\t"+str(minimumNofTransfers)+"\t"+"nofTransfersOptimal"+"\t"+str(nofTransfers)+"\t"
                    +"Available nof Hubs"+"\t"+str(problem.nofSatellites)+"\t"+"nof Used Hubs" +"\t"+str(len(nofHubsUsed))                
                    +"\t"+"Average Transfer Load"+"\t"+str(averageTransferLoad)+"\t"
                    +"WaterLevel nofVessels"+"\t"+str(nofVessels)+"\t"+"WaterTravelTime"+"\t"+str(round(water,2))+"\n")
        file4.close()
