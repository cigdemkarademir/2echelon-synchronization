import instanceGenerator
import solution
import SEVs as sev
from json import load, dump
import os
import io
import numpy as np
import math 
import sys 
import random
import time

BigNumber = 100000
def InitialSolution_Heuristic(problem):
    problem.sequential = False  
    problem.Construct_CW = True 
    problem.type_objective = "makespan"   
    
    problem.initial_solution = construction_CW(problem)
    return problem.initial_solution

def best_CW_insertion(problem, route1, route2):
    newRoute = sev.street_vehicle()
    newRoute.sequence.extend(route1.sequence)
    newRoute.sequence.extend(route2.sequence)    
    newRoute.feas_check(problem)

    if newRoute.isFeasible:#
        calculate_saving(problem, route1, route2, newRoute)        

    return newRoute

def best_insertion_move(problem, route1, route2):
    bestRoute = sev.street_vehicle() 
    bestRoute.saving = BigNumber
    for ind in range(len(route2.sequence) + 1):
        newRoute = sev.street_vehicle()
        newRoute.sequence.extend(route2.sequence)
        if ind == len(route2.sequence):
            newRoute.sequence.extend(route1.sequence)
        else:
            newRoute.sequence[ind:ind] = route1.sequence[:]        
        
        newRoute.feas_check(problem)
        
        if newRoute.isFeasible:#
            calculate_saving(problem, route1, route2, newRoute)
            if newRoute.saving < bestRoute.saving:
                bestRoute = newRoute

    return bestRoute

def calculate_saving(problem, route1, route2, newRoute):
   
      
    if problem.type_objective == "waiting":
        saving = round(newRoute.waiting - (route1.waiting + route2.waiting), 2) # saving in waiting
    elif problem.type_objective == "makespan":
        saving = round(newRoute.makespan - (route1.makespan + route2.makespan), 2) ## saving in makespan
    elif problem.type_objective =="distance":
        saving = round(newRoute.travel - (route1.travel + route2.travel), 2) # saving in distances 
    elif problem.type_objective == "idle":
        saving = round(newRoute.idle - (route1.idle + route2.idle), 2) # + round(newRoute.makespan - (route1.makespan + route2.makespan), 2)
    else:
        saving = round(newRoute.available_saving  - (route1.available_saving + route2.available_saving), 2) #best working
    
    newRoute.saving = saving

    return

def check_merge(problem, route1, route2):
    if not problem.Construct_CW:
        newRoute = best_insertion_move(problem, route1, route2)
    else:
        newRoute = best_CW_insertion(problem, route1, route2)
    if not newRoute.isFeasible:
        newRoute.saving = BigNumber
        return newRoute

    calculate_saving(problem, route1, route2, newRoute)

    return newRoute 

def generate_initial_merges(problem, routesIn = None): #generate single route per customer and create initial matrix ( n by n )
    if not routesIn:
        routes = []
        for c_node in problem.nodes:
            if c_node.type != "C":
                continue
            newRoute = sev.street_vehicle()
            newRoute.sequence.append(c_node)
            newRoute.feas_check(problem)            
            if not newRoute.isFeasible:
                print("infeasible node exists")
            routes.append(newRoute) 
            newRoute.index = len(routes) - 1   #update index of all routes after any merge
    else:
        routes = routesIn

    all_merges = []
    routes = sorted(routes, key=lambda x: len(x.sequence), reverse=False)
        
    for ii, route1 in enumerate(routes):  
        all_merges.append([])      
        for jj, route2 in enumerate(routes):
            if ii == jj:
                all_merges[ii].append(BigNumber)
                continue  
            newRoute = check_merge(problem, route1, route2)          
            saving = newRoute.saving
            all_merges[ii].append(saving)

    return  all_merges, routes

def update_merges(problem, routes, all_merges):
    merged_route = routes[-1]

    all_merges.append([]) #new row for new route
    for ii, route2 in enumerate(routes):       
        if route2 == merged_route:
            all_merges[ii].append(BigNumber)
            continue            
        newRoute = check_merge(problem, merged_route, route2)    
        saving = newRoute.saving
        all_merges[-1].append(saving)

    for ii, route1 in enumerate(routes):
        if route1 == merged_route:
            continue
        newRoute = check_merge(problem, route1, merged_route)
        saving = newRoute.saving
        all_merges[ii].append(saving)

    return all_merges

def construction_CW(problem, routesIn = None): #no load and transfers will assigned later(to reduce pickup delivery conflict in transfer generation)
    start = time.time()
    merges_matrix, routes = generate_initial_merges(problem, routesIn)  #r by r matrix
    
    while np.amin(merges_matrix) < BigNumber:
        best_saving = BigNumber
        if problem.sequential and len(routes) < len(problem.customers): #fill the constructed route first
            for ii, route1 in enumerate(routes):
                if merges_matrix[ii][-1] < best_saving:
                    best_saving = merges_matrix[ii][-1]
                    i_removed = ii
                    j_removed = len(routes) - 1
            
        if best_saving == BigNumber:
            best_saving = np.amin(merges_matrix)
            merges_candidates = np.argwhere(merges_matrix == best_saving)
            #remove routes, add newly merged
            i_removed = merges_candidates[0][0]
            j_removed = merges_candidates[0][1]

        route1 = routes[i_removed]
        route2 = routes[j_removed]
        merged_route = check_merge(problem, route1, route2)
        if not merged_route.isFeasible:
            merges_matrix[i_removed][j_removed] = BigNumber 

            continue        
        #update routes
        routes.remove(route1)    
        routes.remove(route2)
        routes.append(merged_route)

        #update matrix by removing the rows and columns by decreasing index
        indices = [i_removed, j_removed]
        indices.sort(reverse=True)

        for ind in indices:
            merges_matrix = np.delete(merges_matrix, ind, 0)
            merges_matrix = np.delete(merges_matrix, ind, 1)        
        merges_matrix = merges_matrix.tolist()
        #remove merged ones, add new route and merges acc
        update_merges(problem, routes, merges_matrix)      

    arcs_current = []
    transfers_current = []
    for route in routes:
        route.feas_check(problem)
        route.generate_arcs()
        route.generate_transfers_full_load(problem)        
        arcs_current.extend(route.arcs)
        transfers_current.extend(route.transfers)
            
    problem.initial_CW_routes = routes          
    problem.construct_time = round(time.time() - start,2)
    
    currentSolution = solution.Solution(False)
    currentSolution.xij = arcs_current
    currentSolution.vi = transfers_current
    
    return currentSolution    


        
