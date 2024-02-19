import os
import sys
import io
import pandas
import matplotlib.pyplot as plt
import instanceGenerator
import integratedJointModel
import Construction_Heur
import solution
import time
from pathlib import Path
from os import walk

def ReadInitialSolution(file):
    currentSolution = solution.Solution(False)
    df = pandas.read_csv(file, sep="\t")

    df = df.values
    print(df) 
    
    
    for line in df:
        currentSolution.xij.append([int(line[0]), int(line[1])])
        if int(line[2] > 0.5):
            currentSolution.vi.append(int(line[0]))

    print(currentSolution.xij)
    print(currentSolution.vi)

    return currentSolution


def print_solution(bestknown_solution):
    if bestknown_solution.isFeasible:
        routes = []
        for ii, s in enumerate(bestknown_solution.routeSequeneces):
            sequence_nodes = []
            sequence_nodes.append(problem.nodes[0])
            for i in s:
                sequence_nodes.append(problem.nodes[i])
                if i in bestknown_solution.vi:
                    hub_id = bestknown_solution.hubs[bestknown_solution.vi.index(i)]
                    sequence_nodes.append(problem.nodes[hub_id])
            sequence_nodes.append(problem.nodes[0])
            routes.append(sequence_nodes)
        plot_vrp(routes, bestknown_solution.objVal)

def plot_vrp(sequences, obj):
    # Extract the x and y coordinates from the list of nodes
    nerwork_x_coordinates = [node.xcor for node in problem.nodes]
    network_y_coordinates = [node.ycor for node in problem.nodes]
    network_colors_by_type = []
    
    fig, ax = plt.subplots()

    for node in problem.nodes:
        ax.annotate(node.type, (node.xcor, node.ycor))
        if node.type == "CDC":
            network_colors_by_type.append("black")
        elif node.type == "S":
            network_colors_by_type.append("blue")
        elif node.type == "C":
            network_colors_by_type.append("yellow")
        else:
            network_colors_by_type.append("brown")

    # Plot the coordinates
    plt.plot(nerwork_x_coordinates, network_y_coordinates, 'o', markersize=10, color="blue")
    
    
    colors = ["green", "magenta", "red", "yellow"]

    font = {'family': 'serif',
        'color':  'darkred',
        'weight': 'normal',
        'size': 5,
        }
    for cc, sequence in enumerate(sequences):
        #plot the sequence
        sequence_x_coordinates = [node.xcor for node in sequence]
        sequence_y_coordinates = [node.ycor for node in sequence]
        sequence_routes = [node.index for node in sequence]
        plt.plot(sequence_x_coordinates, sequence_y_coordinates, '-', markersize=5, color=colors[cc])
        plt.text(min(nerwork_x_coordinates) - 3*cc, min(network_y_coordinates)-  3*cc, sequence_routes, fontdict=font)
    # Set the labels and title of the plot
    plt.xlabel('X-Coordinate')
    plt.ylabel('Y-Coordinate')
    plt.title('VRP Solution')

    # Show the plot
    plt.savefig(problem.name+str(len(problem.customers))+"_vehicles_"+str(len(sequences))+"objective"+str(round(obj,0))+".png")

#input parameters
#input1 = First instance index to solve, input2 = Last index to solve...... 
index_start =  int(sys.argv[1])  #the min index 0
index_end = int(sys.argv[2])# the max index 27 
size_start = int(sys.argv[3]) #the min index 0  [10, 20,30,40,50,100]
size_end =  int(sys.argv[4])  # the max index 6
cost_scen = int(sys.argv[5]) #max 3 [0.1, 0.2, 1, 10]
time_secs =  int(sys.argv[6]) #time limit in secs

cost_types = [0.1, 0.2, 1, 10]
inputCost = cost_types[cost_scen]

problem_sizes = [10, 20,30,40,50,100]
update_rule = [False, False ] # First: large costs fixed at makespan, then relative ratios are normalized and multiplied with base costs  

path = os.path.dirname(os.path.abspath(__file__))+"/data/json"
filenames = next(walk(path), (None, None, []))[2]

report = os.path.dirname(os.path.abspath(__file__)) +"/Computations/report_instance_"+str(index_start)+"_toinstance"+str(index_end)+"_sizes"+str(size_start)+"_"+str(size_end)+"_cost_scenario"+str(cost_scen)+".txt"
file1 = open(report,"w")
file1.close()
stats = os.path.dirname(os.path.abspath(__file__)) +"/Computations/statistics_instance_"+str(index_start)+"_toinstance"+str(index_end)+"_sizes"+str(size_start)+"_"+str(size_end)+"_cost_scenario"+str(cost_scen)+".txt"
file4 = open(stats,"w")
file4.close()

for instance_name in filenames[index_start:index_end]:
    for size in problem_sizes[size_start:size_end]:
        problem = instanceGenerator.readGenerate(instance_name, size, time_secs)#instance, size, WLS, time limit for solver

        #update costs
        problem.costScenario = inputCost
        problem.update_rule = update_rule                             
        problem.Set_cost_coefficients()

        #solution
        problem.modelStartTime = time.time()
        problem.report = report
        problem.stats = stats
        #solve
        Construction_Heur.InitialSolution_Heuristic(problem)
        bestknown_solution = integratedJointModel.JointSynchronized(problem) #solve joint model by BB
                
                