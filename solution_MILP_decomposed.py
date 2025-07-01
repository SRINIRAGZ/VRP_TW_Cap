'''
This script solves the VRP with time windows and capacity constraints using MILP formulation.
Since the in the provided problem, we do not differentiate between vehicles, we can reduce the 
variables if the problem by not iterating over vehicles. But care shoud be take to ensure different
tours and avoiding subtours, at the same time adhereing to the given Time window and Capacity constraints. 

Author: Srinivasragavan Venkatasubramanian
'''
import copy
import numpy as np
import pandas as pd
import pickle
from ortools.linear_solver import pywraplp
import os
import time

#GENERAL CONFIGS
FILE_DEMAND_SERVICE_ARRIVAL_TIME = 'demand_service_arrival_time.csv'
FILE_DISTANCE = 'distance.txt'
FILE_TRAVEL_TIME = 'travel_time.txt'

FILE_SUFFIX = ''
MPS_FILENAME = f'model{FILE_SUFFIX}.mps'
LP_FILENAME = f'model{FILE_SUFFIX}.lp'
RAW_SOLUTION_FILENAME = f'solution{FILE_SUFFIX}.pkl'
SOLUTION_FILENAME = f'solution_formatted{FILE_SUFFIX}.txt'
PRINT_SOLUTION_FILE = False #mode for printing solution in formatter way from pickle file

#MODEL CONFIGS
CAPACITY = 18           #Vehicle Capacity (Tonnes)
TIME_CAPACITY = 360     #Vehicle Tour Time Limit (Minutes)
VEHICLES = 20           #Number of Trucks available
DEPOT = 0               #Depot Node Index
BigM = 1440
SOLVE_TIME_LIMIT = 5   #in minutes
SOLVER_LOGS = True
#to turn on/off constraints
constraints_toggle = {
    'exit': True,
    'entry': True,
    'node_consistency': True,
    'arrival_time': True,
    'arrival_time_continuity': True,
    'demand_continuity': True,
    'truck_wait_during_tour': 0  # 0 / 1 / 2   options
        # 0 - can wait anytime during tour on road or customer location. But service should happen only during service window
        # 1 - wait allowed only at customer location within service window. Service can happend anytime withing the service window
        # 2 - no truck waiting allowed. meaning - truck immideately starts service after arrival at node and leaves immediately after service completion and no waiting anywhere.
}


#Creating SCIP solver in Google OR-Tools for solvig MILP problems
solver = pywraplp.Solver.CreateSolver('SCIP')
if SOLVER_LOGS:
    solver.EnableOutput()
#solver status
SOLVER_STATUS = {pywraplp.Solver.OPTIMAL: 'OPTIMAL',
                 pywraplp.Solver.FEASIBLE: 'FEASIBLE - NOT COMPLETED',
                 pywraplp.Solver.INFEASIBLE: 'INFEASIBLE',
                 pywraplp.Solver.UNBOUNDED: 'UNBOUNDED',
                 pywraplp.Solver.ABNORMAL: 'ABNORMAL - UNK',
                 pywraplp.Solver.NOT_SOLVED: 'NOT SOLVED - UNK',
}

#Read raw data from files
def read_raw_data(configs=None):
    df  = pd.read_csv(FILE_DEMAND_SERVICE_ARRIVAL_TIME, index_col=0)
    Dist = np.loadtxt(FILE_DISTANCE)
    TTime = np.loadtxt(FILE_TRAVEL_TIME)

    #convert to dict
    node_info = df.to_dict('index')
    return node_info, Dist, TTime
    
#Class that holds all the information about a node
class Node():
    def __init__(self, idx, Dist, TTime, node_info):
        self.idx = idx
        self.Dist_to = Dist[idx,:]
        self.Dist_from = Dist[:,idx]
        self.TTime_to = TTime[idx,:]
        self.TTime_from = TTime[:,idx]
        self.demand = node_info[idx]['demand']
        self.service_time = node_info[idx]['service_time']
        self.arrival_start = node_info[idx]['arrival_start']
        self.arrival_dur = node_info[idx]['arrival_duration']
        self.arrival_end = max(self.arrival_start , node_info[idx]['arrival_start'] + node_info[idx]['arrival_duration'] - node_info[idx]['service_time'])
        self.service_window_end = node_info[idx]['arrival_start'] + node_info[idx]['arrival_duration']
    
    def is_depot(self):
        return False if self.idx else True

#Route Solution
class Route():
    def __init__(self, idx, tour):
        self.idx = idx
        self.tour = copy.deepcopy(tour)                             #tour node visit sequence
        self.tour_start_time = 0                                    #departure from depot for starting tour
        self.service_start_time = []                                #service start time at each node
        self.node_departure_time = []                               #departure time from each node
        self.service_time = []                                      #service start time at each node
        self.service_window = []                                    #service start time at each node
        self.vehicle_load = []                                      #accumulating demand from each site visited
        self.seg_distance = []
        self.tour_distance = 0
        self.seg_travel_duration = []
        self.tour_duration = 0
        self.tour_end_time = 0
        self.tour_idle_time = 0

    
    def populate_other_route_data(self, U, C, T, Nodes, D=None):

        #tour start time 
        self.tour_start_time = T[self.tour[1]]

        #service start time at the node
        self.service_start_time = [0 if i==0 else round(U[self.tour[i-1]] + Nodes[self.tour[i-1]].service_time + Nodes[self.tour[i-1]].TTime_to[n]) if i==len(self.tour)-1 else round(U[n]) for i,n in enumerate(self.tour)]

        self.service_time = [Nodes[i].service_time for i in self.tour]
        if D:
            self.node_departure_time = [round(self.tour_start_time) if i==0 else np.inf if i==len(self.tour)-1 else round(D[n]) for i,n in enumerate(self.tour)]
        else:
            self.node_departure_time = [round(self.tour_start_time) if i==0 else np.inf if i==len(self.tour)-1 else round(U[n])+Nodes[n].service_time for i,n in enumerate(self.tour)]

        self.service_window = [(Nodes[i].arrival_start , Nodes[i].service_window_end) for i in self.tour]

        self.vehicle_load = [ round(C[self.tour[i-1], self.tour[i]])  for i in range(1,len(self.tour))]

        self.seg_distance = [ Nodes[self.tour[i]].Dist_from[self.tour[i-1]]  for i in range(1,len(self.tour))]

        self.tour_distance = sum(self.seg_distance)

        self.seg_travel_duration = [ Nodes[self.tour[i]].TTime_from[self.tour[i-1]]  for i in range(1,len(self.tour))]

        #tour duration = last visited note arrival + service time + travel time to depot ; Note: last visited node is -2 idx
        self.tour_duration = self.node_departure_time[-2] + Nodes[self.tour[-2]].TTime_to[DEPOT] - self.tour_start_time
        self.tour_end_time = self.node_departure_time[-2] + Nodes[self.tour[-2]].TTime_to[DEPOT]
        #truck wait times calculation
        self.tour_idle_time = max(0,self.tour_end_time - self.tour_start_time - sum(self.service_time) - sum(self.seg_travel_duration))


    def format_print(self):
        final_str = ''
        route_str = ''
        tw_str = ''
        times_str = ''
        ttime_str = ''
        cap_str = ''
        dist_str = ''
        arrowl_str = ''
        arrowv_str = ''
        for i,n in enumerate(self.tour):
            if i>0:
                route_str += '->'
                tw_str += '->'
                times_str += '->'
                arrowl_str += f"{'|':>18}"
                arrowv_str += f"{'v':>18}"
                ttime_str += f"{self.seg_travel_duration[i-1]:18}"
                cap_str += f"{self.vehicle_load[i-1]:18}"
                dist_str += f"{self.seg_distance[i-1]:18}"

            route_str += f"{n:^16}"
            tw_str += f"{str(self.service_window[i]):^16}"
            times_str += f"{str((self.service_start_time[i], self.node_departure_time[i], self.service_time[i])):^16}"
        #printing
        n = 33 + len(self.tour)*18
        final_str += f"\n{'-'*n}\nVehicle {self.idx}:\n{'-'*10}\n"
        final_str+= f'Route (Nodes Visited)        : {route_str}\n'
        final_str+= f'Service Time Window at node  : {tw_str}\n'
        final_str+= f'Times at Node (arr, dep, svc): {times_str}\n'
        final_str+= f'                              {arrowl_str}\n'
        final_str+= f'                              {arrowv_str}\n'
        final_str+= f'Travel Time                  : {ttime_str}\n'
        final_str+= f'Vehicle Load (Accumulation)  : {cap_str}\n'
        final_str+= f'Travel Distance              : {dist_str}\n'
        final_str+= f'Total Tour Distance : {self.tour_distance:.1f}\n'
        final_str+= f'Tour Start Time     : {self.tour_start_time:.1f}\n'
        final_str+= f'Tour End Time       : {self.tour_end_time:.1f}\n'
        final_str+= f'Total Tour Time     : {self.tour_duration:.1f}\n'
        final_str+= f'Max Tour Load       : {self.vehicle_load[-1]}\n'
        final_str+= f'Tour Idle Time      : {self.tour_idle_time:.1f}\n'
        return final_str

def graph_routes():
    pass

def generate_gantt_chart():
    pass


#get the solver solution and save it to a pickle file
def get_result(x=None, u=None, t=None, c=None, d=None, Nodes=None, obj=None, status=None):
    X = {}
    U = {}
    T = {}
    C = {}
    D = {}
    solution_string = ""
    if PRINT_SOLUTION_FILE:
        if os.path.exists(RAW_SOLUTION_FILENAME):
            with open(RAW_SOLUTION_FILENAME, "rb") as file:
                X, U, T, C = pickle.load(file)
    else:
        for k,v in x.items():
            if v.solution_value() == 1:
                X[k] = v.solution_value()
                C[k] = c[k].solution_value()

        for k,v in u.items():
            U[k] = v.solution_value()

        for k,v in t.items():
            T[k] = v.solution_value()
        if d:
            for k,v in d.items():
                D[k] = v.solution_value()
            with open(RAW_SOLUTION_FILENAME, "wb") as file:  # "wb" = write-binary mode
                pickle.dump((X,U,T,C,D), file)
        else:
            with open(RAW_SOLUTION_FILENAME, "wb") as file:  # "wb" = write-binary mode
                pickle.dump((X,U,T,C), file)

    tours = convert_solution_to_tours(X, 27)
    summary = [0,len(tours.keys())]
    tours_obj = []
    for k,v in tours.items():
        tours_obj.append(Route(k,v))
        if d:
            tours_obj[-1].populate_other_route_data( U=U, C=C, T=T, Nodes=Nodes, D=D)
        else:
            tours_obj[-1].populate_other_route_data( U=U, C=C, T=T, Nodes=Nodes)
        summary[0] += tours_obj[-1].tour_distance

    solution_string += f"\n\nSolution:\n{'-'*9}\nTotal Graph Distance: {summary[0]} mi\nNum Vehicles Used: {summary[1]}\nObjective: {obj:.1f}\nsolve status: {SOLVER_STATUS.get(status,'UNK')}\n"
    for r in tours_obj:
        solution_string += r.format_print()
    print(solution_string)
    if os.path.exists(SOLUTION_FILENAME):
        os.remove(SOLUTION_FILENAME)

    with open(SOLUTION_FILENAME, "w") as file:
        file.writelines(solution_string)
    

#Converts solver solution to interpretable format
def convert_solution_to_tours( X, num_nodes):
    tours = {}
    new_tour = 1
    for k in X.keys():
        if k[0] == DEPOT:
            tours[new_tour] = list(k)
            #iterate over all soln
            node = k[1]
            while node != DEPOT:
                for j in range(num_nodes):
                    if (node, j) in X:
                        tours[new_tour].append(j)
                        node = j
                        break
            new_tour += 1
    return tours


def MILP_solution_for_VRP(warm_start = None):
    node_info, Dist, TTime = read_raw_data()
    num_nodes = len(node_info)
    nodes = [Node(i, Dist, TTime, node_info) for i in range(num_nodes)]
    
    solver.set_time_limit(SOLVE_TIME_LIMIT * 60000)
    infinity = solver.infinity()
    #Route Arc Decison Variable
    x = {}
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i!=j:
                x[i,j] = solver.BoolVar(f"x_{i}_{j}")
    
    #Arrival Time Variable
    u = {}
    for i in range(num_nodes):
        if i!=DEPOT:
            u[i] = solver.NumVar(0, BigM, f'u_{i}')
    #tour start time variable
    t = {}
    for i in range(num_nodes):
        if i!=DEPOT:
            t[i] = solver.NumVar(0, BigM, f't_{i}')

    # # Apply warm start solution to model
    if warm_start:
        solver.SetHint([x[i, j] for i, j in warm_start.keys()], [warm_start[i, j] for i, j in warm_start.keys()])

    

    #Objective Function - minimize total distance travelled
    solver.Minimize(solver.Sum([x[i,j]*Dist[i,j] for i in range(num_nodes) for j in range(num_nodes) if i!=j]))

    #Constraints

    #Exit arc constraint
    if constraints_toggle['exit']:
        for i in range(num_nodes):
            if i!=DEPOT:
                solver.Add(solver.Sum([x[i,j] for j in range(num_nodes) if i!=j]) == 1, f'exit_arc_n({i})')

        #depot exit constraint
        solver.Add(solver.Sum([x[DEPOT,j] for j in range(num_nodes) if j!=DEPOT]) <= VEHICLES, f'exit_arc_DEPOT')

    #Entery arc constraint
    if constraints_toggle['entry']:
        for j in range(num_nodes):
            if j!=DEPOT:
                solver.Add(solver.Sum([x[i,j] for i in range(num_nodes) if i!=j] ) == 1, f'entry_arc_n({j})')

        #depot entry constraint
        solver.Add(solver.Sum([x[i,DEPOT] for i in range(num_nodes) if i!=DEPOT] ) <= VEHICLES, f'exit_arc_DEPOT')

    #Node Consistency constraint
    if constraints_toggle['node_consistency']:
        for h in range(num_nodes):
            solver.Add(solver.Sum([x[i,h] for i in range(num_nodes) if i!=h]) - solver.Sum([x[h,j] for j in range(num_nodes) if h!=j]) == 0, f'node_consistency_n({h})')
    
    
    #Demand continuity constraint
    if constraints_toggle['demand_continuity']: #optional for eliminating subtours
        c = {}
        for i in range(num_nodes):
            for j in range(num_nodes):
                if i!=j:
                        c[i,j] = solver.NumVar(0, CAPACITY, f'c_{i}_{j}')
        
        #capacity var and decision var linking
        for i in range(num_nodes):
            for j in range(num_nodes):
                if i!=j:
                    solver.Add(c[i,j] <= CAPACITY*x[i,j], f'capacity_var_link_I_n({i})_n({j})')
                    solver.Add(c[i,j] >= nodes[i].demand - (1-x[i,j])*CAPACITY, f'capacity_var_link_II_n({i})_n({j})')
        
        #cap continuity constraint - also eliminates subtours
        for i in range(num_nodes):
            if i!=DEPOT:
                solver.Add(solver.Sum([c[i,j] for j in range(num_nodes) if i!=j]) - solver.Sum([c[j,i] for j in range(num_nodes) if i!=j]) == nodes[i].demand, f'demand_continuity_n({i})')

    #Arrival Time constraints
    #Time window adherence with customer
    if constraints_toggle['arrival_time']:
        for i in range(num_nodes):
            if i!=DEPOT:
                solver.Add(u[i] >= nodes[i].arrival_start, f'arrival_start_bound_n({i})')
                solver.Add(u[i] <= nodes[i].arrival_end, f'arrival_end_bound_n({i})')
    #Arrival Time continuity with route - also eliminates subtours
    if constraints_toggle['arrival_time_continuity']:

        for i in range(num_nodes):
            for j in range(num_nodes):
                if i!=j and i!=DEPOT and j!=DEPOT:
                    solver.Add(u[i] + nodes[i].service_time + nodes[i].TTime_to[j] - u[j] <= (1-x[i,j])*BigM, f'arrival_time_continuity_I_n({i})_n({j})')
                    if constraints_toggle['truck_wait_during_tour']==2: #no waiting allowed anywhere during tour
                        solver.Add((x[i,j]-1)*BigM <= u[i] + nodes[i].service_time + nodes[i].TTime_to[j] - u[j], f'arrival_time_continuity_II_n({i})_n({j})')

        if constraints_toggle['truck_wait_during_tour']==1: #waiting allowed withing the service window at the customer locations
            #departure Time Variable
            d = {}
            for i in range(num_nodes):
                if i!=DEPOT:
                    d[i] = solver.NumVar(0, BigM, f'd_{i}')

            for i in range(num_nodes):
                if i!=DEPOT:
                    #departure bounds
                    solver.Add(d[i] >= u[i] + nodes[i].service_time, f'departure_start_bound_n({i})')
                    solver.Add(d[i] <= nodes[i].service_window_end , f'departure_end_bound_n({i})')
                    for j in range(num_nodes):
                        if j!=DEPOT and i!=j:
                            solver.Add(d[i] + nodes[i].TTime_to[j] <= u[j] + (1-x[i,j])*BigM , f'departure_arrival_continuity_I_n({i})_n({j})')
                            solver.Add(d[i] + nodes[i].TTime_to[j] >= u[j] - (1-x[i,j])*BigM , f'departure_arrival_continuity_II_n({i})_n({j})')

        

        for i in range(num_nodes):
            if i != DEPOT:
                solver.Add(t[i] <=  u[i] - nodes[i].TTime_from[DEPOT] + (1-x[DEPOT, i]) * BigM, f'tour_start_time_bound_ub_n({i})')
                solver.Add(t[i] >=  u[i] - nodes[i].TTime_from[DEPOT] - (1-x[DEPOT, i]) * BigM, f'tour_start_time_bound_lb_n({i})')
                
        #constraint to carry forward t - tour starting time for all nodes in the tour
        for i in range(num_nodes):
            for j in range(num_nodes):
                if i!=DEPOT and j!=DEPOT and i!=j: 
                    solver.Add(t[j] - t[i] <= BigM*(1-x[i,j]), f'carry fwd tour start I')
                    solver.Add(t[i] - t[j] <= BigM*(1-x[i,j]), f'carry fwd tour start II')

        for i in range(num_nodes):
            if i!=DEPOT:
                solver.Add(u[i] + nodes[i].service_time + nodes[i].TTime_to[DEPOT]-t[i] <= TIME_CAPACITY, f'time_capacity_cstr_({i})')


    # **Write to an MPS file**
    mps_content = solver.ExportModelAsMpsFormat(fixed_format=False, obfuscate=False)  # False means human-readable format
    with open(MPS_FILENAME, "w") as f:
        f.write(mps_content)

    print(f"MPS file saved as {MPS_FILENAME}")
    # **Write to an LP file**
    lp_content = solver.ExportModelAsLpFormat(False)  # False means human-readable format
    with open(LP_FILENAME, "w") as f:
        f.write(lp_content)

    print(f"LP file saved as {LP_FILENAME}")

    #SOLVE THE PROBLEM
    status = solver.Solve()

    #SOLUTION EXPORTING AND PRINTING
    if status  in (0,1):
        if constraints_toggle['truck_wait_during_tour'] == 1:
            get_result(x=x, u=u, t=t, c=c, d=d, Nodes=nodes, obj=solver.Objective().Value(), status=status)
        else:
            get_result(x=x, u=u, t=t, c=c, Nodes=nodes, obj=solver.Objective().Value(), status=status)
    else:
        print(f'ERROR: solver status : {SOLVER_STATUS.get(status,'UNKNOWN')}')

if __name__ == "__main__":
    start = time.time()
    MILP_solution_for_VRP()
    print(f'\n\ntotal time taken for solve: {time.time()-start:.2f} s')