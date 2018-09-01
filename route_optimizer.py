############################################################################
# TYPE    : Python Script                                                  #
# NAME    : Route_Optimizer.py                                             #
# PURPOSE : This script performs the gurobi model build, runs optimization #
#            to give an optimal path, given a cluster of nodes/cities      #
# Author  : Amit Kumar                                                     #
# Date    : 27-02-2018                                                     #
# Version : 2.0                                                            #
############################################################################


from gurobipy import GRB, Model, Env, gurobi
import os


def createEnvironment(machinenames,userpassword):
    if (self.env == 'dev'):
        gurobi_environment = Env.ClientEnv(logfilename='', computeServers=machinenames, timeout=-1)
    else:
        gurobi_environment = Env.ClientEnv(logfilename='', computeServers=machinenames, password=userpassword, timeout=-1)
    return gurobi_environment

def set_gurobi_params():
    env = 'dev'
    machineNames = <Gurobi server name>
    userPassword = <Gurobi server password>
    
    gurobi_env = createEnvironment(machineNames, userPassword)
    model_name = 'Route_Optimization'
    
    return model_name, gurobi_env


# Create new model
def create_model(model_name, gurobi_env):
    model = Model(model_name, gurobi_env)
    return model

# Binary Variables that indicates if a route is selected or not
def set_binary_variables(origin_city_list, destination_city_list, model):
    global vars
    vars = {}
    print("Setting Binary Variables")
    for oc in origin_city_list:
        for dc in destination_city_list:
            if edge_matrix[(oc, dc)] != 0:
                vars[oc, dc] = model.addVar(obj = 0.0, vtype = GRB.BINARY, name = 'x_' + str(oc) + '_' + str(dc))


# Integer variables required for eliminating sub-tours
def set_integer_variables(city_list, model):
    global var
    var = {}
    print("Setting Integer Variables to eliminate sub-tours")    
    for city in city_list:
        var[city] = model.addVar(obj = 0.0, lb = 0, vtype = GRB.INTEGER, ub = GRB.INFINITY, name = 'y_' + str(city))
    
def add_model_variables(city_list, origin_city_list, destination_city_list, model):
    set_binary_variables(origin_city_list, destination_city_list, model)
    set_integer_variables(city_list, model)
    model.update()

# Contraint that ensures only v number of vehicles starts from the source node
def degree_Const_1(destination_city_list, edge_matrix, model, vehicle_count):
    print("Contraint to ensure only v number of vehicles starts from the source")
    const1_expr_list = []
    for city in destination_city_list:
        if edge_matrix[origin, city] != 0:
            const1_expr_list.append(vars[origin, city])    
    model.addConstr(quicksum(const1_expr_list) == vehicle_count, name = 'Degree_Const_1')


# Contraint that ensures only v number of vehicles returns to the source node
def degree_Const_2(origin_city_list, edge_matrix, model, vehicle_count):
    print("Contraint to ensure only v number of vehicles returns to the source")
    const2_expr_list = []
    for city in origin_city_list:
        if edge_matrix[city, origin] != 0:
            const2_expr_list.append(vars[city, origin])    
    model.addConstr(quicksum(const2_expr_list) == vehicle_count, name = 'Degree_Const_2')


# Constraints that indicates that a city is visited only once by a vehicle
def degree_Const_3_4(origin_city_list, destination_city_list, model):
    print("Constraints that indicates a city is visited only once by a vehicle")
    for dc in destination_city_list[1:]:
        const3_expr_list = []
        for oc in origin_city_list:
            if(dc != oc):
                const3_expr_list.append(vars[oc, dc])
        model.addConstr(quicksum(const3_expr_list) == 1, name = 'Degree_Const_3_' + str(dc))

    for oc in origin_city_list[1:]:
        const4_expr_list = []
        for dc in destination_city_list:
            if(oc != dc):
                const4_expr_list.append(vars[oc, dc])
        model.addConstr(quicksum(const4_expr_list) == 1, name = 'Degree_Const_4_' + str(oc))


# Constraint to eliminate sub-tours
def sub_tour_elimination_const(city_list, model, city_num):
    for i in range(len(city_list) - 1):
        for j in range(len(city_list)):
            if (city_list[i] != city_list[j]):
                model.addConstr(var[city_list[i]] - var[city_list[j]] + city_num * vars[city_list[i], city_list[j]] <= city_num - 1, name = 'Subtour_rem_const_' + str(city_list[i]) + str(city_list[j]))

def add_model_constraints(city_list, origin_city_list, destination_city_list, edge_matrix, vehicle_count, city_num, model):
    degree_Const_1(destination_city_list, edge_matrix, model, vehicle_count)
    degree_Const_2(origin_city_list, edge_matrix, model, vehicle_count)
    degree_Const_3_4(origin_city_list, destination_city_list, model)
    sub_tour_elimination_const(city_list, model, city_num)
    model.update()
                
# Objective expression definition
def set_objective_expr(origin_city_list, destination_city_list, cost_matrix, model):
    expr_list = []
    for oc in origin_city_list:
        for dc in destination_city_list:
            if vars.has_key((oc, dc)):
                expr_list.append(vars[oc, dc] * cost_matrix[(oc, dc)])
    li = quicksum(expr_list)
        
    model.setObjective(li, GRB.MINIMIZE)
    model.update()

# Set time-limit for the model optimization
def optimize(model):
    model.setParam("TimeLimit", 50.0)

    # Optimize the model and write LP file
    model.optimize()
    if os.path.isfile('tsp.lp'):
        os.remove('tsp.lp')
    model.write('tsp.lp')
    
    # Check for infeasibility and write ILP file
    if model.status== GRB.INFEASIBLE:
        model.computeIIS()
        os.remove('tsp.ilp')
        model.write('tsp.ilp')


# Fetch model solutions (solved values of the variables)
def get_solution_vars_list(model):

    solution_vars = model.getAttr('x', vars)
    solution_var = model.getAttr('x', var)
    solution_vars_list = []
    
    for key, value in solution_vars.items():
        if value == 1.0:
            solution_vars_list.append(key)
    
    print "Solution: ", solution_vars_list
    return solution_vars_list


# Calculate the objective (total cost)
def get_total_cost(cost_matrix, solution_vars_list): 
  
    obj_val = 0
    for key, value in cost_matrix.items():
        if key in solution_vars_list:
            obj_val += cost_matrix[key]
    
    #print "Total Cost: ", obj_val
    return obj_val

# Fetch the optimal path
def get_optimal_path(city_path_list, origin):
    output_str = ""
    temp = origin
    
    while len(city_path_list) != 0:
        for tup in city_path_list:
            if tup[0] == temp:
                output_str += temp + "-->"
                temp = tup[1]
                city_path_list.remove(tup)            
    
    return(output_str + origin)

# Delete model
def delete_model(model):
    del model
    print("Model deleted")



    

if __name__ == "__main__":


    city_cost_edge_pdf = pd.read_table('tsp_city_data.txt', sep = '=')

    # Cost of travelling from one city to another
    temp_cost_matrix = city_cost_edge_pdf[city_cost_edge_pdf.column == 'cost_matrix']['data'][0]
    cost_matrix = ast.literal_eval(temp_cost_matrix)
    
    # Route existence indicator
    temp_edge_matrix = city_cost_edge_pdf[city_cost_edge_pdf.column == 'edge_matrix']['data'][0]
    edge_matrix = ast.literal_eval(temp_edge_matrix)
      
    # Origin and Destination city_list (READ FROM DATA PREP)
    city_list = city_cost_edge_pdf[city_cost_edge_pdf.column == 'city_list']['data'][0]
    origin_city_list = city_list
    destination_city_list = city_list
    
    #origin = 'Washington'
    origin = city_list[0]

    # Number of cities
    city_num = len(city_list)
    
    # Number of vehicles
    vehicle_count = 1
    

    print("Route Optimization Started")

    model_name, gurobi_env = set_gurobi_params()
    model = create_model(model_name, gurobi_env)
    
    add_model_variables(city_list, origin_city_list, destination_city_list, model)
    add_model_constraints(city_list, origin_city_list, destination_city_list, edge_matrix, vehicle_count, city_num, model)
    set_objective_expr(origin_city_list, destination_city_list, cost_matrix, model)
    optimize(model)
    
    solution_vars_list = get_solution_vars_list(model)  
    total_cost = get_total_cost(cost_matrix, solution_vars_list)
    print("Total Cost: " + str(total_cost))
    optimal_path = get_optimal_path(solution_vars_list, origin)
    print("Optimal Path: " + str(optimal_path))
    delete_model(model)

    print("Route Optimization Completed")
