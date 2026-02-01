import math
import random
import numpy as np
import matplotlib.pyplot as plt

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def generate_random_coordinates(n, low=0, high=50):
    coords = []
    for _ in range(n):
        x = random.uniform(low, high)
        y = random.uniform(low, high)
        coords.append((x, y))
    return coords

def create_distance_matrix(coords):
    n = len(coords)
    dist_matrix = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i == j:
                dist_matrix[i, j] = 0
            else:
                dist_matrix[i, j] = math.dist(coords[i], coords[j])
    return dist_matrix

def solve_multi_vehicle(dist_matrix, num_vehicles, depot_index, capacities):
    n_nodes = len(dist_matrix)
    
    starts = [depot_index] * num_vehicles
    ends   = [depot_index] * num_vehicles

    manager = pywrapcp.RoutingIndexManager(n_nodes, num_vehicles, starts, ends)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node   = manager.IndexToNode(to_index)
        return int(dist_matrix[from_node][to_node])
    
    transit_cb_idx = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_idx)
    
    demands = [0]*n_nodes
    for i in range(n_nodes):
        if i != depot_index:
            demands[i] = 1  # each scooter node has demand = 1

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]
    
    demand_cb_idx = routing.RegisterUnaryTransitCallback(demand_callback)
    
    routing.AddDimensionWithVehicleCapacity(
        demand_cb_idx,  # demand callback index
        0,              # slack
        capacities,     # list of capacities, one for each vehicle
        True,           # start cumulative to 0
        "Capacity"
    )

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_params)
    if not solution:
        print("No solution found.")
        return None
    
    routes = []
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route.append(node_index)
            index = solution.Value(routing.NextVar(index))
        # route ends at the depot
        routes.append(route)
    return routes

def main():
    random.seed(42)  # for reproducible results

    num_scooters = 50
    scooter_coords = generate_random_coordinates(num_scooters, 0, 50)

    depot_coord = (25, 25)
    coords = [depot_coord] + scooter_coords  # node 0 is the depot

    dist_mat = create_distance_matrix(coords)

    num_vehicles = 5   
    capacities = [5, 15, 10, 10, 10]

    routes = solve_multi_vehicle(dist_mat, num_vehicles, depot_index=0, capacities=capacities)
    if routes is None:
        return
    
    print("Routes found (each includes depot at start & end):")
    for i, rt in enumerate(routes):
        print(f"Worker {i} route: {rt}")

    x_all = [c[0] for c in coords]
    y_all = [c[1] for c in coords]

    fig, ax = plt.subplots(figsize=(8, 8))
    
    ax.scatter(x_all, y_all, c="black", s=30, zorder=3, label="Scooters")

    ax.scatter(coords[0][0], coords[0][1], c="red", s=100, marker="D", zorder=4, label="Depot")

    color_map = plt.cm.tab10(np.linspace(0,1,num_vehicles))
    
    for v_idx, route in enumerate(routes):
        color = color_map[v_idx % len(color_map)]
        
        for i in range(len(route) - 1):
            start_node = route[i]
            end_node   = route[i+1]
            x_start, y_start = coords[start_node]
            x_end,   y_end   = coords[end_node]
            ax.plot([x_start, x_end], [y_start, y_end], color=color, linewidth=2, zorder=2)

        if len(route) > 1:
            start_x, start_y = coords[route[0]]
            end_x,   end_y   = coords[route[-1]]
            ax.scatter(start_x, start_y, c=color, s=80, marker="s", edgecolors="white", zorder=4,
                       label=f"Worker {v_idx} start")
            ax.scatter(end_x, end_y,   c=color, s=80, marker="*", edgecolors="white", zorder=4,
                       label=f"Worker {v_idx} end")

    ax.set_title(f"Multi-Vehicle Routing with {num_vehicles} Workers (Capacity Constraints)")
    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_xlabel("X coordinate")
    ax.set_ylabel("Y coordinate")
    ax.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()