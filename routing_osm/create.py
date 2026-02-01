import osmnx as ox
import networkx as nx
import geopandas as gpd
import numpy as np
import random
import matplotlib.pyplot as plt
from shapely.geometry import Point
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

place_name = "Bostanlı, Karşıyaka, Izmir, Turkey"
G = ox.graph_from_place(place_name, network_type="drive")

nodes_gdf, edges_gdf = ox.graph_to_gdfs(G)

edges_gdf = edges_gdf.to_crs(epsg=3857)

largest_cc = max(nx.strongly_connected_components(G), key=len)
G = G.subgraph(largest_cc).copy()
print(f"Using largest connected component with {len(G.nodes)} nodes.")

def sample_points_on_lines(edges_gdf, n_points=100):
    edges_gdf['length_m'] = edges_gdf.geometry.length
    edges_gdf['cum_length_m'] = edges_gdf['length_m'].cumsum()
    total_length = edges_gdf['length_m'].sum()

    random_points = []
    for _ in range(n_points):
        r = random.uniform(0, total_length)
        row = edges_gdf.loc[edges_gdf['cum_length_m'] >= r].iloc[0]
        distance_on_line = r - (row['cum_length_m'] - row['length_m'])
        line_geom = row.geometry
        point_on_line = line_geom.interpolate(distance_on_line)
        random_points.append(point_on_line)
    
    return gpd.GeoDataFrame(geometry=random_points, crs=edges_gdf.crs)

num_scooters = 50
scooters_on_roads_gdf = sample_points_on_lines(edges_gdf, n_points=num_scooters)

scooter_nodes = []
for point in scooters_on_roads_gdf.geometry:
    nearest_node = ox.distance.nearest_nodes(G, point.x, point.y)
    if nearest_node in G.nodes:
        scooter_nodes.append(nearest_node)

depot_node = random.choice(list(G.nodes))
all_nodes = [depot_node] + scooter_nodes

num_nodes = len(all_nodes)
dist_matrix = np.zeros((num_nodes, num_nodes))

for i in range(num_nodes):
    for j in range(num_nodes):
        if i == j:
            dist_matrix[i, j] = 0
        else:
            try:
                dist_matrix[i, j] = nx.shortest_path_length(G, all_nodes[i], all_nodes[j], weight="length")
            except nx.NetworkXNoPath:
                dist_matrix[i, j] = np.inf

for i in range(num_nodes):
    for j in range(num_nodes):
        if dist_matrix[i][j] == np.inf:
            print(f"No valid path between node {i} and node {j}")

def solve_multi_vehicle(dist_matrix, num_vehicles, depot_index, capacities):
    n_nodes = len(dist_matrix)
    manager = pywrapcp.RoutingIndexManager(n_nodes, num_vehicles, depot_index)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(dist_matrix[from_node][to_node])
    
    transit_cb_idx = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_idx)

    demands = [0] * n_nodes
    for i in range(1, n_nodes):
        demands[i] = 1

    demand_cb_idx = routing.RegisterUnaryTransitCallback(lambda index: demands[manager.IndexToNode(index)])
    
    routing.AddDimensionWithVehicleCapacity(
        demand_cb_idx, 0, capacities, True, "Capacity"
    )

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.time_limit.seconds = 20  # Increased solver time limit

    solution = routing.SolveWithParameters(search_params)
    if not solution:
        print("No solution found.")
        return None
    
    routes = []
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        routes.append(route)
    
    return routes

num_vehicles = 5
capacities = [10] * num_vehicles 
routes = solve_multi_vehicle(dist_matrix, num_vehicles, depot_index=0, capacities=capacities)

if not routes or all(len(r) == 0 for r in routes):
    print("No routes generated. The solver may have failed.")
else:
    print("Routes successfully generated.")
    for i, route in enumerate(routes):
        print(f"Worker {i+1} Route: {route}")

fig, ax = plt.subplots(figsize=(10, 10))

edges_gdf.plot(ax=ax, color="black", linewidth=0.8, alpha=0.6, label="Roads")

scooters_on_roads_gdf.plot(ax=ax, color="red", marker="o", markersize=30, label="Scooters", zorder=3)

ax.scatter(G.nodes[depot_node]['x'], G.nodes[depot_node]['y'], color="blue", s=100, marker="D", label="Depot", zorder=4)

colors = plt.cm.get_cmap("tab10", num_vehicles)
for v_idx, route in enumerate(routes):
    if len(route) > 1:
        route_coords = [G.nodes[all_nodes[i]] for i in route if all_nodes[i] in G.nodes]
        if route_coords:
            x_vals, y_vals = zip(*[(node['x'], node['y']) for node in route_coords])
            ax.plot(x_vals, y_vals, color=colors(v_idx), linewidth=2, label=f"Worker {v_idx+1}")

plt.title("Multi-Vehicle Scooter Collection Routes in Bostanlı, İzmir", fontsize=14)
plt.legend()
plt.axis("off")
plt.show()
