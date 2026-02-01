import osmnx as ox
import geopandas as gpd
import random
import matplotlib.pyplot as plt
from shapely.geometry import Point

place_name = "Bostanlı, Karşıyaka, Izmir, Turkey"
G = ox.graph_from_place(place_name, network_type="drive")

nodes_gdf, edges_gdf = ox.graph_to_gdfs(G)

boundary_gdf = ox.geocode_to_gdf(place_name)
boundary_polygon = boundary_gdf.unary_union  # Merge all polygons if there are multiple

def generate_random_points_in_polygon(polygon, n_points):
    """Generate n_points random points within a given Shapely polygon."""
    points = []
    
    minx, miny, maxx, maxy = polygon.bounds
    
    while len(points) < n_points:
        rand_x = random.uniform(minx, maxx)
        rand_y = random.uniform(miny, maxy)
        candidate = Point(rand_x, rand_y)
        
        if candidate.within(polygon):
            points.append(candidate)
    
    return points

num_scooters = 50
random_scooters = generate_random_points_in_polygon(boundary_polygon, num_scooters)

scooters_gdf = gpd.GeoDataFrame(geometry=random_scooters, crs=boundary_gdf.crs)

fig, ax = plt.subplots(figsize=(10, 10))
edges_gdf.plot(ax=ax, color="black", linewidth=1)

scooters_gdf.plot(ax=ax, color="red", markersize=30, label="Scooters")

plt.title("Random Scooter Locations in Bostanlı, İzmir", fontsize=14)
plt.axis("off")
plt.legend()

plt.show()