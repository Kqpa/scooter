import osmnx as ox
import geopandas as gpd
import random
import matplotlib.pyplot as plt
from shapely.geometry import Point
import numpy as np
from sklearn.cluster import DBSCAN

def sample_points_on_lines(edges_gdf, n_points=100):
    edges_gdf["length_m"] = edges_gdf.geometry.length
    
    edges_gdf["cum_length_m"] = edges_gdf["length_m"].cumsum()
    total_length = edges_gdf["length_m"].sum()

    random_points = []
    for _ in range(n_points):
        r = random.uniform(0, total_length)
        
        row = edges_gdf.loc[edges_gdf["cum_length_m"] >= r].iloc[0]
        
        distance_on_line = r - (row["cum_length_m"] - row["length_m"])
        
        line_geom = row.geometry
        point_on_line = line_geom.interpolate(distance_on_line)
        random_points.append(point_on_line)

    points_gdf = gpd.GeoDataFrame(geometry=random_points, crs=edges_gdf.crs)
    return points_gdf

def main():
    place_name = "Vaniköy, Kandilli, İstanbul, Turkey"
    G = ox.graph_from_place(place_name, network_type="drive")
    nodes_gdf, edges_gdf = ox.graph_to_gdfs(G)

    edges_gdf = edges_gdf.to_crs(epsg=3857)

    num_scooters = 100
    scooters_gdf = sample_points_on_lines(edges_gdf, n_points=num_scooters)

    scooters_gdf = scooters_gdf.to_crs(epsg=3857)  # ensure they're in the same meter-based CRS
    coords = np.array([[p.x, p.y] for p in scooters_gdf.geometry])

    db = DBSCAN(eps=150, min_samples=3)
    labels = db.fit_predict(coords)
    scooters_gdf["cluster"] = labels

    clusters = []
    valid_labels = [c for c in set(labels) if c != -1]  # ignore noise (-1)
    for cluster_id in valid_labels:
        cluster_points = scooters_gdf[scooters_gdf["cluster"] == cluster_id]
        cluster_coords = np.array([[pt.x, pt.y] for pt in cluster_points.geometry])

        center_x = cluster_coords[:, 0].mean()
        center_y = cluster_coords[:, 1].mean()

        dists = np.sqrt((cluster_coords[:, 0] - center_x) ** 2 + (cluster_coords[:, 1] - center_y) ** 2)
        max_dist = dists.max()

        clusters.append({
            "cluster_id": cluster_id,
            "frequency": len(cluster_points),  # number of points in cluster
            "center": Point(center_x, center_y),
            "radius_m": max_dist
        })

    clusters_gdf = gpd.GeoDataFrame(clusters, geometry="center", crs="EPSG:3857")

    fig, ax = plt.subplots(figsize=(10, 10))

    edges_gdf.plot(ax=ax, color="black", linewidth=1, label="Roads")

    scooters_gdf.plot(
        ax=ax,
        column="cluster",
        categorical=True,
        cmap="tab10",
        markersize=50,
        alpha=0.8,
        legend=True,
        label="Scooters"
    )

    clusters_gdf.plot(ax=ax, color="blue", marker="*", markersize=200, label="Cluster Center")

    for row in clusters_gdf.itertuples():
        circ = plt.Circle(
            (row.center.x, row.center.y),
            row.radius_m,
            color="blue",
            fill=False,
            linewidth=2,
            alpha=0.5
        )
        ax.add_patch(circ)

    plt.title("Bostanlı: Random Scooters, Clusters, and Frequency Radii", fontsize=14)
    plt.axis("off")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()