<div align="center">

# scooter

*Routing, clustering, and multi-vehicle route planning for organizing shared e-scooters.*

</div>

---

<p align="center">
  <img src="img/Screenshot 2025-01-31 at 18.15.36.png" alt="scooter-ops demo" width="560">
</p>

---

## Initialization

This repository contains small, focused tools to: (1) generate scooter locations, (2) find hotspots, and (3) compute multi-vehicle routes (workers) that start at a central depot and minimize total travel.

- **Python Environment (recommended)**:

```bash
# pip
pip install numpy matplotlib ortools osmnx geopandas shapely networkx scikit-learn

# or conda (macOS/Linux often easier for Geo stack)
conda install -c conda-forge numpy matplotlib ortools osmnx geopandas shapely networkx scikit-learn
```

## Modules

* **routing_euclidean/v2.py**
  * Multi-vehicle VRP on a 50×50 plane (Euclidean distances). All routes start/end at the center depot (25,25). Per-vehicle capacity forces load to split across workers. Produces a color-coded Matplotlib diagram of routes and edges.
    * `num_scooters` — total points to visit (default 50)
	* `num_vehicles` — number of workers (e.g., 2, 5, …)
	* `capacities` — list length must equal `num_vehicles (e.g., [25, 25])`; set each `value < num_scooters` to force multiple routes
* **routing_osm/create.py**
  * End-to-end road-network VRP on real streets (OSMnx). Samples scooters on roads, snaps to nearest graph nodes, builds a network shortest-path distance matrix (meters), solves a multi-vehicle VRP, and plots routes over the street geometry.
* **hotspots/v3.py**
  * Road-constrained random scooters + DBSCAN clustering to detect hotspots. Draws cluster centers and frequency radii on top of the road graph to guide staging/worker placement.
* **generators/GenerateRandom.py**
  * Uniform random scooters inside the administrative polygon (not restricted to roads). Useful for synthetic experiments or comparisons against road-only sampling.