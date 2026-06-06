## STEP 1: Parse Li & Lim Data File
def parse_lilim(filepath):
    """
    Parse Li & Lim PDPTW benchmark file (lc101.txt)

    Returns:
        num_vehicles:       number of vehicles
        capacity:           vehicle capacity
        depot:              depot node (dict)
        nodes:              list of all nodes (depot at index 0)
        pickups_deliveries: list of pairs [pickup_id, delivery_id]
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()

    # First line: num_vehicles, capacity, speed
    first_line = lines[0].split()
    num_vehicles = int(first_line[0])
    capacity = int(first_line[1])
    # speed is not used

    nodes = []
    pickups_deliveries = []

    for line in lines[1:]:
        parts = line.split()
        if len(parts) == 0:
            continue

        node = {
            'id':           int(parts[0]),
            'x':            float(parts[1]),
            'y':            float(parts[2]),
            'demand':       int(parts[3]),
            'ready_time':   int(parts[4]),
            'due_date':     int(parts[5]),
            'service_time': int(parts[6]),
            'pickup':       int(parts[7]),   # if delivery node: id of paired pickup
            'delivery':     int(parts[8]),   # if pickup node: id of paired delivery
        }
        nodes.append(node)

        # Identify pickup nodes: pickup column=0 and delivery column!=0
        if node['pickup'] == 0 and node['delivery'] != 0:
            pickups_deliveries.append([node['id'], node['delivery']])

    depot = nodes[0]  # node with id=0 is the depot

    return num_vehicles, capacity, depot, nodes, pickups_deliveries


# ---- Run ----
num_vehicles, capacity, depot, nodes, pickups_deliveries = parse_lilim('lc101.txt')

print(f"Vehicles: {num_vehicles}, Capacity: {capacity}")
print(f"Depot: {depot}")
print(f"Total nodes (incl. depot): {len(nodes)}")
print(f"Number of pairs: {len(pickups_deliveries)}")
print(f"First 3 pairs: {pickups_deliveries[:3]}")


## STEP 2: Compute Distance Matrix
import math

def compute_distance_matrix(nodes):
    """
    Compute Euclidean distance matrix for all nodes (integer precision).
    Depot is at index 0; node id matches list index.
    """
    n = len(nodes)
    dist_matrix = [[0] * n for _ in range(n)]

    for i in range(n):
        for j in range(n):
            if i != j:
                dx = nodes[i]['x'] - nodes[j]['x']
                dy = nodes[i]['y'] - nodes[j]['y']
                dist_matrix[i][j] = int(math.sqrt(dx**2 + dy**2))

    return dist_matrix


# ---- Run ----
dist_matrix = compute_distance_matrix(nodes)

print(f"Distance matrix size: {len(dist_matrix)} x {len(dist_matrix[0])}")
print(f"Depot -> Node 1 distance: {dist_matrix[0][1]}")


## STEP 3: Create OR-Tools Model
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_model(num_vehicles, nodes, dist_matrix):
    """
    Create routing index manager and routing model.
    Returns manager, routing.
    """
    # Total nodes = len(nodes), depot node id = 0
    manager = pywrapcp.RoutingIndexManager(
        len(nodes),
        num_vehicles,
        0   # depot node id
    )

    routing = pywrapcp.RoutingModel(manager)

    # Register distance callback (arc cost)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return dist_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    return manager, routing


# ---- Run ----
manager, routing = create_model(num_vehicles, nodes, dist_matrix)
print("Model created successfully")


## STEP 4: Add Capacity Constraint
def add_capacity_constraint(routing, manager, nodes, num_vehicles, capacity):
    """
    Add capacity constraint.
    Pickup demand > 0 (load), delivery demand < 0 (unload).
    """
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return nodes[from_node]['demand']  # depot=0, pickup>0, delivery<0

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,                          # no slack
        [capacity] * num_vehicles,  # capacity upper bound per vehicle
        True,                       # start cumul from zero
        'Capacity'
    )


# ---- Run ----
add_capacity_constraint(routing, manager, nodes, num_vehicles, capacity)
print("Capacity constraint added successfully")


## STEP 5: Add Time Window Constraint
def add_time_window_constraint(routing, manager, nodes, dist_matrix):
    """
    Add time window constraint.
    Travel time = distance + service time at the origin node.
    Returns time_dimension (used by the PD constraint in STEP 6).
    """
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        travel_time = dist_matrix[from_node][to_node]
        service_time = nodes[from_node]['service_time']  # depot service_time = 0
        return travel_time + service_time

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    # Use depot's due_date as the global time upper bound
    max_time = nodes[0]['due_date']

    routing.AddDimension(
        time_callback_index,
        max_time,   # slack: maximum waiting time allowed
        max_time,   # upper bound for all nodes
        False,      # do not force start cumul to zero (allow waiting at depot)
        'Time'
    )

    # GetDimensionOrDie: retrieves the dimension by name; crashes if not found
    time_dimension = routing.GetDimensionOrDie('Time')

    # Set time window for each node
    for node in nodes:
        index = manager.NodeToIndex(node['id'])
        time_dimension.CumulVar(index).SetRange(
            node['ready_time'],
            node['due_date']
        )

    return time_dimension


# ---- Run ----
time_dimension = add_time_window_constraint(routing, manager, nodes, dist_matrix)
print("Time window constraint added successfully")


## STEP 6: Add Pickup & Delivery Constraint
def add_pickup_delivery_constraint(routing, manager, pickups_deliveries, time_dimension):
    """
    Add Pickup & Delivery constraints:
    1. Register pair
    2. Same vehicle
    3. Pickup before delivery
    """
    for pickup_id, delivery_id in pickups_deliveries:
        pickup_index = manager.NodeToIndex(pickup_id)
        delivery_index = manager.NodeToIndex(delivery_id)

        # 1. Register pair (informs solver these two nodes form one request)
        routing.AddPickupAndDelivery(pickup_index, delivery_index)

        # 2. Same vehicle constraint
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )

        # 3. Precedence constraint: pickup must be visited before delivery
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <= time_dimension.CumulVar(delivery_index)
        )


# ---- Run ----
add_pickup_delivery_constraint(routing, manager, pickups_deliveries, time_dimension)
print(f"PD constraints added successfully: {len(pickups_deliveries)} pairs")


## STEP 7: Solve
def solve(routing, manager, nodes, dist_matrix, num_vehicles):
    """
    Configure search parameters and solve. Print route for each vehicle.
    """
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.seconds = 30

    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        print("No solution found")
        return None

    total_distance = 0
    routes = []
    vehicles_used = 0

    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        route = []
        route_distance = 0

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(node)
            next_index = solution.Value(routing.NextVar(index))
            route_distance += dist_matrix[node][manager.IndexToNode(next_index)]
            index = next_index

        route.append(0)  # return to depot

        if len(route) > 2:  # skip empty routes
            routes.append(route)
            vehicles_used += 1
            end_time = solution.Value(
                routing.GetDimensionOrDie('Time').CumulVar(routing.End(vehicle_id))
            )
            print(f"Vehicle {vehicle_id}: {route}")
            print(f"  Distance: {route_distance}, End time: {end_time}")
            total_distance += route_distance

    print(f"\nVehicles used: {vehicles_used}")
    print(f"Total distance: {total_distance} (integer precision)")
    print(f"\nBest known solution - Vehicles: 10, Distance: 828.94 (float precision)")

    return routes


# ---- Run ----
routes = solve(routing, manager, nodes, dist_matrix, num_vehicles)


## STEP 8: Visualization
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

def visualize_routes(nodes, routes, pickups_deliveries):
    """
    Visualize VRPDPTW routes.
    - Different colors per route
    - Pickup nodes: triangle (▲), delivery nodes: inverted triangle (▼), depot: star (★)
    """
    pickup_ids = set(p[0] for p in pickups_deliveries)
    delivery_ids = set(p[1] for p in pickups_deliveries)

    fig, ax = plt.subplots(figsize=(14, 10))

    # Plot routes
    colors = cm.tab20(np.linspace(0, 1, max(len(routes), 1)))
    for idx, (route, color) in enumerate(zip(routes, colors)):
        xs = [nodes[n]['x'] for n in route]
        ys = [nodes[n]['y'] for n in route]
        ax.plot(xs, ys, color=color, linewidth=1.5, alpha=0.7, label=f'Vehicle {idx}')

    # Plot pickup nodes (green triangle)
    for nid in pickup_ids:
        ax.scatter(nodes[nid]['x'], nodes[nid]['y'],
                   color='green', marker='^', s=60, zorder=4)

    # Plot delivery nodes (orange inverted triangle)
    for nid in delivery_ids:
        ax.scatter(nodes[nid]['x'], nodes[nid]['y'],
                   color='orange', marker='v', s=60, zorder=4)

    # Plot depot (red star)
    ax.scatter(nodes[0]['x'], nodes[0]['y'],
               color='red', marker='*', s=200, zorder=5, label='Depot')

    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='*', color='w', markerfacecolor='red',    markersize=12, label='Depot'),
        Line2D([0], [0], marker='^', color='w', markerfacecolor='green',  markersize=10, label='Pickup'),
        Line2D([0], [0], marker='v', color='w', markerfacecolor='orange', markersize=10, label='Delivery'),
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    ax.set_title('VRPDPTW Routes - lc101', fontsize=14)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    plt.tight_layout()
    plt.savefig('Routes-lc101.png', dpi=150)
    plt.show()
    print("Route map saved as Routes-lc101.png")


# ---- Run ----
if routes:
    visualize_routes(nodes, routes, pickups_deliveries)


# STEP 9: Nearest Neighbor Heuristic 

def nearest_neighbor_vrpdptw(nodes, dist_matrix, capacity, pickups_deliveries):
    """
    Greedy Nearest Neighbor heuristic for VRPDPTW.
    
    Rules:
    - Only visit a pickup before its paired delivery
    - Same vehicle must serve both pickup and delivery
    - Respect time windows and capacity
    """
    # Build a lookup: pickup_id -> delivery_id
    pair_map = {p: d for p, d in pickups_deliveries}
    
    unvisited_pickups = set(p for p, d in pickups_deliveries)
    unvisited_deliveries = set()  # deliveries become available after pickup is done
    
    routes = []
    total_distance = 0.0

    while unvisited_pickups or unvisited_deliveries:
        # Dispatch a new vehicle
        route = [0]
        current = 0
        current_time = 0.0
        current_load = 0
        route_distance = 0.0
        pending_deliveries = set()  # deliveries this vehicle must complete

        while True:
            best = None
            best_dist = float('inf')

            # Candidates: available pickups + pending deliveries for this vehicle
            candidates = unvisited_pickups | pending_deliveries

            for j in candidates:
                node = nodes[j]
                d = dist_matrix[current][j]
                arrival = max(current_time + d, node['ready_time'])

                # Check time window and capacity
                if arrival <= node['due_date'] and current_load + node['demand'] <= capacity:
                    if d < best_dist:
                        best_dist = d
                        best = j

            if best is None:
                break

            # Visit the chosen node
            node = nodes[best]
            route_distance += dist_matrix[current][best]
            current_time = max(current_time + dist_matrix[current][best], node['ready_time'])
            current_time += node['service_time']
            current_load += node['demand']
            route.append(best)

            if best in unvisited_pickups:
                # Pickup visited: unlock its delivery for this vehicle
                unvisited_pickups.remove(best)
                pending_deliveries.add(pair_map[best])
            else:
                # Delivery visited
                pending_deliveries.remove(best)

            current = best

        # Return to depot
        route_distance += dist_matrix[current][0]
        route.append(0)
        routes.append(route)
        total_distance += route_distance

    print(f"[Nearest Neighbor] Vehicles used: {len(routes)}")
    print(f"[Nearest Neighbor] Total distance: {total_distance:.2f}")
    return routes, total_distance


# Run
nn_routes, nn_distance = nearest_neighbor_vrpdptw(nodes, dist_matrix, capacity, pickups_deliveries)
