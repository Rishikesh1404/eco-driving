from copy import deepcopy
import sys
import contextlib

sys.setrecursionlimit(2000)

# Assuming 1 period is 1 minute, convert it to seconds
period = 60  # 1 minute in seconds
adj = []
shortestPaths = [[]]
currentSolution = float("inf")
solutionState = None
recharging_stations = set()  # Set to store recharging stations

def print_results(vehicle, solution_state):
    print("Vehicle " + str(vehicle.id) + ":")
    print("Heuristic Underestimate (Distance): " + str(shortestPaths[vehicle.source][vehicle.destination][0]) + "m")
    print("Heuristic Underestimate (Time): " + str(vehicle.heuristicUnderestimate) + "s")
    print("Heuristic Route: " + " -> ".join(shortestPaths[vehicle.source][vehicle.destination][1]))
    if solution_state:
        print("Optimal Schedule/Route: " + " -> ".join(solution_state.vehicleStates[vehicle.id - 1].route))
        print("Optimal Time: " + str(solution_state.vehicleStates[vehicle.id - 1].timestamp) + "s\n")
    else:
        print("Error computing optimal schedule and time. Please try varying the time period.\n")

def driver(filename='testcases.txt', output_file='output.txt'):
    global adj
    global shortestPaths
    global period
    global solutionState
    global recharging_stations

    recharging_stations = set()  # Initialize recharging_stations here

    with open(filename, 'r') as file:
        lines = file.readlines()

    index = 0

    with open(output_file, 'w') as output:
        with contextlib.redirect_stdout(output):  # Redirecting stdout using a context manager
            
            # n -> number of cities (nodes / vertices); m -> number of roads (edges); k -> number of vehicles
            n, m, k = map(int, lines[index].split(" "))
            index += 1
            period = int(lines[index])
            index += 1

            # edges -> local adjacency list
            edges = [[] for _ in range(n)]
            vehicles = []

           
            for i in range(m):
              # Add this line for debugging
                a, b, d = map(int, lines[index].split(" "))
                index += 1
                edges[a - 1].append([b - 1, d])
                edges[b - 1].append([a - 1, d])

             # Read recharging stations
            recharging_station_count = int(lines[index].split()[0])
            index += 1
            for _ in range(recharging_station_count):
                station_id = int(lines[index].split()[0])
                recharging_stations.add(station_id - 1)
                index += 1

            adj = edges

            dijkstraAll(n)

            for i in range(k):
                a, b = map(int, lines[index].split(" "))
                index += 1
                c, d, e, f, g = map(float, lines[index].split(" "))
                index += 1
                dist, path = shortestPaths[a - 1][b - 1][0], shortestPaths[a - 1][b - 1][1]
                vehicles.append(Vehicle(i + 1, a - 1, b - 1, c, d, e, f, g, dist, path))
            
            initialState = State(n, vehicles)

            generate(initialState)

            print("\nAssuming all distance measurements are provided in kilometers and time in minutes.")
            print("\nOptimal Time for all vehicles to reach their destinations is " + str(round(currentSolution / 60, 2)) + " minutes ± " + str(period / 60) + " minutes.\n")
            for i in range(k):
                print_results(vehicles[i], solutionState)

    return




def dijkstraAll(n):
    global shortestPaths
    global adj
    shortestPaths = [[0 for j in range(n)] for i in range(n)]
    for i in range(n):
        for j in range(n):
            a, b = dijkstrasAlgorithm(i, j, adj)
            shortestPaths[i][j] = [a, b]


def generate(state, currentVehicle=0):
    global currentSolution
    global shortestPaths
    global solutionState

    if not state.feasible:
        return

    k = len(state.vehicleStates)

    if currentVehicle == k:

        t = state.timestamp
        state.timestamp += period

        if state.success():
            for vehicleState in state.vehicleStates:
                state.timestamp = max(state.timestamp, vehicleState.timestamp)
            if state.timestamp < currentSolution:
                currentSolution = min(currentSolution, state.timestamp)
                solutionState = deepcopy(state)
            state.timestamp = t
            return

        else:
            generate(state)

        state.timestamp -= period

    else:
        vehicleState = state.vehicleStates[currentVehicle]

        if vehicleState.arrived:
            generate(state, currentVehicle + 1)
            return

        if vehicleState.timestamp <= state.timestamp:
            vehicleState.timestamp = state.timestamp

            # charge the vehicle at current node
            if vehicleState.charge < vehicleState.info.maxCapacity and (
                    shortestPaths[vehicleState.position][vehicleState.info.destination][0]) / (
                    vehicleState.info.averageSpeed) > vehicleState.charge / vehicleState.info.dischargingRate:
                # charge
                if state.nodeState[vehicleState.position] == 0 or state.nodeState[
                    vehicleState.position] == vehicleState.info.id:
                    state.nodeState[vehicleState.position] = vehicleState.info.id
                    t = vehicleState.charge
                    t1 = vehicleState.route
                    vehicleState.charge = min(vehicleState.charge + vehicleState.info.chargingRate * period,
                                              vehicleState.info.maxCapacity)
                    if vehicleState.route[-1][0] != 'C':
                        vehicleState.route.append('Charge for ' + str(period) + 's')
                    else:
                        vehicleState.route[-1] = vehicleState.route[-1][:11] + str(
                            int(vehicleState.route[-1][11:-1]) + period) + "s"
                    vehicleState.timestamp += period
                    generate(state, currentVehicle + 1)
                    vehicleState.timestamp -= period
                    vehicleState.charge = t
                    vehicleState.route = t1
                    state.nodeState[vehicleState.position] = 0
                # wait
                else:
                    vehicleState.timestamp += period
                    t = vehicleState.route
                    if vehicleState.route[-1][0] != 'W':
                        vehicleState.route.append('Wait for ' + str(period) + 's')
                    else:
                        vehicleState.route[-1] = vehicleState.route[-1][:9] + str(
                            int(vehicleState.route[-1][9:-1]) + period) + "s"
                    generate(state, currentVehicle + 1)
                    vehicleState.timestamp -= period
                    vehicleState.route = t

            # move on
            state.nodeState[vehicleState.position] = 0
            canMoveOn = False
            for edge in adj[vehicleState.position]:
                if edge[1] > (vehicleState.charge / vehicleState.info.dischargingRate) * (
                        vehicleState.info.averageSpeed) or edge[0] in vehicleState.visitedMemo: continue
                a = str(vehicleState.position)
                b = shortestPaths[edge[0]][vehicleState.info.destination]
                if str(vehicleState.position + 1) in shortestPaths[edge[0]][vehicleState.info.destination][1]:
                    continue
                canMoveOn = True
                t1 = vehicleState.timestamp
                t = edge[1] / vehicleState.info.averageSpeed
                vehicleState.timestamp += t
                t2 = vehicleState.charge
                vehicleState.charge -= t * vehicleState.info.dischargingRate
                t3 = vehicleState.position
                vehicleState.position = edge[0]
                t4 = vehicleState.arrived
                vehicleState.arrived = edge[0] == vehicleState.info.destination
                vehicleState.route.append(str(edge[0] + 1))
                vehicleState.visitedMemo.add(edge[0])
                generate(state, currentVehicle + 1)
                vehicleState.visitedMemo.remove(edge[0])
                vehicleState.route.pop()
                vehicleState.arrived = t4
                vehicleState.position = t3
                vehicleState.charge = t2
                vehicleState.timestamp = t1

            if not canMoveOn:
                vehicleState.canArrive = False

    return


# Modified Dijkstra's Algorithm using Min-Heaps | O((n + m) * log(n)) time | O(n) space,
# where n is the number of vertices and m is the number of edges in the input graph.
# Is used to calculate shortest path time, which in turn, together with required charging time for said shortest path, is used to generate a heuristic underestimate.
def dijkstrasAlgorithm(start, end, edges):
    numberOfVertices = len(edges)

    minDistances = [float("inf") for _ in range(numberOfVertices)]
    minDistances[start] = 0

    predecessors = [None for _ in range(numberOfVertices)]

    minDistancesHeap = MinHeap([(idx, float("inf")) for idx in range(numberOfVertices)])
    minDistancesHeap.update(start, 0)

    while not minDistancesHeap.isEmpty():
        vertex, currentMinDistance = minDistancesHeap.remove()

        if currentMinDistance == float("inf"):
            break

        for edge in edges[vertex]:
            destination, distanceToDestination = edge

            newPathDistance = currentMinDistance + distanceToDestination
            currentDestinationDistance = minDistances[destination]
            if newPathDistance < currentDestinationDistance:
                minDistances[destination] = newPathDistance
                minDistancesHeap.update(destination, newPathDistance)
                predecessors[destination] = vertex

    shortestPath = [str(end + 1)]
    vertex = end
    while predecessors[vertex] is not None:
        shortestPath.append(str(predecessors[vertex] + 1))
        vertex = predecessors[vertex]
    shortestPath.reverse()

    return (-1, shortestPath) if minDistances[end] == float("inf") else (minDistances[end], shortestPath)


class Vehicle:
    def __init__(self, id, source, destination, initialBattery, chargingRate, dischargingRate, maxCapacity,
                 averageSpeed,
                 shortestDistance, shortestRoute):
        self.id = id
        self.source = source
        self.destination = destination
        self.initialBattery = initialBattery
        self.chargingRate = chargingRate
        self.dischargingRate = dischargingRate
        self.maxCapacity = maxCapacity
        self.averageSpeed = averageSpeed / 3.6  # km/h to m/s
        self.shortestRoute = shortestRoute
        self.minRoadTime = (shortestDistance * 1000) / self.averageSpeed  # Convert km to m for distance
        self.minChargingTime = max(0, (self.minRoadTime - self.initialBattery / self.dischargingRate) * (
                self.dischargingRate / self.chargingRate))
        self.needsToBeCharged = self.minChargingTime > 0
        self.heuristicUnderestimate = self.minRoadTime + self.minChargingTime


class MinHeap:
    def __init__(self, array):
        self.vertexMap = {idx: idx for idx in range(len(array))}
        self.heap = self.buildHeap(array)

    def isEmpty(self):
        return len(self.heap) == 0

    def buildHeap(self, array):
        firstParentIdx = (len(array) - 2) // 2
        for currentIdx in reversed(range(firstParentIdx + 1)):
            self.siftDown(currentIdx, len(array) - 1, array)
        return array

    def siftDown(self, currentIdx, endIdx, heap):
        childOneIdx = currentIdx * 2 + 1
        while childOneIdx <= endIdx:
            childTwoIdx = currentIdx * 2 + 2 if currentIdx * 2 + 2 <= endIdx else -1
            if childTwoIdx != -1 and heap[childTwoIdx][1] < heap[childOneIdx][1]:
                idxToSwap = childTwoIdx
            else:
                idxToSwap = childOneIdx
            if heap[idxToSwap][1] < heap[currentIdx][1]:
                self.swap(currentIdx, idxToSwap, heap)
                currentIdx = idxToSwap
                childOneIdx = currentIdx * 2 + 1
            else:
                return

    def siftUp(self, currentIdx, heap):
        parentIdx = (currentIdx - 1) // 2
        while currentIdx > 0 and heap[currentIdx][1] < heap[parentIdx][1]:
            self.swap(currentIdx, parentIdx, heap)
            currentIdx = parentIdx
            parentIdx = (currentIdx - 1) // 2

    def remove(self):
        if self.isEmpty():
            return

        self.swap(0, len(self.heap) - 1, self.heap)
        vertex, distance = self.heap.pop()
        self.vertexMap.pop(vertex)
        self.siftDown(0, len(self.heap) - 1, self.heap)
        return vertex, distance

    def swap(self, i, j, heap):
        self.vertexMap[heap[i][0]] = j
        self.vertexMap[heap[j][0]] = i
        heap[i], heap[j] = heap[j], heap[i]

    def update(self, vertex, value):
        self.heap[self.vertexMap[vertex]] = (vertex, value)
        self.siftUp(self.vertexMap[vertex], self.heap)


class State:
    def __init__(self, n, vehicles):
        self.timestamp = 0
        self.nodeState = [0] * n
        self.vehicleStates = [VehicleState(vehicle) for vehicle in vehicles]

    def feasible(self):
        feasible = True
        for vehicleState in self.vehicleStates:
            feasible = feasible and vehicleState.canArrive
        return feasible

    def success(self):
        success = True
        for vehicleState in self.vehicleStates:
            success = success and vehicleState.arrived
        return success


class VehicleState:
    def __init__(self, vehicle):
        self.arrived = not vehicle.needsToBeCharged
        self.timestamp = vehicle.heuristicUnderestimate if self.arrived else 0
        self.info = vehicle
        self.charge = vehicle.initialBattery
        self.position = vehicle.source
        self.route = [str(vehicle.source + 1)]
        if self.arrived:
            self.route = vehicle.shortestRoute
        self.canArrive = True
        self.visitedMemo = set()
        self.visitedMemo.add(vehicle.source)

         # Check if initial position is a recharging station
        if self.position in recharging_stations:
            self.arrived = False
        else:
            self.arrived = not vehicle.needsToBeCharged

driver()
