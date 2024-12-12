# Trip Planner

A trip planning API that provides routing and searching services for locations and points of interest (POIs) on a map. The system uses graph algorithms and efficient data structures to enable path finding and location-based queries.

## Features

- **Location Search**: Find all locations matching a specific category (e.g., restaurants, banks)
- **Route Planning**: Calculate shortest paths between locations
- **Nearby POI Search**: Find nearest points of interest by category within a specified limit

## Technical Details

### Data Structures

The implementation uses several key data structures:

- **Weighted Undirected Graph** (Adjacency Matrix)
  - Represents road segments and connections between positions
  - Vertices represent positions
  - Edges represent roads
  - Edge weights represent distances between positions
  - O(1) edge operations with space tradeoff

- **Hash Tables** for:
  - Mapping positions to vertices
  - Mapping POI names to vertices 
  - Mapping positions to POI categories
  - Provides O(1) lookups and insertions

- **Direct Addressing Vector** for:
  - Mapping vertices to positions
  - O(1) operations with optimal space usage for integer keys

- **Binary Heap Priority Queue**
  - Used in Dijkstra's Algorithm implementation
  - O(log n) insert and remove_min operations

### Algorithms

#### Dijkstra's Algorithm
- Used for shortest path calculations in route planning
- Time complexity: O(|V|^2 log|V|)
- Chosen over Bellman-Ford for better performance with positive weights

#### Heapsort
- Used for sorting POIs by distance in nearby searches
- Provides efficient sorting with custom comparator support

## API Reference

### Constructor

```Racket
TripPlanner(road_segments: VecC[RawSeg?], points_of_interest: VecC[RawPOI?])
```

Initializes a new trip planner with:
- `road_segments`: Vector of raw road segments (4-element vectors with [lat1, lon1, lat2, lon2])
- `points_of_interest`: Vector of raw POIs (4-element vectors with [lat, lon, category, name])

### Methods

#### locate_all
```Racket
def locate_all(self, category: str?) -> ListC[RawPos?]
```
Returns positions of all POIs in the given category. Results have no duplicates.

#### plan_route
```Racket
def plan_route(self, start: RawPos?, destination: str?) -> ListC[RawPos?]
```
Finds shortest path from start position to named destination. Returns empty list if unreachable.

#### find_nearby
```Racket
def find_nearby(self, start: RawPos?, category: str?, limit: nat?) -> ListC[RawPOI?]
```
Returns up to `limit` closest POIs in given category from start position.

## Input/Output Types

- **RawPos**: `[latitude, longitude]`
- **RawSeg**: `[lat1, lon1, lat2, lon2]`
- **RawPOI**: `[latitude, longitude, category, name]`

All positions are represented as 2-element vectors with latitude and longitude.

## Assumptions

1. All roads are two-way
2. Road segment length is Euclidean distance between endpoints
3. POIs can only be located at road segment endpoints
4. Starting positions for queries are at road segment endpoints

## Performance Considerations

- Graph operations use adjacency matrix for O(1) edge queries
- Hash tables provide O(1) lookups for position/POI mappings
- Dijkstra's Algorithm with priority queue for efficient pathfinding
- Results may contain ties resolved arbitrarily for nearby POI searches

## Dependencies

Required libraries (provided in project-lib.zip):
- stack-queue.rkt
- dictionaries.rkt 
- graph.rkt
- binheap.rkt

## Usage Example

```Racket
# Create trip planner
road_segments = [[0,0, 0,1], [0,1, 1,1]]  # Two connected road segments
pois = [[0,0, "food", "Restaurant"], [1,1, "bank", "ATM"]]
planner = TripPlanner(road_segments, pois)

# Find all food locations
food_locations = planner.locate_all("food")

# Plan route from start to destination
route = planner.plan_route([0,0], "ATM")

# Find 3 closest food places
nearby = planner.find_nearby([0,0], "food", 3)
```
