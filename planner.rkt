#lang dssl2

# Final project: Trip Planner

#Honor Code
let eight_principles = ["Know your rights.",
    "Acknowledge your sources.",
    "Protect your work.",
    "Avoid suspicion.",
    "Do your own work.",
    "Never falsify a record or permit another person to do so.",
    "Never fabricate data, citations, or experimental results.",
    "Always tell the truth when discussing your work with your instructor."]

##ACKNOWLEDGEMENT OF EXTERNAL SOURCES (E.G HW CODE OR LECTURE CODE)      
import cons
import sbox_hash

#Import files from project library as follows:
import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'


### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?

interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs     
                      
struct dist_pred:
    let distances
    let predecessors

#Trip Planner Class                    
class TripPlanner (TRIP_PLANNER):
    # points of interest
    let points
    # road segments
    let roads
    # WU graph
    let map
    # dictionary from positions to vertices
    let p2v
    # dictionary from vertices to positions
    let v2p
    # dictionary from poi names to vertices
    let name2vert
   
  
    def __init__(self, road_segs, pois):
        #points of iterest
        self.points = pois
        
        #road segments
        self.roads = road_segs
        
        #maximum number of positions
        let max_pos = 2 * len(self.roads)
        
        #dictionary from positions to vertices (Hash Table)
        self.p2v = HashTable(max_pos, make_sbox_hash())
        
        let counter = 0
        let positions = [None;max_pos]
        for road in self.roads:
            let p1 = [road[0], road[1]]
            let p2 = [road[2], road[3]]
            
            if not self.p2v.mem?(p1):
                self.p2v.put(p1, counter)
                positions[counter] = p1
                counter = counter + 1
                
            if not self.p2v.mem?(p2):
                self.p2v.put(p2, counter)
                positions[counter] = p2
                counter = counter + 1
        
        #dictionary from vertices to positions (direct addressing array)
        self.v2p = [None; counter]
        
        for i in range(counter):
            self.v2p[i] = positions[i]
            
        #println(positions)
        #println(self.v2p)
                
        #WU graph
        self.map = WuGraph(counter)
        
        for road in self.roads:
            
            let p1 = [road[0], road[1]]
            let p2 = [road[2], road[3]]
            
            let a = (road[3] - road[1])*(road[3] - road[1])
            let b = (road[2] - road[0])*(road[2] - road[0])
            let w =(a + b).sqrt()
            
            let v1 = self.p2v.get(p1)
            let v2 = self.p2v.get(p2)
            
            self.map.set_edge(v1, v2, w)
        
        # poi names to vertices
        self.name2vert = HashTable(len(self.points), make_sbox_hash())
        
        for poi in self.points:
            let v = self.p2v.get([poi[0],poi[1]])
            self.name2vert.put(poi[3], v)

    # Query methods
            
    #A. locate_all       
    def locate_all(self, dst_cat):
        #initialize a dictionary mapping positions to poi categories
        let pos2cat = HashTable(self.map.len(), make_sbox_hash())
        #initialize linked list
        let all_list = None
        for p in self.points:
            #get position of poi
            let pos = [p[0], p[1]]
            #check of position has been visited
            if pos2cat.mem?(pos):
                continue
            #check poi category
            if p[2] == dst_cat:
                all_list = cons(pos, all_list)
                pos2cat.put(pos, dst_cat)   
        return all_list
        
    #B. plan_route helper function (Dijkstra's Algorithm)
    def get_shortest_path(self, start_pos):
        
        #1. store self.map graph in a variable
        let graph = self.map
        
        #2. convert start pos to start vertex using self.pos_to_node
        let start = self.p2v.get(start_pos)
       
        #3. intiialize a dist_pred table struct -> to be returned
        let da_table = dist_pred([inf;graph.len()],[None;graph.len()]) 
        da_table.distances[start] = 0
      
        #4. initialize todo priority queue   
        let todo = BinHeap((graph.len()**2), λ x, y: x[0] < y[0])
       
        let vec = [0, start]
        todo.insert(vec) 

        #5. Dijkstra's algorithm
        while todo.len() > 0:
            let nv = todo.find_min()[1] #get vertex associated with minimum distance from start
            todo.remove_min()
            let vertices = graph.get_adjacent(nv)
            let curr = vertices 

            while curr != None:
                let w = graph.get_edge(curr.data, nv)
                    
                if da_table.distances[nv] + w < da_table.distances[curr.data]:
                    da_table.distances[curr.data] = da_table.distances[nv] + w
                    da_table.predecessors[curr.data] = nv
                    todo.insert([da_table.distances[curr.data], curr.data])
                    curr = curr.next
                else:
                    curr = curr.next
                        
        #return table for calculating shortest path               
        return da_table 
    
    #C. plan_route      
    def plan_route(self, src_lat, src_lon, dst_name):
        
        #1. Check that dst_name is actually in the set of POIs and return None if not
        if not self.name2vert.mem?(dst_name):
            return None
            
        #2. let pos = [src_lat, src_lon]
        let pos = [src_lat, src_lon]
        
        #3. call self.get_shortest_path(pos)
        let all_paths = self.get_shortest_path(pos)
        
        #4. find destination vertex
        let dst_v = self.name2vert.get(dst_name)
                             
        #5. create a linked list of vertices and convert back to positions
        let v_route = None

        while int?(dst_v): 
            v_route = cons(dst_v, v_route)
            dst_v = all_paths.predecessors[dst_v] 
            
        let route = Cons.map(λ x: self.v2p[x], v_route)
        
        
        #6. check if end point is accessible from starting pos and return None if not
        if route.data == pos:
            return route 
           
        return None
        
    #D. find_nearby  
    def find_nearby(self, src_lat, src_lon, dst_cat, n):
        
        #1. make a cons list of pois in the given category
        let start_pos = [src_lat, src_lon]
        let poi_positions = Cons.to_vec(self.locate_all(dst_cat))
        let pois_in_cat = None
        for poi in self.points:
            let poi_pos = [poi[0], poi[1]]
            for pos in poi_positions:
                if poi_pos == pos:
                    pois_in_cat = cons(poi, pois_in_cat)
        
        #2. convert cons list to array
        let pois_in_cat_vec = Cons.to_vec(pois_in_cat)
        
                       
        #3. call dijkstras from starting position and access the distances array
        let dijkstra = self.get_shortest_path(start_pos)
        let distances = dijkstra.distances
         
        #4. find_nearby helper (heap sort comparator) ->  converts the positions of pois
        #to vertices and uses these vertices to index the distance array from dijkstra's; returns true of
        #if the distance corresponding to poi 1 < the distance corresponding to poi 2 
        def poi_dist_comp(poi_1, poi_2):
            let v1 = self.p2v.get([poi_1[0], poi_1[1]])
            let v2 = self.p2v.get([poi_2[0], poi_2[1]])
            let dist1 = distances[v1] 
            let dist2 = distances[v2]
        
            return dist1 < dist2
            
        #5. call heapsort on the array of pois and the helper function
        heap_sort(pois_in_cat_vec, poi_dist_comp)
        
        #6. account for unreachable points
        let counter = 0
        for dist in distances:
            if dist == inf:
                continue
            counter = counter + 1
        
        let reachable_pois = [None; counter]
        let i = 0
        
        for poi in pois_in_cat_vec:
            let v = self.p2v.get([poi[0], poi[1]])
            if distances[v] == inf:
                continue
            reachable_pois[i] = poi
            i = i + 1
        
        #7. prepare output
        let pos_in_cat_cons = None
        let j = 0
        #println(reachable_pois)
        while j < n:
            if reachable_pois[j] == None:
                j = j+1
                continue
            pos_in_cat_cons = cons(reachable_pois[j], pos_in_cat_cons)
            j = j + 1
        return pos_in_cat_cons
        
        
#### GENERATE SOME TRIP PLANNERS ####
def pdf_example():
    return TripPlanner([[0,0, 0,1], 
                        [0,0, 1,0], 
                        [0,1, 1,1], 
                        [0,1, 0,2], 
                        [1,0, 1,1],
                        [1,1, 1,2],
                        [0,2, 1,2],
                        [1,2, 1,3],
                        [1,3, -0.2,3.3]],
    
                       [[0,0, "food", "Sandwiches"],
                        [0,1, "food", "Pasta"],
                        [0,1, "clothes", "Pants"],
                        [1,1, "bank", "Local Credit Union"],
                        [-0.2,3.3, "food", "Burritos"],
                        [1,3, "bar", "Bar None"],
                        [1,3, "bar", "H Bar"]])        
                
def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pelmeni"]])
                        
def my_second_example():
    return TripPlanner([[0,0, 1,0],[1,0, 1,1],[1,1, 1,2]], 
                                [[1,0, "food", "Whole Foods"],
                                [1,1, "landmark", "Frances Willard House"]])
                              
def my_third_example(): #make various distances that can be easily sorted
    return TripPlanner([[0,0, 2,4],[2,4, 1,7],[1,7, 1,2], [0,0, 1,2]], 
                                [[2,4, "shopping", "Target"],
                                [1,2, "landmark", "Frances Willard House"]])
                                                             
def large_lat_lon():
    return TripPlanner([[9.9,10, 0,1], [0,1, 100,20]],
                        [[9.9,10, "bar", "The Empty Bottle"],
                        [0,1, "bar", "Mamita"]])
                                            
                                               
#### TEST LOCATE ALL ####
                           
test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)


test 'Basic Locate All 1: Construct PDF Example Map':
    let ex_tp = pdf_example()
    assert ex_tp.locate_all("food") == cons([-0.2, 3.3], cons([0,1], cons([0,0], None)))
    assert ex_tp.locate_all("clothes") == cons([0,1], None)
    assert ex_tp.locate_all("bank") == cons([1,1], None)
    assert ex_tp.locate_all("bar") == cons([1,3], None) 
    
test 'Locate All 2':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
     
    assert tp.locate_all("barber") == cons([5,0], cons([3,0], None))
    assert tp.locate_all("bank") == cons([1.5,0], None)
    assert tp.locate_all("food") == cons([4,0], None)
    
test 'Locate All 3 (Advanced)':
    let tp_large =large_lat_lon()
    assert tp_large.locate_all("bar") == cons([0,1], cons([9.9,10], None))
   
  
#### TEST get shortest path ####
test 'dijkstra helper':
    let p = my_third_example()
    let d = p.get_shortest_path([0,0])
    
    assert d == dist_pred {distances: [0, 4.47213595499958, 7.23606797749979, 2.23606797749979], predecessors: [None, 0, 3, 0]}
    
    let p2 = my_first_example()
    let d2 = p2.get_shortest_path([0,0])
    
    assert d2 == dist_pred {distances: [0, 1, 1], predecessors: [None, 0, 0]}
    
    
#### TEST PLAN ROUTE ####
test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pelmeni") == \
       cons([0,0], cons([0,1], None))

test 'Basic Route Test':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
       
    let result = tp.plan_route(3, 0, 'Union')
    assert Cons.to_vec(result)  == [[3, 0], [2.5, 0], [1.5, 0]] 

test'Basic Route Test 2':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) == [[0, 0], [1.5, 0], [2.5, 0], [3, 0]]        
    
test 'Advanced Route Test 1':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.plan_route(0, 0, 'Judy')
    assert Cons.to_vec(result) == []   
    
test 'Advanced Route Test 2':
     let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'food', 'Jollibee']]) 
          
     let result = tp.plan_route(0, 0, 'Judy')
     assert Cons.to_vec(result) == [[0, 0], [1.5, 0], [2.5, 0], [3, 0], [4, 0], [5, 0]]
     
     let result2 = tp.plan_route(0, 0, 'Jollibee')
     assert Cons.to_vec(result2) == [[0, 0], [1.5, 0], [2.5, 0], [3, 0], [4, 0], [5, 0]]
  
test 'Advanced Route Test 3':
    let tp = TripPlanner(
      [[0, 0, 0, 1],
       [0, 1, 3, 0],
       [0, 1, 4, 0],
       [0, 1, 5, 0],
       [0, 1, 6, 0],
       [0, 0, 1, 1],
       [1, 1, 3, 0],
       [1, 1, 4, 0],
       [1, 1, 5, 0],
       [1, 1, 6, 0],
       [0, 0, 2, 1],
       [2, 1, 3, 0],
       [2, 1, 4, 0],
       [2, 1, 5, 0],
       [2, 1, 6, 0]],
      [[0, 0, 'blacksmith', "Revere's Silver Shop"],
       [6, 0, 'church', 'Old North Church']])
       
    let result = tp.plan_route(0, 0, 'Old North Church')
    
    assert Cons.to_vec(result) == [[0, 0], [2, 1], [6, 0]]
    
test 'Advanced Route Test 4':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[0, 0, 'bank', 'Union']])
    let result = tp.plan_route(0, 0, 'Union')
    assert Cons.to_vec(result) == [[0, 0]]

test 'BFS is not SSSP (route)':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.plan_route(0, 0, 'Cem')
    assert Cons.to_vec(result)  == [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9]]    
    
    
        
test 'Edge case: a path with a self edge':
    let tp = TripPlanner(
      [[0, 0, 0, 0],
       [0, 0, 1, 0],
       [1, 0, 0.5, 1],
       [2, 1, 0.5, 1],
       [2, 1, 1, 0]],
      [[0, 0, 'food', "Bob's Pizza"],
       [2, 1, 'church', "Old North Church"]])
       
    let result = tp.plan_route(0, 0, "Bob's Pizza")
    
    assert Cons.to_vec(result) == [[0, 0]]    

test 'Two equivalent route':
    let tp = TripPlanner(
      [[-2, 0, 0, 2],
       [0, 2, 2, 0],
       [2, 0, 0, -2],
       [0, -2, -2, 0]],
      [[2, 0, 'cooper', 'Dennis']])
    let result = tp.plan_route(-2, 0, 'Dennis')
    assert Cons.to_vec(result) \
      in [[[-2, 0], [0, 2], [2, 0]],
          [[-2, 0], [0, -2], [2, 0]]]           

#### TEST FIND NEARBY ####
test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pelmeni"], None)
        
test 'Basic find_nearby 1':
   let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[1, 0, 'bank', 'Union']])
      
   let result = tp.find_nearby(0, 0, 'bank', 1)
   assert Cons.to_vec(result) == [[1, 0, 'bank', 'Union']]

   
test 'Basic find_nearby 2':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.find_nearby(0, 0, 'barber', 1)
    assert Cons.to_vec(result) == [[3, 0, 'barber', 'Tony']] 
    

test 'Basic find_nearby 3':  
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.find_nearby(3, 0, 'bank', 1)
    assert Cons.to_vec(result) == [[1.5, 0, 'bank', 'Union']]
    
test 'Basic find_nearby 4': 
   let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
   let result = tp.find_nearby(0, 0, 'barber', 2)
   assert Cons.to_vec(result) == [[3, 0, 'barber', 'Tony']]
   
test 'Advanced find_nearby 1':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    assert Cons.to_vec(result) == []
    
test 'Advanced find_nearby 2': 
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    assert Cons.to_vec(result) == []
    
test 'Advanced find_nearby 3': 
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.find_nearby(0, 0, 'haberdasher', 2)
    assert Cons.to_vec(result) == [[8, 8, 'haberdasher', 'Braden'], [7, 7, 'haberdasher', 'Archit']]
       
test 'Advanced find_nearby 4': 
    let tp = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    let result = tp.find_nearby(-1.1, -1.1, 'barber', 1)
    assert Cons.to_vec(result) == [[3, 4, 'barber', 'Tony']]
       
test 'Advanced find_nearby 5': 
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 3)
    assert Cons.to_vec(result) == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]

test 'Advanced find_nearby 6': 
    let tp = TripPlanner(
      [[-1, -1, 0, 0],
       [0, 0, 3.5, 0],
       [0, 0, 0, 3.5],
       [3.5, 0, 0, 3.5]],
      [[-1, -1, 'food', 'Jollibee'],
       [0, 0, 'bank', 'Union'],
       [3.5, 0, 'barber', 'Tony'],
       [0, 3.5, 'barber', 'Judy']])
    let result = tp.find_nearby(-1, -1, 'barber', 1)
    Cons.to_vec(result)  == [3.5, 0, 'barber', 'Tony']    
       
test 'Advanced find_nearby 7': 
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[5, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]
      
test 'Advanced find_nearby 8': 
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [0, 0, 'barber', 'Lily'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(2.5, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[0, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]
       
test 'Advanced find_nearby 9': 
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]