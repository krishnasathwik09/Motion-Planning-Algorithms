# Basic searching algorithms
from collections import deque
import copy

class Node:
    def __init__(self,coordinate,parent):
        self.coordinate = coordinate
        self.parent = parent
        
def Path_Tracking(node,start):
    shortest_path = []
    # track path nodes from  Goal node parent to Start node  
    while node.parent is not None:
        shortest_path.append(list(node.coordinate))
        node = node.parent
    shortest_path.append(list(start)) #add start node to the path
    shortest_path.reverse() #Reverse list = since we are starting from goal node 
    return shortest_path  
   
def neighbour_nodes(current_node,grid,visited_nodes,DFS_flag):
    x_c,y_c = current_node.coordinate
    adjacent_list = [] #to store valid nodes
    #right,down,left,up
    neighbour_list = [(x_c,y_c+1),(x_c+1,y_c),(x_c,y_c-1),(x_c-1,y_c)]
    
    # If DFS function reverse the neighbour list 
    if DFS_flag == 1:
        neighbour_list.reverse()
    
    # check valid nodes 
    for (x,y) in neighbour_list:
            if 0<=x<len(grid) and 0<=y<len(grid[0]) and grid[x][y]!=1 and (x,y) not in visited_nodes:
                adjacent_list.append((x,y))
    return adjacent_list

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    goal = tuple(goal)
    
    start = tuple(start)
    start_node = Node(start,None)
    # Priority Queue - List of Node objects
    open_list = deque()
    open_list.append(start_node)
    # Visited list
    visited_nodes = set()
    visited_nodes.add(start)
    
    while True:
        explore_node = open_list.popleft() # Pop left node in QUEUE
        
        # Store explored/visited Nodes
        if explore_node.coordinate not in visited_nodes:
            visited_nodes.add(explore_node.coordinate)
            
        # Break the loop if Goal is reached     
        if explore_node.coordinate == goal:
            found = True
            steps = len(visited_nodes)
            path = Path_Tracking(explore_node,start)
            break
        
         # Add valid adjacent nodes to the Priority queue
        for (x,y) in neighbour_nodes(explore_node,grid,visited_nodes,0):
            open_list.append(Node((x,y),explore_node))

        if len(open_list) == 0:
            break

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    goal = tuple(goal)
    
    start = tuple(start)
    start_node = Node(start,None)
    # Priority stack - List of Node objects
    open_list = deque()
    open_list.append(start_node)
    # Visited list
    visited_nodes = set()
    visited_nodes.add(start)
    
    while True:
        explore_node = open_list.pop() #stack LIFO
        
        # Store explored/visited Nodes
        if explore_node.coordinate not in visited_nodes:
            visited_nodes.add(explore_node.coordinate)
        
        # Break the loop if Goal is reached
        if explore_node.coordinate == goal:
            found = True
            steps = len(visited_nodes) 
            path = Path_Tracking(explore_node,start)
            break
        # Add valid adjacent nodes to the Priority stack
        for (x,y) in neighbour_nodes(explore_node,grid,visited_nodes,1):
            open_list.append(Node((x,y),explore_node))

        if len(open_list) == 0:
            break

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    goal = tuple(goal)
    
    start = tuple(start)
    start_node = Node(start,None)
    # Priority Queue - List of Node objects
    open_list = deque()
    open_list.append(start_node)
    # Visited list
    visited_nodes = set()
    visited_nodes.add(start)
    
    while True:
        explore_node = open_list.popleft()  # Pop left node in QUEUE
        
        # Store explored/visited Nodes
        if explore_node.coordinate not in visited_nodes:
            visited_nodes.add(explore_node.coordinate)
            
        # Break the loop if Goal is reached   
        if explore_node.coordinate == goal:
            found = True
            steps = len(visited_nodes)
            # Extract shortest path by tracking parent node from Goal node
            path = Path_Tracking(explore_node,start)
            break
        
        # create adjacent nodes list for each explored node
        dj_adjacent_node_list = []
        
        #calculate manhattan distance from Current node to goal node
        for (x,y) in neighbour_nodes(explore_node,grid,visited_nodes,0):
            #heuristic function-Manhattan distance
           distance = abs(x - goal[0]) + abs( y - goal[1])
           dj_adjacent_node_list.append(((x,y) , distance))
        
        # Sort adjacent list based on Distance heuristic
        dj_adjacent_node_list.sort(key=lambda dist: dist[1])
       
        #Add sorted adjacent nodes to the priority queue
        for (x,y),z in dj_adjacent_node_list:
            if not (x, y) in visited_nodes:
                open_list.append(Node((x,y) , explore_node))
            
        # break the loop if the priority queue is empty     
        if len(open_list) == 0:
           break
    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    goal = tuple(goal)
    start = tuple(start)
    start_node = Node(start,None)
    
    # Priority Queue - List of Node objects
    open_list_dist = deque()
    open_list_dist.append((start_node,0))
    
    # Visited list
    visited_nodes = set()
    visited_nodes.add(start)
    
    while True:
       explore_node_dist = open_list_dist.popleft()  # Pop left node in QUEUE
       explore_node = explore_node_dist[0]
        
       # Store explored/visited Nodes
       if explore_node.coordinate not in visited_nodes:
            visited_nodes.add(explore_node.coordinate)
            
        # Break the loop if Goal is reached   
       if explore_node.coordinate == goal:
            found = True
            steps = len(visited_nodes)
            # Extract shortest path by tracking parent node from Goal node
            path = Path_Tracking(explore_node,start)
            break
       
       # Distance from start to current node = Sum of distances from current node and Parent node (till start node)
       dis_node = copy.copy(explore_node)
       distance_start = 0
       while dis_node.parent is not None:
           distance_start += 1
           dis_node = dis_node.parent
           
       # create adjacent nodes list for each explored node
       astar_adjacent_node_list = []
        
       for (x,y) in neighbour_nodes(explore_node,grid,visited_nodes,0):
           
           # Manhattan distance -Heuristic function
           distance_goal = abs(x - goal[0]) + abs( y - goal[1])
           
           # A star - Distance from start + Distance from Goal
           total_distance = distance_start + distance_goal
           astar_adjacent_node_list.append(((x,y) , total_distance))
               
       # sort adjacent node list based on distance heuristic
       astar_adjacent_node_list.sort(key=lambda dist: dist[1])
       
       # Push sorted adjacent node list to Queue
       for (x,y),dis in astar_adjacent_node_list:
           open_list_dist.append((Node((x,y) , explore_node),dis))
               
       open_list_dist = deque(sorted(open_list_dist,key=lambda z: z[1]))
       
       if len(open_list_dist)==0:
           break
    
    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps



# # Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
