# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost
        


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.extend_dist = 10
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        distance = np.sqrt((node1.row-node2.row)**2 + (node1.col-node2.col)**2)
        return distance

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        n1 = np.array([node1.row,node1.col])
        n2 = np.array([node2.row,node2.col])
        d = n2-n1
        M = max(abs(d))
        if M == 0:
            return True
        s = d/M
        for j in range(0,M):
            n1 = n1 + s
            if int(n1[1]) >=300:
                n1[1]=299
            if int(n1[0]) >=300:
                n1[0]=299
            if not self.map_array[int(n1[0]),int(n1[1])]:
                return True
        return False

    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        if np.random.random() < goal_bias:
            return [self.goal.col, self.goal.row]
        else:
            x_pt = np.random.randint(0,self.size_row-1)
            y_pt = np.random.randint(0,self.size_col-1)
            return [x_pt,y_pt]

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        samples =[]
        for ver in self.vertices:
            samples.append((ver.row, ver.col))
            
        kdtree = spatial.KDTree(samples)
        _, indic = kdtree.query(point,k=1)
        return self.vertices[indic]


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        samples_pt = []
        for ver in self.vertices:
            samples_pt.append((ver.row, ver.col))
        #using Kd tree find near nodes within neighbor_size  
        kdtree = spatial.KDTree(samples_pt)
        ind = kdtree.query_ball_point((new_node.row, new_node.col), neighbor_size)
        neighbors = []
        for i in ind:
            neighbors.append(self.vertices[i])

        
        return neighbors

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        # calculate minimum cost
        for idx,node in enumerate(neighbors):
            if self.dis(new_node,node)+ node.cost < self.minimum_cost:
                self.less_node  = neighbors[idx]
                self.minimum_cost    = self.dis(new_node,node)+ node.cost
        #chnage parents of the node path which has less cost
        if self.check_collision(new_node,self.less_node) == False:
            new_node.parent = self.less_node
            new_node.cost   = self.less_node.cost + self.dis(new_node,self.less_node)
            self.vertices.append(new_node)
            for node in neighbors:
                if self.check_collision(new_node,node) == False and (self.dis(new_node,node) + new_node.cost) < node.cost:
                    node.parent = new_node
                    node.cost   = new_node.cost + self.dis(new_node,node)
       
        

    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###
        for i in range(n_pts):
            new_point = self.get_new_point(0.05)
            near_node = self.get_nearest_node(new_point)
            slope = np.arctan2(-1*(near_node.col-new_point[1]), -1*(near_node.row-new_point[0]))
            new_node_x = int(near_node.row + (self.extend_dist * np.cos(slope)))
            new_node_y = int(near_node.col + (self.extend_dist * np.sin(slope)))
            new_extend_node = Node(new_node_x,new_node_y)
            if self.check_collision(near_node, new_extend_node) is True:
                continue
            if (0 <= new_node_x < self.size_row) and (0 <= new_node_y < self.size_col) and \
                not self.check_collision(near_node, new_extend_node):
                    
                new_extend_node.parent = near_node
                new_extend_node.cost = near_node.cost + self.dis(new_extend_node,near_node)
                self.vertices.append(new_extend_node)
                if self.found is not True:
                    d = self.dis(new_extend_node,self.goal)
                    if d < self.extend_dist and not self.check_collision(self.goal, new_extend_node):
                        self.found = True
                        self.goal.parent = new_extend_node
                        self.goal.cost = new_extend_node.cost + d
                        self.vertices.append(self.goal)
                        break

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()

    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()
        self.neighbor_size = neighbor_size
        ### YOUR CODE HERE ###
        for i in range(n_pts):
            new_point = self.get_new_point(0.05)
            near_node = self.get_nearest_node(new_point)
            slope = np.arctan2(-1*(near_node.col-new_point[1]), -1*(near_node.row-new_point[0]))
            new_node_x = int(near_node.row + (self.extend_dist * np.cos(slope)))
            new_node_y = int(near_node.col + (self.extend_dist * np.sin(slope)))
            new_extend_node = Node(new_node_x,new_node_y)
            if self.check_collision(near_node, new_extend_node) is True:
                continue
            if (0 <= new_node_x < self.size_row) and (0 <= new_node_y < self.size_col) and \
                not self.check_collision(near_node, new_extend_node):
                    neighbors   = self.get_neighbors(new_extend_node,neighbor_size)
                    self.less_node  = near_node
                    self.minimum_cost    = self.dis(new_extend_node,near_node) + near_node.cost
                    self.rewire(new_extend_node,neighbors)
                    
                    d = self.dis(new_extend_node,self.goal)
                    if d < self.extend_dist and not self.check_collision(self.goal, new_extend_node):
                        self.found = True
                        self.goal.parent = new_extend_node
                        self.goal.cost = new_extend_node.cost + d
                        self.vertices.append(self.goal)
                           
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
