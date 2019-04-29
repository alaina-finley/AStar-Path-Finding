# @author Alaina Finley

import numpy as np
from heapq import heappush, heappop
import matplotlib
matplotlib.use('TKAgg')
from animation import draw
import argparse


class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class Maze():
    
    def __init__(self, map, start_state, goal_state, map_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.map = map
        self.visited = [] # state
        self.m, self.n = map.shape 
        self.map_index = map_index


    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
    
        draw(self.map, path[::-1], self.map_index)


    def goal_test(self, current_state):        
        if np.array_equal(current_state, self.goal_state) is True:
            return True
        else:
            return False


    def get_cost(self, current_state, next_state):        
        return 1
        


    def get_successors(self, state):
        successors = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
        newState = state
        for position in directions:
            row, column = newState
            nodePosition = (row + position[0], column + position[1])
            
            if int(nodePosition[0]) > (len(self.map)-1) or nodePosition[0] < 0 or nodePosition[1] > len(self.map[len(self.map)-1]) or nodePosition[1] <0:
                continue

            if self.map[nodePosition[0]][nodePosition[1]] == 0:
                continue

            self.map[nodePosition] = 0.5
            successors.append(nodePosition)
        
        return successors

    # heuristics function
    def heuristics(self, state):
        hCost = ((state[0]- self.goal_state[0])**2)+((state[1]- self.goal_state[1])**2)
        return hCost


    # priority of node 
    def priority(self, node):
        priority = node.cost_from_start + self.heuristics(node.state)
        return priority
        
    # solve it
    def solve(self):
        state = self.start_state
        currentNode = Node(state,0, None)
        count = 1
        self.visited.append(state)
        
        openList = []
        heappush(openList, (self.priority(currentNode), count, currentNode))

        while openList:
            currentNode = heappop(openList)[2]

            if self.goal_test(currentNode.state) is True:
                self.draw(currentNode)

            successors = self.get_successors(currentNode.state)

            for nextState in successors:
                isVisited = False

                for element in self.visited:
                    if np.array_equal(nextState, element):
                        isVisited = True
                
                if isVisited is False:
                    gCost = currentNode.cost_from_start + self.get_cost(currentNode.state, nextState)                    
                    nextNode = Node(nextState, gCost, currentNode)

                    if self.goal_test(nextState) is True:
                        self.draw(nextNode)
                        return 

                    if nextNode not in openList:
                        heappush(openList, (self.priority(nextNode), count, nextNode))
                        count += 1
                    elif currentNode.cost_from_start >= nextNode.cost_from_start:
                        continue

                    nextNode.cost_from_start =gCost
                
                    self.visited.append(currentNode.state)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='maze')
    parser.add_argument('-index', dest='index', required = True, type = int)
    index = parser.parse_args().index

    # Example:
    # Run this in the terminal solving map 1
    #     python maze_astar.py -index 1
    
    data = np.load('map_'+str(index)+'.npz')
    
    map, start_state, goal_state = data['map'], tuple(data['start']), tuple(data['goal'])

    game = Maze(map, start_state, goal_state, index)
    game.solve()
    