from numpy import genfromtxt
from Node import Node

__all__ = ['AStar']

class AStar:
    def __init__(self, start, goal):
        #Translate the start and goal coordinates into indices
        self.world = genfromtxt('world.csv', delimiter=',') #Load the world data
        self.start = Node(start, None, goal) #Set the starting node
        self.goal = goal


    def run(self):
        self.closedSet = [] #Nodes already searched
        self.openSet = [self.start] #Open nodes
        while len(self.openSet) >= 1: #While there are nodes unsearched
            #Sort the open nodes by f_score
            self.openSet.sort(key = lambda node: node.f_score, reverse=True)
            #Open the node with the lowest f_score
            self.traverse(self.openSet.pop())
        gNode = Node(self.goal, None, self.goal)
        i = self.closedSet.index(gNode)
        node = self.closedSet[i]
        path = []
        #Work backwards to solve the path traversed
        while True:
            path.append(node)
            if node.parent is not None:
                node = node.parent
            else:
                break
        path.reverse() #Reverse the list to start at the beggining
        self.path = path #The path navigated


    def traverse(self, node):
        #Add the traversed node to the closed set
        self.closedSet.append(node)
        #Open all four nodes surounding current node.
        for i in range(0,4):
            if i == 0:
                index = (node.index[0], node.index[1]+1)
            elif i == 1:
                index = (node.index[0]+1, node.index[1])
            elif i == 2:
                index = (node.index[0], node.index[1]-1)
            elif i == 3:
                index = (node.index[0]-1, node.index[1])
            neighbor = Node(index, node, self.goal)
            if self.isValid(neighbor):
                if neighbor in self.openSet:
                    #If the neighbor is in the openSet update it's f_score
                    n = self.openSet.index(neighbor)
                    self.openSet[n].update(neighbor)
                else:
                    #If it is not in the oppen set add it
                    self.openSet.append(neighbor)


    def isValid(self, node):
        x = node.index[0]
        y = node.index[1]
        if node in self.closedSet:
            # print '{},{} is in closedSet'.format(x,y)
            return False
        if x<20 and x>=0 and y<20 and y>=0:
            if self.world[y][x] == 0:
                return True
            else:
                # print '{},{} is occupied'.format(x,y)
                return False
        # print '{},{} is not in this world'.format(x,y)
        return False