from numpy import genfromtxt

class Node:
    def __init__(self, index, parent, goal):
        self.index = index
        self.parent = parent
        if parent is not None:
            self.g_score = parent.g_score+1#Cost to get to this node
        else:
            self.g_score = 0
        self.f_score = self.g_score + self.costEstimate(goal)


    #Overloading the == operator
    def __eq__(self, other):
        return self.index == other.index


    def costEstimate(self, goal):
        xDiff = goal[0]-self.index[0]
        yDiff = goal[1]-self.index[1]
        return xDiff+yDiff #Return the manhattan heuristic


    def update(self, node):
        if node.f_score < self.f_score:
            self.parent = node.parent
            self.f_score = node.f_score


class AStar:
    def __init__(self, start, goal):
        self.world = genfromtxt('world.csv', delimiter=',') #Load the world data
        self.start = Node(start, None, goal)
        self.goal = goal


    def run(self):
        self.closedSet = [] #Nodes already searched
        self.openSet = [self.start] #Open nodes
        while len(self.openSet) >= 1:
            self.openSet.sort(key = lambda node: node.f_score, reverse=True)
            self.traverse(self.openSet.pop())
        gNode = Node(self.goal, None, self.goal)
        i = self.closedSet.index(gNode)
        node = self.closedSet[i]
        path = []
        while True:
            path.append(node)
            if node.parent is not None:
                node = node.parent
            else:
                break
        self.path = path #The path navigated


    def traverse(self, node):
        self.closedSet.append(node)
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
                    n = self.openSet.index(neighbor)
                    self.openSet[n].update(neighbor)
                else:
                    self.openSet.append(neighbor)
                    print 'adding {}to Openset'.format(index)


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