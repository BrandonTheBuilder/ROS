
__all__ = ['Node']

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
        #Check if the new node has a lower f_score
        if node.f_score < self.f_score:
            self.parent = node.parent
            self.f_score = node.f_score

