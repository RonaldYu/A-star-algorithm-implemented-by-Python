# A* algorithm for making features
# Calculate the distance between two point
# This code was written by Cheng-Yuan Yu


def hv_distance(x, y):
    return abs(x[0] - y[0]) + abs(x[1] - y[1])


# Following this class, we create the object(node) for using a* algorithm
class Node:

    def __init__(self, current_point):
        # node's parent node
        self.parent = None
        # node's coordinate
        self.node = current_point
        # g is the cost of the path from the start node to here
        self.g = 0
        # h is a heuristic that estimates the cost of the cheapest path from
        # here to the goal.
        self.h = 0
        # f = g + h
        self.f = 0
        # node's children's node
        self.children = []

    # method to calculate h cost of the node
    def calculcate_h(self, end_point):
        self.h = hv_distance(self.node, end_point)

    # method to calculate g cost of the node
    def calculate_g(self):
        self.g = self.parent.g + hv_distance(self.node, self.parent.node)

    # method to calculate f cost of the node
    def calculate_f(self):
        self.f = self.h + self.g


# execute a* algorithm
# Input:
#     current_node: the object Node
#     end_point: tuple, (x, y), the coordinate of the destination
#     obstacles: the list of coordinates (x, y), which is not allowed to pass
#     open_set: the set of Node's objects, which are candidate we are going to find deeply
#     closed_set: the set of Node's objects, which are selected from open_set
# Output:
#     the set of Node's objects which we will get optimal path from
def A_Star(current_node, end_point, obstacles, open_set, closed_set):
    # extract the current coordinate from the object Node
    current_point = current_node.node
    # computing the neighbors of the current
    candidates = list(set([(current_point[0] + i, current_point[1] + j) for i in [-1, 0, 1]
                           for j in [-1, 0, 1] if abs(i) + abs(j) == 1]) - set(obstacles))
    # find the boundary of the map
    boundary = [(min(i), max(i)) for i in zip(*obstacles)]
    # filter some candidates which are not in the map
    candidates = [i for i in candidates if i[0] > boundary[0][0] and i[0]
                  < boundary[0][1] and i[1] > boundary[1][0] and i[1] < boundary[1][1]]

    # Don't need to find the point which has been in the close list
    candidates = set(candidates) - set([i.node for i in closed_set])

    # If it meet the destination or there is no other point in the open set,
    # it stop finding deeply.
    # if current_point != end_point and len(open_set) > 0:
    if len(open_set) > 0:
        try:
            # looking for each candidate
            for element in candidates:
                # make it the object of Node
                candidate_node = Node(element)
                # set its parent
                candidate_node.parent = current_node
                # calculate its h
                candidate_node.calculcate_h(end_point)
                # calculate its g
                candidate_node.calculate_g()
                # calculate its f
                candidate_node.calculate_f()

                # if a node with the same position as candidate is in the open set which has a lower f than candidate, skip this candidate
                # if a node with the same position as candidate is in the
                # closed set which has a lower f than successor, skip this
                # candidate
                criterion = [i for i in open_set.union(closed_set) if (
                    i.f < candidate_node.f) and (i.node == candidate_node.node)]
                # Otherwise, add this candidate to th open set and to the
                # children node of the current node
                if len(criterion) == 0:
                    open_set.add(candidate_node)
                    current_node.children.append(candidate_node)

            # remove the current node from the open set
            open_set.remove(current_node)
            # add the current node into the closed set
            closed_set.add(current_node)
            # if arriving end_point, return closed_set
            if current_node.node == end_point:
                return closed_set
            # find the node in the open set with the minimum f
            next_node = min(open_set, key=lambda i: i.f)
            return A_Star(next_node, end_point, obstacles, open_set, closed_set)
        # If there is some error, it will return the closed set
        except:
            return closed_set
    # If it meet the destination or there is no point in the open set, it
    # return the closed set.
    else:
        return closed_set


# Find the path from the result of A_Star/the closed set.
# Input:
#    end_point: tuple, (x, y), which is the coordinate of destination.
#    closed_set: the set of Node's objects
#    selective_path: the list of tuples which are coordinates
#        ex. if our destination is (3, 3)
#        the output is = [(1, 1), (1, 2), (1, 3), (2,3)]
# we will move from (1, 1) to (3, 3) according to (1, 1) -> (1, 2) -> (1,
# 3) -> (2,3)
def backforward(end_node, closed_set, selective_path):

    # looking for all nodes in the closed_set
    for i in closed_set:
        # if once finding end_point is a child of the node, then this node will
        # be put in the front of selective_path.
        if end_node in i.children:
            selective_path.insert(0, i)

            # look backforward for the child of this node.
            return backforward(i, closed_set, selective_path)
    # if finding the beginning, it will return selective_path
    return selective_path
