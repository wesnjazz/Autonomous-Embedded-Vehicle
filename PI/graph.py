from heapq import heappush, heappop
import enum

        
class Node():
    def __init__(self, id=None):
        self.id = id
        self.neighbors = {}


    def __repr__(self):
        return str(self.id)


    def __lt__(self, other):
        # this is only implemented so that heappop can run
        # the actual implementation shouldn't affect the answer
        return self.id < other.id


class Edge():
    def __init__(self, turn, distance, speed=10):
        self.distance = distance
        self.turn = turn
        self.speed = speed


    def __repr__(self):
        return "({}, {}, {})".format(self.turn, self.distance, self.speed)
    

class Turn(enum.Enum):
    left = 0
    right = 1
    straight = 2


    def __lt__(self, other):
        # heappop needs this to be defined
        # the implementation doesn't matter
        return False


''''
Returns a list of all the duckietown nodes in order
with correctly arranged edges. Note that the array is
0-indexed while the duckietown representation is
1-indexed so nodes are off by one, i.e. "node 8" is at
position 7 in the list
'''
def get_duckietown():
    nodes = [Node(i+1) for i in range(12)]

    # construct the graph
    nodes[0].neighbors[nodes[11]] = \
        Edge(Turn.straight, 2)
    nodes[0].neighbors[nodes[3]] = \
        Edge(Turn.left, 2)

    nodes[1].neighbors[nodes[3]] = \
        Edge(Turn.right, 2)
    nodes[1].neighbors[nodes[7]] = \
        Edge(Turn.straight, 2)
    
    nodes[2].neighbors[nodes[7]] = \
        Edge(Turn.right, 2)
    nodes[2].neighbors[nodes[11]] = \
        Edge(Turn.left, 2)

    nodes[3].neighbors[nodes[6]] = \
        Edge(Turn.left, 4)
    nodes[3].neighbors[nodes[10]] = \
        Edge(Turn.right, 3)

    nodes[4].neighbors[nodes[2]] = \
        Edge(Turn.left, 2)
    nodes[4].neighbors[nodes[6]] = \
        Edge(Turn.straight, 4)
    
    nodes[5].neighbors[nodes[2]] = \
        Edge(Turn.right, 2)
    nodes[5].neighbors[nodes[10]] = \
        Edge(Turn.straight, 4)

    nodes[6].neighbors[nodes[0]] = \
        Edge(Turn.left, 2)
    nodes[6].neighbors[nodes[9]] = \
        Edge(Turn.straight, 8, speed=15)

    nodes[7].neighbors[nodes[5]] = \
        Edge(Turn.right, 4)
    nodes[7].neighbors[nodes[9]] = \
        Edge(Turn.left, 8, speed=15)

    nodes[8].neighbors[nodes[0]] = \
        Edge(Turn.right, 2)
    nodes[8].neighbors[nodes[5]] = \
        Edge(Turn.straight, 4)
    
    nodes[9].neighbors[nodes[1]] = \
        Edge(Turn.left, 2)
    nodes[9].neighbors[nodes[4]] = \
        Edge(Turn.straight, 4)
    
    nodes[10].neighbors[nodes[1]] = \
        Edge(Turn.right, 2)
    nodes[10].neighbors[nodes[8]] = \
        Edge(Turn.straight, 8, speed=15)
    
    nodes[11].neighbors[nodes[4]] = \
        Edge(Turn.left, 4)
    nodes[11].neighbors[nodes[8]] = \
        Edge(Turn.right, 8, speed=15)

    return nodes


def get_path(start_node, end_node):
    # BFS
    q = [ (0, start_node, [], []) ] # queue of (distance, node, [(turn, speed)], [nodes]) tuples

    while len(q) > 0:
        dist, node, directions, sequence = heappop(q)

        if node == end_node:
            return directions, sequence

        for nnode in node.neighbors:
            edge = node.neighbors[nnode]
            new_dist = dist + edge.distance
            new_directions = directions[:] + [(edge.turn, edge.speed)]
            new_sequence = sequence[:] + [nnode.id]

            q.append( (new_dist, nnode, new_directions, new_sequence) )


'''
starting at node_list[0], this function returns a
list of instructions to hit all the nodes in order
'''
def get_tour(node_list):
    directions, sequence = [], [node_list[0].id]

    for i in range(len(node_list) - 1):
        new_directions, new_sequence = get_path(node_list[i], node_list[i + 1])
        directions += new_directions
        sequence += new_sequence

    return directions, sequence


'''
Convinient, human freindly version of get_tour that
takes a list of integers that are interpreted as the
1-indexed duckietown node sequence
'''
def get_duckietown_tour(int_list):
    graph = get_duckietown()

    sequence = [graph[i - 1] for i in int_list]

    directions, full_sequence = get_tour(sequence)

    return iter(directions), full_sequence


# for testing things
if __name__ == "__main__":
    # graph = get_duckietown()

    # print(get_path(graph[8], graph[9]))

    # sequence = [graph[8], graph[9], graph[11]]
    # print(get_tour(sequence))

    # print(get_duckietown_tour([9, 10, 12]))

    sequence = [2, 6, 3, 1, 10, 7, 6, 4, 9, 1, 3]
    # sequence = [4, 2]

    directions, full_sequence = get_duckietown_tour(sequence)

    print(list(directions))
    print(full_sequence)
