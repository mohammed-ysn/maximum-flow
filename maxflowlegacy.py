import csv
from math import inf

class Graph:

    def __init__(self, capacity):
        self.graph = self.create_adjacency_matrix(capacity)
        self.num_of_vertices = len(self.graph)

    def create_adjacency_matrix(self, capacity):
        int_capacity = {(int(v), int(w)): c for (v, w), c in capacity.items()}
        pairs = list(int_capacity.keys())
        limit = pairs[0][0]
        for pair in pairs:
            limit = max(limit, max(pair))
        return [[int_capacity.get((i, j), 0) for j in range(limit + 1)] for i in range(limit + 1)]

    # Using BFS as a searching algorithm
    def bfs(self, s, t, parent):
        visited = [False] * (self.num_of_vertices)
        
        queue = []
        queue.append(s)

        visited[s] = True

        while queue:
            v = queue.pop(0)
            for i, val in enumerate(self.graph[v]):
                if visited[i] == False and val > 0:
                    queue.append(i)
                    visited[i] = True
                    parent[i] = v

        # clean this up at the end
        return True if visited[t] else False

    def ford_fulkerson(self, source, sink):
        parent = [-1] * (self.num_of_vertices)
        max_flow = 0

        while self.bfs(source, sink, parent):
            path_flow = inf
            s = sink
            while(s != source):
                path_flow = min(path_flow, self.graph[parent[s]][s])
                s = parent[s]

            # Adding the path flows
            max_flow += path_flow

            # Updating the residual values of edges
            v = sink
            while(v != source):
                w = parent[v]
                self.graph[w][v] -= path_flow
                self.graph[v][w] += path_flow
                v = parent[v]

        return max_flow, self.graph

'''
Args:
    capacity: a dictionary {(from,to):edge_capacity, ...}
    s, t: the source and sink vertices
Returns a triple  (flow_value, flow, cutset)  where:
    flow_value: the value of the flow, i.e. a number
    flow: a dictionary {(from,to):flow_amount, ...}
    cutset: a list or set of vertices
'''
def compute_max_flow(capacity, source, sink):
    g = Graph(capacity)

    flow_value, flow = g.ford_fulkerson(int(source), int(sink))

    print(flow)

    return (flow_value)


with open('flownetwork_00.csv') as f:
    rows = [row for row in csv.reader(f)][1:]
capacity = {(u, v): int(c) for u,v,c in rows}

print(compute_max_flow(capacity, 0, 3))
