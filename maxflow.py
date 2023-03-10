import csv
from math import inf


class Vertex:
    def __init__(self, node):
        self.id = node

        # key: neighbouring vertex
        # value: edge weight
        self.adjacent = {}

    def __str__(self):
        return f"{self.id} adjacent: {[x.id for x in self.adjacent]}"

    def add_neighbour(self, neighbour, weight=0):
        self.adjacent[neighbour] = weight

    # get all neighbouring vertices
    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    # get weight of edge between vertex and a given neighbour
    def get_weight(self, neighbour):
        return self.adjacent[neighbour]


class Graph:
    def __init__(self, directed=False):
        self.directed = directed
        # key: vertex id
        # value: vertex object
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        # iterate through vertex objects in graph
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices += 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        return self.vert_dict.get(n)

    def add_edge(self, frm, to, weight=0):
        # create vertices if they do not already exist
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        # add connection to both vertices with the given weight
        self.vert_dict[frm].add_neighbour(self.vert_dict[to], weight)
        if not self.directed:
            self.vert_dict[to].add_neighbour(self.vert_dict[frm], weight)

    def get_vertices(self):
        # get a list of all vertex ids in the graph
        return self.vert_dict.keys()

    def get_weight_from_ids(self, frm, to):
        # get the weight of an edge between two vertices
        return self.get_vertex(frm).get_weight(self.get_vertex(to))

    def bfs_shortest_path(self, source_id, target_id):
        source_vertex = self.get_vertex(source_id)
        target_vertex = self.get_vertex(target_id)

        # mark all vertices as unseen and without a come_from vertex
        for v in self:
            v.seen = False
            v.come_from = None

        # start at the source vertex
        source_vertex.seen = True
        to_explore = [source_vertex]

        while to_explore:
            v = to_explore.pop(0)
            if v == target_vertex:
                # found shortest path to target
                break

            # add unseen neighbours to to_explore
            for neighbour in v.get_connections():
                if not neighbour.seen:
                    to_explore.append(neighbour)
                    neighbour.seen = True
                    neighbour.come_from = v

        if target_vertex.come_from is None:
            # there exists no path from source to target
            return None
        else:
            # construct path working backwards, from target to source
            path = [target_vertex]
            while path[0].come_from != source_vertex:
                path.insert(0, path[0].come_from)
            path.insert(0, source_vertex)
            return path


def compute_max_flow(capacity, s, t):
    # capacity graph
    g = Graph(directed=True)
    # flow graph
    f = Graph(directed=True)

    # add all edges in capacity to capacity graph and flow graph
    for (u, v), c in capacity.items():
        # set capacities
        g.add_edge(u, v, c)

        # set all flows initially to 0
        f.add_edge(u, v, 0)

    def find_augmenting_path():
        # define residual graph on same vertices as g
        h = Graph(directed=True)
        for v in g.get_vertices():
            h.add_vertex(v)

        # iterate through each edge u -> v in g
        # to create h with edge weights 1 or -1, depending on flow
        for u in g:
            for v in u.get_connections():
                u_id = u.get_id()
                v_id = v.get_id()
                flow = f.get_weight_from_ids(u_id, v_id)

                if flow < u.get_weight(v):
                    # give h an edge u -> v with weight 1 meaning "inc"
                    h.add_edge(u_id, v_id, 1)
                if flow > 0:
                    # give h an edge v -> u with weight -1 meaning "dec"
                    h.add_edge(v_id, u_id, -1)

        return h, h.bfs_shortest_path(s, t)

    # continuously find augmenting paths until none can be found
    while True:
        # h: residual graph
        # p: shortest path from s to t
        h, p = find_augmenting_path()

        if p is None:
            # no augmenting path can be found
            break
        else:
            # update the flow along the augmenting path
            δ = inf

            for v, v_next in zip(p, p[1:]):
                v_id = v.get_id()
                v_next_id = v_next.get_id()
                if v.get_weight(v_next) == 1:
                    # edge labelled "inc"
                    δ = min(
                        δ,
                        g.get_weight_from_ids(v_id, v_next_id)
                        - f.get_weight_from_ids(v_id, v_next_id),
                    )
                else:
                    # edge labelled "dec"
                    δ = min(δ, f.get_weight_from_ids(v_next_id, v_id))

            # update flow along the augmenting path
            for v, v_next in zip(p, p[1:]):
                v_id = v.get_id()
                v_next_id = v_next.get_id()
                if v.get_weight(v_next) == 1:
                    # edge labelled "inc"
                    f.add_edge(
                        v_id, v_next_id, f.get_weight_from_ids(v_id, v_next_id) + δ
                    )
                else:
                    # edge labelled "dec"
                    f.add_edge(
                        v_next_id, v_id, f.get_weight_from_ids(v_next_id, v_id) - δ
                    )

    # compute flow_value
    source_in_flow = f.get_vertex(s)
    # sum outgoing edges
    source_flow_out = sum(
        source_in_flow.get_weight(v) for v in source_in_flow.get_connections()
    )
    # sum incoming edges
    source_flow_in = sum(
        v.get_weight(source_in_flow) for v in f if source_in_flow in v.get_connections()
    )
    flow_value = source_flow_out - source_flow_in

    # compute cutset by running bfs from s to all nodes
    # after augmenting path not found
    cutset = {s}
    for v in h.get_vertices():
        if h.bfs_shortest_path(s, v):
            cutset.add(v)

    # convert flow graph to dictionary
    flow_dict = {
        (u.get_id(), v.get_id()): u.get_weight(v)
        for u in f
        for v in u.get_connections()
    }

    # voila
    return flow_value, flow_dict, cutset


# test graphs
for fname in ["flownetwork_00.csv", "flownetwork_01.csv", "flownetwork_02.csv"]:
    with open(fname) as f:
        rows = [row for row in csv.reader(f)][1:]
    capacity = {(u, v): int(c) for u, v, c in rows}

    print(f'{fname} results: {compute_max_flow(capacity, "0", "2")}')
