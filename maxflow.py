import csv
from math import inf


class Vertex:
    def __init__(self, node):
        self.id = node
        # key: vertex object
        # value: path weight
        self.adjacent = {}

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

    def add_neighbour(self, neighbour, weight=0):
        self.adjacent[neighbour] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

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
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, weight=0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbour(self.vert_dict[to], weight)
        if not self.directed:
            self.vert_dict[to].add_neighbour(self.vert_dict[frm], weight)

    def get_vertices(self):
        # return vertex ids
        return self.vert_dict.keys()

    def get_weight_from_ids(self, frm, to):
        return self.get_vertex(frm).get_weight(self.get_vertex(to))

    def bfs_shortest_path(self, s, t):
        s = self.get_vertex(s)
        t = self.get_vertex(t)
        for v in self:
            v.seen = False
            v.come_from = None
        s.seen = True
        toexplore = [s]

        while toexplore:
            v = toexplore.pop(0)
            if v == t:
                # found shortest path to t
                break
            else:
                for w in v.get_connections():
                    if not w.seen:
                        toexplore.append(w)
                        w.seen = True
                        w.come_from = v

        if t.come_from is None:
            # there exists no path from s to t
            return None
        else:
            # construct path working backwards, from t to s
            path = [t]
            while path[0].come_from != s:
                path.insert(0, path[0].come_from)
            path.insert(0, s)
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

    while True:
        # h: residual graph
        # p: shortest path from s to t
        h, p = find_augmenting_path()
        if p is None:
            break
        else:
            ?? = inf
            
            for v, v_next in zip(p, p[1:]):
                v_id = v.get_id()
                v_next_id = v_next.get_id()
                if v.get_weight(v_next) == 1:
                    # edge labelled "inc"
                    ?? = min(??, g.get_weight_from_ids(v_id, v_next_id) -
                            f.get_weight_from_ids(v_id, v_next_id))
                else:
                    # edge labelled "dec"
                    ?? = min(??, f.get_weight_from_ids(v_next_id, v_id))
            
            for v, v_next in zip(p, p[1:]):
                v_id = v.get_id()
                v_next_id = v_next.get_id()
                if v.get_weight(v_next) == 1:
                    # edge labelled "inc"
                    f.add_edge(v_id, v_next_id, f.get_weight_from_ids(
                        v_id, v_next_id) + ??)
                else:
                    # edge labelled "dec"
                    f.add_edge(v_next_id, v_id, f.get_weight_from_ids(
                        v_next_id, v_id) - ??)

    # compute flow_value
    s_in_f = f.get_vertex(s)
    # sum outgoing edges
    s_flow_out = sum([s_in_f.get_weight(v) for v in s_in_f.get_connections()])
    # sum incoming edges
    s_flow_in = sum([v.get_weight(s_in_f)
                    for v in f if s_in_f in v.get_connections()])
    flow_value = s_flow_out - s_flow_in

    # compute cutset by running bfs from s to all nodes
    # after augmenting path not found
    cutset = {s}
    for v in h.get_vertices():
        if h.bfs_shortest_path(s, v):
            cutset.add(v)

    # convert flow graph to dictionary
    flow_dict = {}
    for u in f:
        for v in u.get_connections():
            flow_dict[(u.get_id(), v.get_id())] = u.get_weight(v)

    # voila
    return flow_value, flow_dict, cutset

# test graph
with open('flownetwork_00.csv') as f:
    rows = [row for row in csv.reader(f)][1:]
capacity = {(u, v): int(c) for u, v, c in rows}

print(compute_max_flow(capacity, '0', '2'))
