import numpy as np
from multimethods import multimethod
import cddwrap as cdd

class Tree(object):
    def __init__(self, node=None, cost=np.infty, state=None):
        self.children = []
        self.node = node
        self.cost = cost
        self.state = state
        self.parent = None

    def add_child(self, x):
        self.children.append(x)
        x.parent = self

    def add_children(self, xs):
        for x in xs:
            self.add_child(x)

    def rem_child(self, x):
        self.children.remove(x)
        x.parent = None

    def nodes(self):
        return [self.node] + [n for nodes in [c.nodes() for c in self.children]
                              for n in nodes]

    def flat(self):
        return [self] + [n for nodes in [c.flat() for c in self.children]
                              for n in nodes]

    def make_root(self):
        if self.parent is not None:
            self.parent.make_root()
            self.parent.rem_child(self)
            self.add_child(self.parent)
            self.parent = None

    def find(self, x):
        if all(self.node == x):
            return self
        for c in self.children:
            f = c.find(x)
            if f is not None:
                return f
        return None

    def copy(self):
        t = Tree(self.node)
        t.add_children(c.copy() for c in self.children)
        return t


class Box(object):
    def __init__(self, constraints):
        self.constraints = constraints
        self.n = constraints.shape[0]

    def contains(self, x):
        if x.shape == (self.constraints.shape[0],):
            return np.all(x >= self.constraints[:,0]) and \
                np.all(x <= self.constraints[:,1])
        elif len(x.shape) > 1 and x.shape[1] == self.constraints.shape[0]:
            return np.logical_and(np.all(x >= self.constraints[:,0], axis=1),
                                  np.all(x <= self.constraints[:,1], axis=1))
        else:
            return False

    def aspoly(self):
        m = np.empty((self.n * 2, self.n + 1))
        m[0::2, 1:] = np.identity(self.n)
        m[1::2, 1:] = -np.identity(self.n)
        m[0::2, 0] = -self.constraints[:, 0]
        m[1::2, 0] = self.constraints[:, 1]
        return Polytope(m)

    def corners(self):
        return np.array(list(it.product(*self.constraints)))


def random_sample(box):
    return np.array([np.random.uniform(c[0], c[1])
                    for c in box.constraints])

def steer(x_exp, x_ran, d):
    v = x_ran - x_exp
    vnorm = np.linalg.norm(v)
    if vnorm > d:
        x_new = x_exp + d * v / vnorm
    else:
        x_new = x_ran

    return x_new

def nearest(t, x):
    if len(t.children) > 0:
        nearc = [nearest(c, x) for c in t.children]
        dists = [np.linalg.norm(x - nc.node) for nc in nearc]
        minc = np.argmin(dists)
        if dists[minc] < np.linalg.norm(x - t.node):
            return nearc[minc]
        else:
            return t
    else:
        return t

def near(ts, x, d):
    return [t for t in ts if np.linalg.norm(x - t.node) <= d + 0.001]

def connect(x, goal, constraints, obstacles):
    for y in goal.corners():
        if col_free(y, x, constraints, obstacles):
            return y

def col_free(x_new, x_near, constraints, obstacles):
    l = line(x_new, x_near)
    isinside = contains(constraints, x_new)
    return isinside and \
        all(not contains(obs, l) for obs in obstacles)


class Polytope(cdd.CDDMatrix):

    @staticmethod
    def fromcdd(m):
        x = Polytope([])
        x._m = m._m
        return x

    def contains(self, x):
        if isinstance(x, Polytope):
            return not cdd.pempty(cdd.pinters(self, x))
        elif isinstance(x, np.ndarray):
            return not cdd.pempty(
                cdd.pinters(self, Polytope([np.insert(x, 0, 1)], False)))
        else:
            raise Exception("Not implemented")

    @property
    def n(self):
        return self.col_size - 1


def inters(*args):
    x = cdd.pinters(*args)
    return Polytope.fromcdd(x)

def inters_to_union(p, ps):
    x = cdd.pinters_to_union(p, ps)
    return [Polytope.fromcdd(a) for a in x]

def line(a, b):
    return Polytope([np.insert(x, 0, 1) for x in [a, b]], False)

def conv_pts(m):
    return Polytope(np.insert(m, 0, 1, axis=1), False)

def conv(pols):
    return conv_pts(np.vstack([cdd.vrep_pts(p) for p in pols]))


@multimethod(Box, np.ndarray)
def contains(s, x):
    return s.contains(x)

@multimethod(Box, Polytope)
def contains(s, p):
    return s.aspoly().contains(p)
