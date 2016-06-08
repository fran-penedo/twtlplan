import numpy as np
from multimethods import multimethod
import cddwrap as cdd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.colors as colors
import matplotlib.cm as cmx
import logging
logger = logging.getLogger('TWTLPLAN')

class Tree(object):
    """A recursive tree structure to store the exploration tree in twtlplan

    Attributes:
        children : list(Tree)
            Children of the node
        node : ndarray
            Workspace point associated with this node
        cost : numeric
            Cost of the node
        state : int
            State of the DFA associated with this node
        parent : Tree
            Parent of the node
    """
    def __init__(self, node=None, cost=np.infty, state=None):
        """Constructs a Tree node

        Parameters:
            node : ndarray, optional
                Workspace point associated with this node
            cost : numeric, optional
                Cost of the node
            state : int, optional
                State of the DFA associated with this node
        """
        self.children = []
        self.node = node
        self.cost = cost
        self.state = state
        self.parent = None

    def add_child(self, x):
        """Adds the Tree x as a child"""
        self.children.append(x)
        x.parent = self

    def add_children(self, xs):
        """Adds the list of Tree xs as children"""
        for x in xs:
            self.add_child(x)

    def rem_child(self, x):
        """Removes the Tree x as a child"""
        self.children.remove(x)
        x.parent = None

    def nodes(self):
        """Returns a list of the nodes of the tree"""
        # return [self.node] + [n for nodes in [c.nodes() for c in self.children]
        #                       for n in nodes]
        return self.traverse(lambda x: x.node)

    def flat(self):
        """Returns the tree in a flat list"""
        # return [self] + [n for nodes in [c.flat() for c in self.children]
        #                       for n in nodes]
        return self.traverse(lambda x: x)

    def traverse(self, f):
        """Returns the tree mapped through f : Tree -> a, as a flat list"""
        return [f(self)] + [n for nodes in [c.traverse(f) for c in self.children]
                            for n in nodes]

    def make_root(self):
        """Makes this node the root of the tree"""
        if self.parent is not None:
            self.parent.make_root()
            self.parent.rem_child(self)
            self.add_child(self.parent)
            self.parent = None

    def path_from_root(self):
        path = [self]
        cur = self.parent
        while cur is not None:
            path.append(cur)
            cur = cur.parent
        path.reverse()
        return path

    def root(self):
        cur = self
        while cur.parent is not None:
            cur = cur.parent
        return cur

    def find(self, x):
        """Returns the Tree with attribute node = x or None if none exists"""
        if all(self.node == x):
            return self
        for c in self.children:
            f = c.find(x)
            if f is not None:
                return f
        return None

    def copy(self):
        """Shallow copy of the tree"""
        t = Tree(self.node, self.cost, self.state)
        t.add_children(c.copy() for c in self.children)
        return t


class Box(object):
    """A rectangular region

    Attributes:
        constraints : ndarray
            nx2 array denoting the minimum and maximum value for each dimension
        n : int
            Number of dimensions of the object
    """
    def __init__(self, constraints):
        """Constructs a Box

        Parameters:
            constraints : ndarray
                nx2 array denoting the minimum and maximum value for each dimension
        """
        self.constraints = constraints
        self.n = constraints.shape[0]

    def contains(self, x):
        """Checks if x is contained in this rectangle.

        Parameters:
            x : ndarray
                An mxn array containing m points with n dimensions
        Returns:
            An mx1 ndarray of boolean (or a single boolean if m = 1)
        """
        if x.shape == (self.constraints.shape[0],):
            return np.all(x >= self.constraints[:,0]) and \
                np.all(x <= self.constraints[:,1])
        elif len(x.shape) > 1 and x.shape[1] == self.constraints.shape[0]:
            return np.logical_and(np.all(x >= self.constraints[:,0], axis=1),
                                  np.all(x <= self.constraints[:,1], axis=1))
        else:
            return False

    def aspoly(self):
        """Returns the Polytope representation of this rectangle"""
        m = np.empty((self.n * 2, self.n + 1))
        m[0::2, 1:] = np.identity(self.n)
        m[1::2, 1:] = -np.identity(self.n)
        m[0::2, 0] = -self.constraints[:, 0]
        m[1::2, 0] = self.constraints[:, 1]
        return Polytope(m)

    def corners(self):
        """Returns the vertices of this rectangle"""
        return np.array(list(it.product(*self.constraints)))

    def center(self):
        """Returns the center of this rectangle"""
        return self.constraints.mean(axis=1)

def random_sample(box):
    """Returns a uniform sample from the given rectangle"""
    return np.array([np.random.uniform(c[0], c[1])
                    for c in box.constraints])

def steer(x_exp, x_ran, d):
    """Returns a point in the trajectory from x_exp to x_ran at distance <= d"""
    v = x_ran - x_exp
    vnorm = np.linalg.norm(v)
    if vnorm > d:
        x_new = x_exp + d * v / vnorm
    else:
        x_new = x_ran

    return x_new

@multimethod(Tree, np.ndarray)
def nearest(t, x):
    """Returns the node in the tree closest to x"""
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

@multimethod(list, np.ndarray)
def nearest(ts, x):
    """Returns the node in a list of tree nodes closest to x"""
    return ts[min(enumerate(ts), key=lambda t: np.linalg.norm(x - t[1].node))[0]]

def mincost_nodes(ts):
    mincost = min([t.cost for t in ts])
    return [t for t in ts if t.cost == mincost]

def near(ts, x, d):
    """Returns the nodes in ts at distance less than d from the point x"""
    return [t for t in ts if np.linalg.norm(x - t.node) <= d + 0.001]

def connect(x, goal, constraints, obstacles):
    """Returns a point from goal that can be connected to x"""
    for y in goal.corners():
        if col_free(y, x, constraints, obstacles):
            return y

def col_free(x_new, x_near, constraints, obstacles):
    """Checks if the path from x_new to x_near is collision free"""
    l = line(x_new, x_near)
    isinside = contains(constraints, x_new)
    return isinside and \
        all(not contains(obs, l) for obs in obstacles)


class Polytope(cdd.CDDMatrix):
    """A polytope. Mostly a convenience class"""

    @staticmethod
    def fromcdd(m):
        """Creates a Polytope from a cdd matrix"""
        x = Polytope([])
        x._m = m._m
        return x

    def contains(self, x):
        """Checks if x intersects the polytope

        Parameters:
            x : Polytope or ndarray
                If x is ndarray, it's interpreted as the set of vertices of a
                polytope.
        """
        if isinstance(x, Polytope):
            return not cdd.pempty(cdd.pinters(self, x))
        elif isinstance(x, np.ndarray):
            return not cdd.pempty(
                cdd.pinters(self, Polytope([np.insert(x, 0, 1)], False)))
        else:
            raise Exception("Not implemented")

    @property
    def n(self):
        """The dimension of the ambient space"""
        return self.col_size - 1


def inters(*args):
    """Computes the intersection of the arguments"""
    x = cdd.pinters(*args)
    return Polytope.fromcdd(x)

def inters_to_union(p, ps):
    """Computes the intersection of a polytope to a union of polytopes"""
    x = cdd.pinters_to_union(p, ps)
    return [Polytope.fromcdd(a) for a in x]

def line(a, b):
    """Returns a Polytope corresponding to the line between two points"""
    return Polytope([np.insert(x, 0, 1) for x in [a, b]], False)

def conv_pts(m):
    """Returns the convex hull of a set of points

    Parameters:
        m : ndarray
            An mxn array. Each row is a vertex
    """
    return Polytope(np.insert(m, 0, 1, axis=1), False)

def conv(pols):
    """Returns the convex hull of a set of Polytopes"""
    return conv_pts(np.vstack([cdd.vrep_pts(p) for p in pols]))


@multimethod(Box, np.ndarray)
def contains(s, x):
    """Checks if x is contained in s"""
    return s.contains(x)

@multimethod(Box, Polytope)
def contains(s, p):
    """Checks if x is contained in s"""
    return s.aspoly().contains(p)


def plot_tree(ax, t):
    ts = t.flat()
    nodes = np.array([[n.node[0], n.node[1], n.state] for n in ts])
    cmap = plt.get_cmap("cool")
    ax.scatter(nodes[:,0], nodes[:,1], c=nodes[:,2], s=50,
               cmap=cmap)
    cnorm = colors.Normalize()
    cnorm.autoscale(nodes[:,2])
    scalarmap = cmx.ScalarMappable(norm=cnorm, cmap=cmap)
    plot_tree_lines(ax, t, scalarmap)

def plot_tree_lines(ax, t, scalarmap):
    for c in t.children:
        ax.plot([t.node[0], c.node[0]], [t.node[1], c.node[1]], '-',
                color=scalarmap.to_rgba(t.state))
        plot_tree_lines(ax, c, scalarmap)
    # label(ax, t.node + [0.1, 0], str(t.cost))

def plot_box(ax, box, **kwargs):
    cs = box.constraints
    x, y = cs[:,0]
    w, h = cs[:,1] - cs[:,0]
    ax.add_patch(patches.Rectangle((x,y), w, h, alpha=.5, **kwargs))

def plot_poly(ax, poly, **kwargs):
    vs = cdd.vrep_pts(poly)
    c = centroid(vs)
    vs = sorted(vs, key=lambda p: math.atan2(p[1]-c[1],p[0]-c[0]))
    ax.add_patch(patches.Polygon(vs, **kwargs))

def plot_path(ax, t_end):
    cur = t_end
    while cur.parent is not None:
        ax.plot([cur.node[0], cur.parent.node[0]],
                [cur.node[1], cur.parent.node[1]], 'm-', lw=2)
        cur = cur.parent


def centroid(vs):
    return np.average(vs, axis=0)

def label(ax, center, text):
    ax.text(center[0], center[1], text, ha="center", va="center")

def plot_casestudy(cons, props, obsts, tree, cur):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plot_box(ax, cons, facecolor="white")
    plot_tree(ax, tree)
    for name, region in props.items():
        plot_box(ax, region, facecolor="green")
        label(ax, region.center(), name)
    for o in obsts:
        plot_box(ax, o, facecolor="red")
    if cur is not None:
        plot_path(ax, cur)
    plt.show()
