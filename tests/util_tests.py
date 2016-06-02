from twtlplan.util import *
import numpy as np
import nose.tools as nt

def nodes_test():
    t = Tree(1)
    t.add_child(Tree(2))
    t.add_child(Tree(3))
    t.children[0].add_child(Tree(0))
    nt.assert_set_equal(set(t.nodes()), set(range(4)))

def box_test():
    cons = np.array([[-1, 1], [2, 3]])
    b = Box(cons)
    nt.assert_true(b.contains(np.array([0, 2.5])))
    nt.assert_false(b.contains(np.array([0, 0])))
    nt.assert_false(b.contains(np.array([0, 2.5, 0])))

    x = np.array([[0, 2.5], [0, 0]])
    np.testing.assert_array_equal(b.contains(x), np.array([True, False]))

    x = np.array([[1,1], [2,2]])
    b = Box(x)
    nt.assert_true(b.contains(np.array([1,2])))

def polytope_test():
    p = Polytope([[1, 0, 0], [1, 1, 0], [1, 0, 1]], False)
    p1 = Polytope([[1, 0.5, 0.5], [1, 1.5, 1.5]], False)
    p2 = Polytope([[1, 1.2, 1.2], [1, 1.5, 1.5]], False)

    nt.assert_true(p.contains(p1))
    nt.assert_false(p.contains(p2))

def line_test():
    p = Polytope([[1, 0, 0], [1, 1, 0], [1, 0, 1]], False)
    p1 = line(np.array([0.5, 0.5]), np.array([1.5, 1.5]))
    p2 = line(np.array([1.2, 1.2]), np.array([1.5, 1.5]))

    nt.assert_true(p.contains(p1))
    nt.assert_false(p.contains(p2))

