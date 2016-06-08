from twtlplan.twtlplan import twtlplan
from twtlplan.util import Box
import twtlplan.util as util
import numpy as np
from unittest import skip

def twtlplan_test():
    region = Box(np.array([[0, 10], [0, 10]]))
    obstacles = [
        Box(np.array([[3, 3.5], [4, 8]])), # Left
        Box(np.array([[3, 8], [3, 3.5]])), # Bottom
        Box(np.array([[6, 8], [8, 8.5]])), # Up
        Box(np.array([[7.5, 8], [4, 8.5]])) # Right
    ]
    A = Box(np.array([[1, 2], [1, 2]]))
    B = Box(np.array([[4, 7], [4, 7]]))
    C = Box(np.array([[9, 10], [9, 10]]))
    props = {'A': A, 'B': B, 'C':C}

    spec = '[H^2 A]^[0, 10] * [H^2 B]^[0, 10] * [H^2 C]^[0, 10]'

    x_init = np.array([1, 3])
    d = 1

    end = twtlplan(region, props, obstacles, x_init, spec, d)
    util.plot_casestudy(region, props, obstacles, end.root(), end)

@skip
def twtlplan2_test():
    region = Box(np.array([[0, 10], [0, 10]]))
    obstacles = []
    A = Box(np.array([[1, 3], [1, 3]]))
    B = Box(np.array([[7, 9], [7, 9]]))
    props = {'A': A, 'B': B}

    spec = '[H^2 A]^[0, 10] * [H^2 B]^[0, 10]'

    x_init = np.array([5, 5])
    d = 1

    end = twtlplan(region, props, obstacles, x_init, spec, d)
    util.plot_casestudy(region, props, obstacles, end.root(), end)
