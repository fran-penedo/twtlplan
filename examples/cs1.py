from twtlplan.twtlplan import twtlplan
from twtlplan.util import Box
import twtlplan.util as util
import numpy as np
import sys
from timeit import default_timer as timer

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

spec = '[H^2 A]^[3, 10] * [H^3 B]^[0, 15] * [H^2 C]^[0, 15]'

x_init = np.array([1, 3])
d = 0.75


def run_cs1_draw():
    end = twtlplan(region, props, obstacles, x_init, spec, d)
    util.plot_casestudy(region, props, obstacles, end.root(), end,
                        max([n.state for n in end.root().flat()]))


def run_cs1_time():
    times = []
    its = 20
    for i in range(its):
        print "------ iteration {}".format(i)
        start = timer()
        _ = twtlplan(region, props, obstacles, x_init, spec, d, draw_its=0)
        end = timer()
        times.append(end - start)
        print "- time {}".format(times[-1])

    print "twtlplan times: max {0} min {1} avg {2}".format(
        max(times), min(times), sum(times) / float(its))


if __name__ == "__main__":
    helpstr = "Call with --draw or --time"
    if len(sys.argv) != 2:
        print helpstr

    if sys.argv[1] == '--draw':
        run_cs1_draw()
    elif sys.argv[1] == '--time':
        run_cs1_time()
    else:
        print helpstr
