import sys
import os
sys.path.insert(0, os.getcwd() + '/lib/twtl')
sys.path.insert(0, os.getcwd() + '/lib')
import numpy as np
from timeit import default_timer as timer
import networkx as nx
import StringIO
import twtl.twtl as twtl
from twtl.dfa import DFAType
from twtl.synthesis import expand_duration_ts, compute_control_policy, ts_times_fsa,\
                      verify
from lomap import Ts

n = 20

FREE = 0
OBST = 1
A = 2
B = 3
C = 4

m = {0: 'F', 1: 'O', 2: 'A', 3: 'B', 4: 'C'}

grid = np.zeros((n, n)) + FREE

grid[np.ix_([2,3], [2,3])] = A
grid[np.ix_(list(range(8, 14)), list(range(8, 14)))] = B
grid[np.ix_([18,19], [18,19])] = C

grid[np.ix_(list(range(6, 16)), [6])] = OBST
grid[np.ix_(list(range(12, 16)), [16])] = OBST
grid[np.ix_([6], list(range(8, 16)))] = OBST
grid[np.ix_([15], list(range(8, 17)))] = OBST


def l(i):
    return "s_{}".format("_".join([str(x) for x in i]))

def toTS(g, init, out):
    print >>out, "name Foo DTS"
    print >>out, "init {{ '{}':1 }}".format(l(init))
    print >>out, ";"

    for i in range(g.shape[0]):
        for j in range(g.shape[1]):
            print >>out, "{} {{'prop': {{'{}'}}, 'position': ({}, {})}}".format(
                l([i,j]), m[g[i,j]], i, j)
    print >>out, ";"

    for i in range(g.shape[0]):
        for j in range(g.shape[1]):
            print_edge([i,j], [i,j], g[i,j], out)
            if i - 1 >= 0:
                print_edge([i,j], [i-1,j], g[i-1,j], out)
            if j - 1 >= 0:
                print_edge([i,j], [i,j-1], g[i,j-1], out)
            if i + 1 < g.shape[0]:
                print_edge([i,j], [i+1,j], g[i+1,j], out)
            if j + 1 < g.shape[1]:
                print_edge([i,j], [i,j+1], g[i,j+1], out)

def print_edge(a, b, btype, out):
    if btype != OBST:
        print >>out, "{} {} {{'duration': 1}}".format(l(a), l(b))

out = StringIO.StringIO()
toTS(grid, [2,6], out)
ts_file = "./foo.txt"
f = open(ts_file, 'w')
f.write(out.getvalue())
f.close()

phi = "[H^2 A]^[3, 10] * [H^3 B]^[0,15] * [H^2 C]^[0,15]"

its = 20
times = []

for i in range(its):
    print "---- it {}".format(i)
    start = timer()
    _, dfa_0, dfa_inf, bdd = twtl.translate(phi, kind='both', norm=True)

    ts = Ts(directed=True, multi=False)
    ts.read_from_file(ts_file)

    ets = expand_duration_ts(ts)
    dfa = dfa_inf

    pa = ts_times_fsa(ets, dfa)
    policy, output, tau = compute_control_policy(pa, dfa, dfa.kind)
    print('Max deadline: %s', tau)
    if policy is not None:
        print('Generated output word is: %s', [tuple(o) for o in output])

        policy = [x for x in policy if x not in ets.state_map]
        out = StringIO.StringIO()
        for u, v in zip(policy[:-1], policy[1:]):
            print>>out, u, '->', ts.g[u][v][0]['duration'], '->',
        print>>out, policy[-1],
        print('Generated control policy is: %s', out.getvalue())
        out.close()

        print('Relaxation is: %s',
                        twtl.temporal_relaxation(output, formula=phi))
    else:
        print('No control policy found!')

    end = timer()
    times.append(end - start)
    print "- time {}".format(times[-1])

print "times: max {} min {} avg {}".format(max(times), min(times), sum(times)/float(its))
