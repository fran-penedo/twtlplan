from twtlplan.util import Box
import numpy as np

n = 2

if n > 2:
    region = Box(np.vstack([np.array([[0, 10], [0, 10]]),
                            np.array([[0, 10] for i in range(n - 2)])]))
else:
    region = Box(np.array([[0, 10], [0, 10]]))

obstacles = [
    Box(np.array([[4, 4.5], [3, 8]])), # CenterLeft
    Box(np.array([[5.5, 6], [3, 8]])), # CenterRight
    Box(np.array([[4, 6], [7.5, 8]])), # Up
    Box(np.array([[6, 8], [4, 4.5]])), # RightBottom
    Box(np.array([[8, 10], [7, 7.5]])) # RightUp
]
if n > 2:
    pad = region.constraints[2:, :]
    for i, o in enumerate(obstacles):
        obstacles[i] = Box(np.vstack([o.constraints, pad]))

A = Box(np.array([[4, 5], [0, 1]]))
B = Box(np.array([[0, 3], [0, 2]]))
C = Box(np.array([[8, 10], [0, 2]]))
D = Box(np.array([[8, 10], [8, 10]]))
props = {'A': A, 'B': B, 'C':C, 'D':D}
if n > 2:
    pad = np.vstack([np.array([0,3]) for i in range(n - 2)]) + \
        np.vstack(range(n - 2)) % 8
    for k, v in props.items():
        props[k] = Box(np.vstack([v.constraints, pad]))

spec = '[H^1 A]^[0, 15] * ([H^1 B]^[0, 15] | [H^1 C]^[0, 15]) * [H^1 D]^[0, 25]'

x_init = np.array([5, 7])
if n > 2:
    x_init = np.hstack([x_init, np.array(range(n - 2)) % 9])
d = 1.0
