from twtlplan.util import Box
import numpy as np

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
