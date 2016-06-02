from twtlplan.twtl_util import *
import nose.tools as nt
import numpy as np
from twtlplan.util import Box

def iscatform_test():
    phi = '[H^2 A]^[1, 5] * [H^3 B]^[0, 3] * [H^1 C]^[0, 2]'
    _, dfa= translate(phi)
    nt.assert_true(iscatform(dfa.tree))
    phi = '[H^2 A]^[1, 5] * [H^3 B]^[0, 3] * (H^1 C)'
    _, dfa= translate(phi)
    nt.assert_false(iscatform(dfa.tree))


def get_cat_operands_test():
    phi = '[H^2 A]^[1, 5] * [H^3 B]^[0, 3] * [H^1 C]^[0, 2]'
    _, dfa= translate(phi)
    phis = get_cat_operands(dfa.tree)
    nt.assert_true(all(p.operation == Op.event for p in phis))

def next_state_test():
    phi = '[H^2 A]^[1, 5] * [H^3 B]^[0, 3] * [H^1 C]^[0, 2]'
    _, dfa= translate(phi)
    init = 4
    sym = 0
    n = 0
    nt.assert_equal(next_state(init, sym, dfa), n)
    init = 0
    sym = 1
    n = 1
    nt.assert_equal(next_state(init, sym, dfa), n)
    # FIXME why?
    # init = 0
    # sym = 0
    # n = 4
    # nt.assert_equal(next_state(init, sym, dfa), n)

def forward_inputsyms_test():
    pass

def subform_states_test():
    phi = '[H^2 A]^[1, 5] * [H^3 B]^[0, 3] * [H^1 C]^[0, 2]'
    _, dfa= translate(phi)
    phis = get_cat_operands(dfa.tree)
    ss = subform_states(phis[1], dfa)
    nt.assert_set_equal(ss, {3, 5, 6, 7})

def alpha_test():
    phi = '[H^2 A]^[1, 5] * [H^3 B]^[0, 3] * [H^1 C]^[0, 2]'
    _, dfa= translate(phi)

    A = Box(np.array([[0, 5], [0, 5]]))
    B = Box(np.array([[0, 5], [2, 15]]))
    C = Box(np.array([[10, 15], [10, 15]]))
    props = {'A': A, 'B': B, 'C':C}
    propmap = dfa.props

    x = np.array([2, 1])
    sym = toalpha(x, props, propmap)
    np.testing.assert_array_equal(fromalpha(sym, props, propmap), [A])
    x = np.array([2, 4])
    sym = toalpha(x, props, propmap)
    nt.assert_set_equal(set(fromalpha(sym, props, propmap)), {A, B})
    x = np.array([-1, -1])
    sym = toalpha(x, props, propmap)
    np.testing.assert_array_equal(fromalpha(sym, props, propmap), [])


