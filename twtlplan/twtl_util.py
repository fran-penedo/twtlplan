import sys
sys.path.insert(0, 'lib/twtl')

import twtl.twtl as twtl
from twtl.dfa import Op, DFAType
import operator
import util

def translate(phi):
    """Wrapper for twtl.translate. Creates a DFA from a TWTL formula.

    Parameters:
        phi : string
            TWTL formula to translate

    Returns:
        (alphabet, dfa) : (set, Fsa)
            A tuple with the alphabet and the dfa
    """
    return twtl.translate(phi, kind=DFAType.Infinity)

def iscatform(ast):
    """Checks if a formula with the given AST is in concatenation form"""
    if ast.operation != Op.cat:
        return False
    if ast.left.operation == Op.cat:
        isleft = iscatform(ast.left)
    else:
        isleft = ast.left.operation == Op.event

    return ast.right.operation == Op.event and isleft

def get_cat_operands(ast):
    """Returns the nodes of the AST for each concatenation operand"""
    if not iscatform(ast):
        raise Exception("AST does not correspond to cat form")

    if ast.left.operation == Op.cat:
        left = get_cat_operands(ast.left)
    else:
        left = [ast.left]

    return left + [ast.right]

def final(ast):
    """Returns the final state for the annotated ast"""
    return ast.final

def initial(ast):
    """Returns the initial state for the annotated ast"""
    return ast.init

def interval(ast):
    """Returns a pair with the within interval for ast"""
    if ast.operation == Op.event:
        return ast.low, ast.high
    else:
        raise Exception("Cannot compute interval of {} formula".format(
            Op.str(ast.operation)))

def successors(state, dfa):
    """Returns the successors of a given state in the dfa"""
    return dfa.g[state].keys()

def next_state(state, sym, dfa):
    """Computes the next state in the dfa from the given state and symbol"""
    for n, dic in dfa.g[state].items():
        if sym in dic['input']:
            return n
    raise Exception("Cannot jump from {state} with symbol {sym}".format(
        state=state, sym=sym))

def forward_inputsyms(state, dfa):
    """Computes the set of symbols that jump forward from a state in the dfa"""
    s = set()
    for n, d in dfa.g[state].items():
        # if d['label'] != '(else)': # FIXME more complicated than this, not
        # sure how to do this
        # @CRISTI: HALP!
        symbits = reduce(operator.and_, d['input'])
        s.add(symbits)

    return s

def subform_states(f, dfa):
    """Computes the set of states corresponding to a subformula in the dfa"""
    final_states = final(f)
    v = set()
    s = set(initial(f))
    while len(s) > 0:
        n = s.pop()
        if n not in final_states:
            v.add(n)
            succ = set(successors(n, dfa))
            succ.difference_update(v)
            s.update(succ)
    return v

def toalpha(x, props, propmap):
    """Converts from a point in the workspace to an alphabet symbol

    Parameters:
        x : ndarray
            A point in the workspace
        props : dict (string -> region)
            A map from labels to regions
        propmap : dict (string -> int)
            A map from labels to the alphabet symbol corresponding to the label

    Returns:
        The alphabet symbol corresponding to the set of regions containing x
    """
    return sum((1 if util.contains(region, x) else 0) * propmap[k]
               for k, region in props.items())

def fromalpha(x, props, propmap):
    """Converts from an alphabet symbol to the list of corresponding regions

    Parameters:
        x : int
            An alphabet symbol
        props : dict (string -> region)
            A map from labels to regions
        propmap : dict (string -> int)
            A map from labels to the alphabet symbol corresponding to the label

    Returns:
        The list of regions corresponding to the given symbol
    """
    m = 1
    s = []
    for i in range(x.bit_length()):
        if x & m > 0:
            for k, v in propmap.items():
                if m == v:
                    s.append(props[k])
        m <<= 1

    return s
