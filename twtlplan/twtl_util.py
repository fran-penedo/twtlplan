import sys
sys.path.insert(0, 'lib/twtl')

import twtl.twtl as twtl
from twtl.dfa import Op, DFAType
import util

def translate(phi):
    return twtl.translate(phi, kind=DFAType.Infinity)

def iscatform(ast):
    if ast.operation != Op.cat:
        return False
    if ast.left.operation == Op.cat:
        isleft = iscatform(ast.left)
    else:
        isleft = ast.left.operation == Op.event

    return ast.right.operation == Op.event and isleft

def get_cat_operands(ast):
    if not iscatform(ast):
        raise Exception("AST does not correspond to cat form")

    if ast.left.operation == Op.cat:
        left = get_cat_operands(ast.left)
    else:
        left = [ast.left]

    return [ast.right] + left

def final(ast):
    return ast.final

def initial(ast):
    return ast.init

def interval(ast):
    if ast.operation == Op.event:
        return ast.low, ast.high
    else:
        raise Exception("Cannot compute interval of {} formula".format(
            Op.str(ast.operation)))

def successors(state, dfa):
    return dfa.g[state].keys()

def next_state(state, sym, dfa):
    for n, dic in dfa.g[state].items():
        if sym in dic['input']:
            return n
    raise Exception("Cannot jump from {state} with symbol {sym}".format(
        state=state, sym=sym))

def forward_inputsyms(state, dfa):
    s = set()
    for n, d in dfa.g[state].items():
        # if d['label'] != '(else)': # FIXME more complicated than this, not
        # sure how to do this
        symbits = reduce(operator.and_, d['input'])
        s.add(symbits)

    return s

def subform_states(f, dfa):
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
    return sum((1 if util.contains(region, x) else 0) * propmap[k]
               for k, region in props.items())

def fromalpha(x, props, propmap):
    m = 1
    s = []
    for i in range(x.bit_length()):
        if x & m > 0:
            for k, v in propmap.items():
                if m == v:
                    s.append(props[k])
        m <<= 1

    return s
