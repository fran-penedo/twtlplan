from util import Tree, nearest, col_free, random_sample, steer, near
from twtl_util import get_cat_operands, toalpha, successors, translate, final, \
    next_state, fromalpha, subform_states, forward_inputsyms, interval
import numpy as np
import logging
logger = logging.getLogger('TWTLPLAN')


def twtlplan(region, props, obstacles, x_init, spec, d, eps=0,
             samplers=None, p=None):
    if samplers is None:
        samplers = [bias_sample, unif_sample]
    if p is None:
        p = [0.5, 0.5]

    _, dfa = translate(spec)
    propmap = dfa.props
    tree = Tree(x_init, 0, dfa.init[0])
    cur = None
    phis = get_cat_operands(dfa.tree)
    taus = [np.infty for phi in phis]

    while np.sum(taus) > eps:
        sampler = np.random.choice(samplers, p=p)
        t_exp, x_ran = sampler(region, obstacles, tree,
                               phis, taus, dfa, props, propmap)
        x_new = steer(t_exp.node, x_ran, d)
        if col_free(t_exp.node, x_new, region, obstacles):
            ts_near = near([a for a in tree.flat() if a.state == t_exp.state],
                           x_new, d)
            ts_near = [t for t in ts_near
                       if col_free(t.node, x_new, region, obstacles)]
            t_min = ts_near[np.argmin([t.cost for t in ts_near])]
            t_new = Tree(x_new, t_min.cost + 1,
                         next_state(t_min.state,
                                    toalpha(x_new, props, propmap), dfa))
            t_min.add_child(t_new)

            candidate = handle_final(t_new, dfa, phis, taus)
            if candidate is None:
                ts_next = near([a for a in tree.flat()
                                if a.state in successors(t_new.state, dfa)],
                               x_new, d)
                candidate = rewire(ts_next, t_new, region, obstacles,
                                   dfa, phis, taus, props, propmap)
                if candidate is not None:
                    cur = candidate
            else:
                cur = candidate

    return cur

def rewire(ts_next, t_new, region, obstacles, dfa, phis, taus, props, propmap):
    cur = None
    for t_next in ts_next:
        if t_next.cost > t_new.cost + 1 and \
                col_free(t_new.node, t_next.node, region, obstacles):
            t_next.parent.rem_child(t_next)
            t_new.add_child(t_next)
            t_next.cost = t_new.cost + 1
            t_next.state = next_state(t_new.state,
                                      toalpha(t_next.node, props, propmap), dfa)
            candidate = update_info(t_next, dfa, phis, taus, props, propmap)
            if candidate is not None:
                cur = candidate

    return cur


def update_info(t, dfa, phis, taus, props, propmap):
    cur = handle_final(t, dfa, phis, taus)
    for c in t.children:
        c.cost = t.cost + 1
        c.state = next_state(t.state, toalpha(c.node, props, propmap), dfa)
        candidate = update_info(c, dfa, phis, taus, props, propmap)
        if candidate is not None:
            cur = candidate

    return cur

def handle_final(t, dfa, phis, taus):
    if t.state in final(phis[-1]) and t.cost < np.sum(taus):
        cur = t
        while cur is not None:
            for i, phi in enumerate(phis):
                if cur.state in final(phi):
                    taus[i] = cur.cost - interval(phi)[1]
        return t
    else:
        return None

def bias_sample(region, obstacles, t, phis, taus, dfa, props, propmap):
    phi = phis[np.argmax(taus)]
    phi_states = subform_states(phi, dfa)
    ts = [x for x in t.flat() if x.state in phi_states]
    if len(ts) == 0:
        return unif_sample(region, obstacles, t, phis, taus, dfa, props, propmap)
    t_exp = np.random.choice(ts)

    input_syms = forward_inputsyms(t_exp.state, dfa)
    input_regions = [fromalpha(s) for s in input_syms]
    bias_regions = np.random.choice(input_regions)
    if len(prop) > 1:
        bias_region = np.random.choice(bias_regions)
    else:
        bias_region = bias_regions[0]

    x_ran = random_sample(bias_region)

    return t_exp, x_ran

def unif_sample(region, obstacles, t, phis, taus, dfa, props, propmap):
    x_ran = random_sample(region)
    t_exp = nearest(t, x_ran)

    return t_exp, x_ran

