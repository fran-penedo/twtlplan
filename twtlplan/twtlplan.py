from util import Tree, nearest, col_free, random_sample, steer, near
import util
from twtl_util import get_cat_operands, toalpha, successors, translate, final, \
    next_state, fromalpha, subform_states, forward_inputsyms, interval
import numpy as np
import logging
logger = logging.getLogger('TWTLPLAN')
handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s %(module)s:%(lineno)d:%(funcName)s: %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

logger.setLevel(logging.DEBUG)


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
    its = 0

    while np.sum(taus) > eps:
        its += 1
        if its % 50 == 0:
            util.plot_casestudy(region, props, obstacles, tree, cur)

        sampler = np.random.choice(samplers, p=p)
        # notation: t_i.node = x_i
        t_exp, x_ran = sampler(region, obstacles, tree,
                               phis, taus, dfa, props, propmap)
        x_new = steer(t_exp.node, x_ran, d)
        if col_free(t_exp.node, x_new, region, obstacles):
            # ts_near contains nodes in the same DFA state within steering
            # radius of x_new. t_exp is always in ts_near
            # @CRISTI: check that this makes sense
            ts_near = near([a for a in tree.flat() if a.state == t_exp.state],
                           x_new, d)
            ts_near = [t for t in ts_near
                       if col_free(t.node, x_new, region, obstacles)]
            # Obtain min cost node in ts_near
            t_min = ts_near[np.argmin([t.cost for t in ts_near])]
            t_new = Tree(x_new, t_min.cost + 1,
                         next_state(t_min.state,
                                    toalpha(x_new, props, propmap), dfa))
            t_min.add_child(t_new)

            # Check if t_new is in final state
            candidate = handle_final(t_new, dfa, phis, taus)
            if candidate is None:
                # Rewire (as in RRT*) nodes in ts_near that can be connected to
                # t_new, i.e., nodes in a successor state
                # @CRISTI: this should be conservative but correct
                ts_next = near([a for a in tree.flat()
                                if a.state in successors(t_new.state, dfa)],
                               x_new, d)
                candidate = rewire(ts_next, t_new, region, obstacles,
                                   dfa, phis, taus, props, propmap)
                if candidate is not None:
                    cur = candidate
                    util.plot_casestudy(region, props, obstacles, tree, cur)
            else:
                cur = candidate
                util.plot_casestudy(region, props, obstacles, tree, cur)

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
            # Update cost and states of children and check if they've become
            # better solutions
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
    # Check if t is final and has better cost than current best
    if t.state in final(phis[-1]) and t.cost < np.sum(taus):
        cur = t
        while cur is not None:
            # Update temporal relaxations with the new path
            for i, phi in enumerate(phis):
                if cur.state in final(phi):
                    # @CRISTI: make sure this is correct
                    # FIXME this assumes costs are specific for each formula,
                    # which is not the case. We can't do that since there would
                    # be no way to rewire to final states (since they're also
                    # initial and thus would have cost 0)
                    taus[i] = cur.cost - interval(phi)[1]
            cur = cur.parent
        return t
    else:
        # This is mostly a hack
        for i, phi in enumerate(phis[:-1]):
            if t.state in final(phi) and taus[i] == np.infty:
                taus[i] = t.cost - interval(phi)[1]
        return None

def bias_sample(region, obstacles, t, phis, taus, dfa, props, propmap):
    # Bias towards formula with worst temporal relaxation for current solution
    phi = phis[np.argmax(taus)]
    # Select a random node in a state that corresponds to the formula
    phi_states = subform_states(phi, dfa)
    ts = [x for x in t.flat() if x.state in phi_states]
    if len(ts) == 0:
        # If no nodes correspond to the formula (early), sample uniformly
        return unif_sample(region, obstacles, t, phis, taus, dfa, props, propmap)
    t_exp = np.random.choice(ts)

    # Sample towards a region that appears as symbol to move forwards in the DFA
    input_syms = forward_inputsyms(t_exp.state, dfa)
    input_regions = [fromalpha(s, props, propmap) for s in input_syms]
    bias_regions = np.random.choice(input_regions)
    if len(bias_regions) > 1:
        # FIXME we probably want to sample in the intersection instead
        bias_region = np.random.choice(bias_regions)
    elif len(bias_regions) == 1:
        bias_region = bias_regions[0]
    else:
        # FIXME this shouldn't happen after fixing forward_inputsyms
        bias_region = region

    x_ran = random_sample(bias_region)

    return t_exp, x_ran

def unif_sample(region, obstacles, t, phis, taus, dfa, props, propmap):
    t_exp = np.random.choice(t.flat())
    x_ran = random_sample(region)

    return t_exp, x_ran

