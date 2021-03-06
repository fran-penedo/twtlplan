from util import Tree, nearest, col_free, random_sample, steer, near, \
    mincost_nodes, PlotOpts
import util
from twtl_util import get_cat_operands, toalpha, successors, translate, final, \
    next_state, fromalpha, subform_states, forward_inputsyms, interval, nstates
import numpy as np
import logging
logger = logging.getLogger('TWTLPLAN')
handler = logging.StreamHandler()
formatter = logging.Formatter('%(levelname)s %(module)s:%(lineno)d:%(funcName)s: %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)
logger.propagate = False

logger.setLevel(logging.DEBUG)


def twtlplan(region, props, obstacles, x_init, spec, d, eps=0,
             samplers=None, p=None, **kwargs):
    if samplers is None:
        samplers = [bias_sample, unif_sample]
    if p is None:
        p = [0.5, 0.5]

    _, dfa = translate(spec)
    propmap = dfa.props
    tree = Tree(x_init, 0, 0, dfa.init.keys()[0], toalpha(x_init, props, propmap))
    tree_flat = [tree]
    nodes_by_state = {tree.state : [tree]}
    # cur holds the last node in the current candidate path
    cur = None
    # phis contain the ASTs corresponding to each concatenation operand
    phis = get_cat_operands(dfa.tree)
    phis_states = [subform_states(phi, dfa) for phi in phis]

    # holds the temp relaxations of each cat operand in cur path
    taus = [np.infty for phi in phis]

    its = 0
    drawed = False
    po = PlotOpts(kwargs)

    while np.sum(taus) > eps:
        if po.draw_its > 0 and its % po.draw_its == 0:
            util.plot_casestudy(region, props, obstacles, tree, cur,
                                nstates(dfa), po.plot_file_prefix)
        if po.draw_first_path and not drawed and cur is not None:
            util.plot_casestudy(region, props, obstacles, None, cur,
                                nstates(dfa), po.plot_file_prefix)
            drawed = True
        if its % 1000 == 0:
            logger.info("Its = {}".format(its))

        sampler = np.random.choice(samplers, p=p)
        # notation: t_i.node = x_i
        t_exp, x_ran = sampler(region, obstacles, nodes_by_state,
                               phis, taus, dfa, props, propmap, phis_states)
        x_new = steer(t_exp.node, x_ran, d)
        if col_free(t_exp.node, x_new, region, obstacles):
            # ts_near contains nodes in the same DFA state within steering
            # radius of x_new. t_exp is always in ts_near
            ts_near = near([a for a in tree_flat if a.state == t_exp.state],
                           x_new, d)
            ts_near = [t for t in ts_near
                       if col_free(t.node, x_new, region, obstacles)]
            # Obtain min cost node in ts_near
            # FIXME I'm not sure if this is making any difference
            t_min = nearest(mincost_nodes(ts_near), x_new)
            # t_min = ts_near[np.argmin([t.cost for t in ts_near])]

            sym_new = toalpha(x_new, props, propmap)
            s_new = next_state(t_min.state, sym_new, dfa)
            c_new, acc_new = next_cost(t_min.cost, t_min.state, sym_new,
                          s_new, phis, dfa, phis_states, t_min.acc)
            t_new = Tree(x_new, c_new, acc_new, s_new, sym_new)
            t_min.add_child(t_new)
            nodes_by_state.setdefault(t_new.state, []).append(t_new)
            tree_flat.append(t_new)

            # Check if t_new is in final state
            candidate = handle_final(t_new, dfa, phis, taus)
            if candidate is None:
                # Rewire (as in RRT*) nodes in ts_near that can be connected to
                # t_new, i.e., nodes in a successor state
                ts_next = near([a for a in tree_flat
                                if a.state in successors(t_new.state, dfa) and
                                a not in t_new.path_from_root()],
                               x_new, d)
                candidate = rewire(ts_next, t_new, region, obstacles,
                                   dfa, phis, taus, props, propmap, phis_states)

            cur = update_cur(cur, candidate)
        its += 1

    logger.debug(taus)
    return cur

# Rewires the ts_next nodes through t_new if it has less cost. Propagates any
# cost to children. Returns the best new candidate path if any
# have been discovered or None otherwise
def rewire(ts_next, t_new, region, obstacles, dfa, phis, taus, props, propmap, phis_states):
    cur = None
    for t_next in ts_next:
        s_next = next_state(t_new.state, t_next.sym, dfa)
        c_next, _ = next_cost(t_new.cost, t_new.state, t_new.sym,
                              s_next, phis, dfa, phis_states, t_new.acc)
        if t_next.cost > c_next and \
                col_free(t_new.node, t_next.node, region, obstacles):
            t_next.parent.rem_child(t_next)
            t_new.add_child(t_next)
            t_next.cost = c_next
            t_next.state = s_next
            # Update cost and states of children and check if they've become
            # better solutions
            candidate = update_info(t_next, dfa, phis, taus, phis_states)
            cur = update_cur(cur, candidate)

    return cur


# Returns candidate if it is not None, else returns cur
def update_cur(cur, candidate):
    if candidate is not None:
        return candidate
    else:
        return cur

# Updates cost and states of t's children. Returns the best new candidate path
# if any is discovered or None otherwise
def update_info(t, dfa, phis, taus, phis_states):
    cur = handle_final(t, dfa, phis, taus)
    for c in t.children:
        c.state = next_state(t.state, c.sym, dfa)
        c_new, acc_new = next_cost(t.cost, t.state, c.sym, c.state,
                                   phis, dfa, phis_states, t.acc)
        c.cost = c_new
        c.acc = acc_new
        candidate = update_info(c, dfa, phis, taus, phis_states)
        if candidate is not None:
            cur = candidate

    return cur

def path_taus(path, phis):
    taus = [np.infty for phi in phis]
    last_tau = 0
    i = 0
    for cur in path:
        # Update temporal relaxations with the new path
        if cur.state in final(phis[i]):
            # logger.debug("cost, last_tau: {}, {}".format(cur.cost, last_tau))
            taus[i] = max(cur.cost - last_tau, 0)
            last_tau = taus[i]
            i += 1
    if sum(taus) < np.infty:
        logger.debug("Computed taus: {}".format(taus))
    return taus


# Returns t if it corresponds to a final state and is a better candidate path
# than the current one and updates the taus vector. Otherwise, returns None
def handle_final(t, dfa, phis, taus):
    # Check if t is final and has better cost than current best
    if t.state in final(phis[-1]):
        taus_new = path_taus(t.path_from_root(), phis)
        if np.sum(taus_new) < np.sum(taus):
            for i in range(len(taus)):
                taus[i] = taus_new[i]
            return t
    else:
        if any([tau == np.infty for tau in taus]):
            taus_new = path_taus(t.path_from_root(), phis)
            if len([t for tau in taus if tau != np.infty]) < \
                    len([tau for tau in taus_new if tau != np.infty]):
                logger.info("taus = {}, {}".format(taus_new, taus))
                for i in range(len(taus)):
                    taus[i] = taus_new[i]
        return None

def bias_sample(region, obstacles, nodes_by_state, phis, taus, dfa,
                props, propmap, phis_states):
    # Bias towards formula with worst temporal relaxation for current solution
    i = np.argmax(taus)
    # Select a random node in a state that corresponds to the formula
    st_ran = np.random.choice(list(phis_states[i]))

    if len(nodes_by_state.get(st_ran, [])) == 0:
        # If no nodes correspond to the formula (early), sample uniformly
        return unif_sample(region, obstacles, nodes_by_state, phis, taus, dfa,
                           props, propmap, phis_states)

    # Sample towards a region that appears as symbol to move forwards in the DFA
    input_syms = forward_inputsyms(st_ran, dfa)
    input_regions = [fromalpha(s, props, propmap) for s in input_syms]
    bias_regions = input_regions[np.random.choice(len(input_regions))]
    if len(bias_regions) > 1:
        bias_region = np.random.choice(bias_regions)
    elif len(bias_regions) == 1:
        bias_region = bias_regions[0]
    else:
        # FIXME this shouldn't happen after fixing forward_inputsyms
        bias_region = region

    # x_ran = random_sample(bias_region)
    x_ran = random_sample(region)
    # t_exp = np.random.choice(nodes_by_state[st_ran])
    t_exp = nearest(nodes_by_state[st_ran], x_ran)

    return t_exp, x_ran

def unif_sample(region, obstacles, nodes_by_state, phis, taus, dfa,
                props, propmap, phis_states):
    st_ran = np.random.choice([st for st in nodes_by_state.keys()
                               if st not in dfa.final])
    x_ran = random_sample(region)
    t_exp = nearest(nodes_by_state[st_ran], x_ran)

    return t_exp, x_ran

def next_cost(cur, s_cur, symbol, s_next, phis, dfa, phis_states, acc):
    for i, phi in enumerate(phis):
        if s_cur in phis_states[i] \
                and s_cur not in final(phi) \
                and s_next in final(phi):
            nacc = acc + max(cur - acc + 1 - interval(phi, s_cur, symbol)[1], 0)
            return nacc, nacc
    return cur + 1, acc

