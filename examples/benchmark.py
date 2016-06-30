from twtlplan.twtlplan import twtlplan
import twtlplan.util as util
import numpy as np
import argparse
from timeit import default_timer as timer
import importlib

def run_cs_draw(m, only_first, draw_tree):
    end = twtlplan(m.region, m.props, m.obstacles, m.x_init, m.spec, m.d,
                   draw_first_path=only_first,
                   draw_its=500 if not only_first else 0)
    util.plot_casestudy(m.region, m.props, m.obstacles,
                        end.root() if draw_tree else None, end,
                        max([n.state for n in end.root().flat()]))


def run_cs_time(m):
    times = []
    its = 20
    for i in range(its):
        print "------ iteration {}".format(i)
        start = timer()
        _ = twtlplan(m.region, m.props, m.obstacles, m.x_init, m.spec, m.d,
                     draw_its=0)
        end = timer()
        times.append(end - start)
        print "- time {}".format(times[-1])

    print "twtlplan times: max {0} min {1} avg {2}".format(
        max(times), min(times), sum(times) / float(its))


def get_argparser():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    subparsers = parser.add_subparsers(dest='action')
    parser_draw = subparsers.add_parser(
        'draw', help='Run an example once and plot evolution')
    parser_draw.add_argument('--only-first', action='store_true', default=False,
                             help='draw the first candidate path')
    parser_draw.add_argument('--draw-tree', action='store_true', default=False,
                             help='draw the tree when plotting the final state')
    parser_time = subparsers.add_parser(
        'time', help='Run an example a number of times a show execution times')
    parser.add_argument('module', help='module containing the case study')
    return parser


if __name__ == "__main__":
    parser = get_argparser()
    args = parser.parse_args()
    module = importlib.import_module(args.module)
    if args.action == 'draw':
        run_cs_draw(module, args.only_first, args.draw_tree)
    elif args.action == 'time':
        run_cs_time(module)
    else:
        parser.print_help()
