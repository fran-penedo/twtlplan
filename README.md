# TWTLPlan: Language-Guided Path Planning Using Time-Window Temporal Logic (TWTL)

![Case Study 1 scenario](https://franpenedo.com/media/cs1_a.png) 
![Case Study 1 building tree](https://franpenedo.com/media/cs1_b.png)

![Case Study 1 relaxed path found](https://franpenedo.com/media/cs1_c.png) 
![Case Study 1 satisfying path found](https://franpenedo.com/media/cs1_d.png)

## What is TWTLPlan?

TWTLPlan is a tool to solve a 2D path planning problem where the path must satisfy a
specification given as a Time-Window Temporal Logic (TWTL) formula. A TWTL formula
formally captures the notion of visiting regions of interest in order and staying in
them for a period of time. For example, a specification in natural language such as
"perform task A of duration 1 within 2 time units; then, within the time
interval [1, 8] perform tasks B and C of durations 3 and 2, respectively; furthermore, C
must be finished within 4 time units from the start of B;" would correspond to the
following TWTL formula:

    f = [H^1 A]^[0, 2] * [H^3 B ^ [H^2 C]^[0, 4]]^[1, 8]
  
A key feature of TWTL is that a formula may be relaxed, i.e., we can add a variable
delay to each deadline. A relaxed formula with a positive delay is a weaker
specification, while a negative delay indicates a stronger specification. A path
satisfying a stronger specification satisfies a weaker one. TWTLPlan makes use of this
feature by finding a path that satisfies a weaker specification first, then attempts to
reduce the delay until the original specification is satified. These paths are computed
using a sampling-based algorithm called RRT*.

## Requirements

You need Python2.7 with PIP installed, git and wget. The setup script is written in bash 
and has been tested only in Linux. You should also consider using virtualenvs.

## Quickstart

Clone the repository with:

    $ git clone https://github.com/franpenedo/twtlplan.git

Then run the setup script (optionally within a virtualenv):

    $ ./setup.sh

You can run the provided examples with:

    $ python run_benchmark.py draw examples.cs1
    $ python run_benchmark.py draw examples.cs2

## Writing your own scenario

You must create a module with the following objects defined in it:

- `region`: a `twtlplan.util.Box` object defining the domain of the scenario.
- `obstacles`: a list of `Box`es defining the obstacles in the scenario.
- `props`: a dictionary of `str` labels to `Box` representing regions of interest in the
  scenario.
- `spec`: a `str` with the TWTL specification. All regions of interest appearing in this
  formula must also appear as keys in `props`.
- `x_init`: a `numpy.array` of shape `(2,)` with the initial position of the agent.
- `d`: a `float` with the step size.

## Publication

A full description of TWTL, the algorithm implemented in TWTLPlan and an analysis of its
performance can be found in our peer-reviewed publication 
[Language-Guided Sampling-Based Planning using Temporal Relaxation](https://franpenedo.com/publication/wafr16/).

## Copyright and Warranty Information

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Copyright (C) 2016, Francisco Penedo Alvarez (contact@franpenedo.com)

Copyright (C) 2016, Cristian-Ioan Vasile (cvasile@lehigh.edu, cristian.ioan.vasile@gmail.com)
