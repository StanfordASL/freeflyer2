# Freeflyer Path Planning

# Introduction

Please read both the Quickstart and Caveats.

## Quickstart

To run the path planning node
```bash
$ ros2 run ff_path_planning path_planning
```

To run the example client call
```bash
$ ros2 run ff_path_planning examples:service_call
```

## Caveats

* this node depends on [https://github.com/StanfordASL/pmpc](https://github.com/StanfordASL/pmpc) so that package must be correctly installed
* the `pmpc` installation can take up to 20 min!
* the `pmpc` depends on Julia, so:
    * installation might take up 10 - 30 min
    * first 1-2 runs (`ros2 run ...`) might take 1 - 5 min
    * subsequent runs should run much faster
    * *note*: the path planning module supports hot-reloading of dynamics and cost, no need to restart it

---

# Path Planning Service

This node defines the following service
```
string dynamics
string cost "default_cost"
float64[] x0
float64 t0
float64 dt 0.25
int64 horizon 20
int64 xdim -1
int64 udim -1
int64 max_it 20
string params_json
---
float64[] times
float64[] states
float64[] controls
float64[] feedback
```

---

# Defining Custom Dynamics and Costs

The path planning node **automatically discovers** new dynamics and costs and
supports hot-reloading of those when you edit the code. Start the path planning
node once and develop as much as you want!

Both the dynamics and cost function accept an optional `params` argument, a
JSON-serializable object (e.g., a dictionary of values), which can be sent with
every request via a JSON-string `params_json`.

## Dynamics

Place a custom Python file in `./ff_path_planning/dynamics`. The name of the file (minus `.py`) is the string with which you can reference your dynamics.

*Example: `./ff_path_planning/dynamics/example_dynamics.py` becomes `dynamics = example_dynamics`.*

The file
* must define a `f_fx_fu_fn(x, u, params=None)` function, which **batch** evaluates the 
  * `xp`, the next state, `shape = [N, xdim]`
  * `fx`, the state linearization, `shape = [N, xdim, xdim]`
  * `fu`, the control linearization, `shape = [N, xdim, udim]`
* *optionally* can define `xdim` or `XDIM` global variable
* *optionally* can define `udim` or `UDIM` global variable

Argument `params` refers to the optional problem parameters provided every service call is `params_json`.

```python
# ./ff_path_planning/dynamics/example_dynamics.py
XDIM, UDIM = 2, 1

def f_fx_fu_fn(x, u, params=None):
    ...
```

## Cost

Place a custom Python file in `./ff_path_planning/costs`. The name of the file (minus `.py`) is the string with which you can reference your cost.

*Example: `./ff_path_planning/costs/example_cost.py` becomes `cost = example_cost`.*

The file 
* must define a `cost(N=None, xdim=None, udim=None, params=None)` function that defines cost parameters as per [`pmpc`](https://github.com/StanfordASL/pmpc) and returns a dictionary

```python
# ./ff_path_planning/costs/example_costs.py
def cost(N=None, xdim=None, udim=None, params=None):
    ...
    return dict(...)
```

---

# Getting Started

## How to start developing?

Copy and modify `./ff_path_planning/examples/path_planning.py`

Take a look at `//freeflyer2/freeflyer/launch/sim_path_plnner.launch.py` which can be launched via

```bash
$ # launch path planning node separately to gain cleaner output
$ ros2 run ff_path_planning path_planning
$ # then, launch the simulation example
$ ros2 launch freeflyer sim_path_planner.launch.py
```

## Examples

* a full path planning example: `./ff_path_planning/examples/path_planning.py`
    * run via `ros2 run ff_path_planning examples:path_planning`
* calling the path planning service: `./ff_path_planning/examples/service_call.py`
    * run via `ros2 run ff_path_planning examples:service_call`
* setting a static goal for the path-following controller: `./ff_path_planning/examples/setting_goal.py`
    * run via `ros2 run ff_path_planning examples:setting_goal`
