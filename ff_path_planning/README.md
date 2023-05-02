# Freeflyer Path Planning

## Introduction

Please read both the Quickstart and Caveats.

### Quickstart

To run the path planning node
```bash
$ ros2 run ff_path_planning path_planning
```

To run the example client call
```bash
$ ros2 run ff_path_planning example
```

### Caveats

* this node depends on [`pmpc`](https://github.com/StanfordASL/pmpc) so that package must be correctly installed
* the `pmpc` depends on JIT compilation, so the node startup takes about *1 min*!!!

---

## Path Planning Service

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
---
float64[] times
float64[] states
float64[] controls
float64[] feedback
```

---

## Defining Custom Dynamics and Costs

The path planning node **automatically discovers** new dynamics and costs (but, because of ROS), still requires you to run `colcon build` (or your own build code) after adding custom Python costs and dynamics.

### Dynamics

Place a custom Python file in `./ff_path_planning/dynamics`. The name of the file (minus `.py`) is the string with which you can reference your dynamics.

*Example: `./ff_path_planning/dynamics/example_dynamics.py` becomes `dynamics = example_dynamics`.*

The file
* must define a `f_fx_fu_fn(x, u, p=None)` function, which **batch** evaluates the 
  * `xp`, the next state, `shape = [N, xdim]`
  * `fx`, the state linearization, `shape = [N, xdim, xdim]`
  * `fu`, the control linearization, `shape = [N, xdim, udim]`
* *optionally* can define `xdim` or `XDIM` global variable
* *optionally* can define `udim` or `UDIM` global variable

```python
# ./ff_path_planning/dynamics/example_dynamics.py
XDIM, UDIM = 2, 1

def f_fx_fu_fn(x, u, p=None):
    ...
```

### Cost

Place a custom Python file in `./ff_path_planning/costs`. The name of the file (minus `.py`) is the string with which you can reference your cost.

*Example: `./ff_path_planning/costs/example_cost.py` becomes `cost = example_cost`.*

The file 
* must define a `cost(N=None, xdim=None, udim=None)` function that defines cost parameters as per [`pmpc`](https://github.com/StanfordASL/pmpc) and returns a dictionary

```python
# ./ff_path_planning/costs/example_costs.py
def cost(N=None, xdim=None, udim=None):
    ...
    return dict(...)
```

**You must rerun `colcon build` after adding custom costs or dynamics!!!**

---

## Examples

A simple example is 
* in `./ff_path_planning/example.py` 
* and can be run via `ros2 run ff_path_planning example`
