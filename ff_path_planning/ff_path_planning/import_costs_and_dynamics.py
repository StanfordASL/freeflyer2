import sys
import os
from pathlib import Path
from importlib import import_module

all_dynamics = [
    x
    for x in os.listdir(Path(__file__).absolute().parent / "dynamics")
    if Path(x).suffix == ".py" and x != "__init__.py"
]
all_costs = [
    x
    for x in os.listdir(Path(__file__).absolute().parent / "costs")
    if Path(x).suffix == ".py" and x != "__init__.py"
]

dynamics_path = Path(__file__).absolute().parent / "dynamics"
costs_path = Path(__file__).absolute().parent / "costs"

for path in [dynamics_path, costs_path]:
    if str(path) not in sys.path:
        sys.path.append(str(path))

DYNAMICS_MODULES = {}
COSTS_MODULES = {}

for dynamics in all_dynamics:
    mod_name = str(Path(dynamics).stem)
    DYNAMICS_MODULES[mod_name] = import_module(mod_name)

for cost in all_costs:
    mod_name = str(Path(cost).stem)
    COSTS_MODULES[mod_name] = import_module(mod_name)