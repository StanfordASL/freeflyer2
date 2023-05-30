import sys
import os
from typing import Optional, Union
from pathlib import Path
from importlib import import_module, reload

DYNAMICS_MODULES = {}
COSTS_MODULES = {}

####################################################################################################

def find_all_dynamics_and_costs(root_path: Optional[Union[Path, str]] = None):
    root_path = Path(root_path) if root_path is not None else Path(__file__).absolute().parent
    print("Looking for modules in the dynamics and costs folders:")
    print(root_path.absolute())
    all_dynamics = [
        x
        for x in os.listdir(root_path / "dynamics")
        if Path(x).suffix == ".py" and x != "__init__.py"
    ]
    all_costs = [
        x for x in os.listdir(root_path / "costs") if Path(x).suffix == ".py" and x != "__init__.py"
    ]
    return all_dynamics, all_costs

def append_to_path(root_path: Optional[Union[Path, str]] = None):
    root_path = Path(root_path) if root_path is not None else Path(__file__).absolute().parent
    print("Looking for modules in the dynamics and costs folders:")
    print(root_path.absolute())
    dynamics_path = Path(root_path) / "dynamics"
    costs_path = Path(root_path) / "costs"
    for path in [dynamics_path, costs_path]:
        if str(path) not in sys.path:
            sys.path.append(str(path))


def load_all_modules(root_path: Optional[Union[Path, str]] = None):
    all_dynamics, all_costs = find_all_dynamics_and_costs(root_path)

    for dynamics in all_dynamics:
        mod_name = str(Path(dynamics).stem)
        try:
            if mod_name in DYNAMICS_MODULES:
                reload(DYNAMICS_MODULES[mod_name])
            else:
                DYNAMICS_MODULES[mod_name] = import_module(mod_name)
        except:
            pass

    for cost in all_costs:
        mod_name = str(Path(cost).stem)
        try:
            if mod_name in COSTS_MODULES:
                reload(COSTS_MODULES[mod_name])
            else:
                COSTS_MODULES[mod_name] = import_module(mod_name)
        except:
            pass

####################################################################################################

append_to_path()
load_all_modules()

####################################################################################################