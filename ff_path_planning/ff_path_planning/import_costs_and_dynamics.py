import sys
import os
from typing import Optional, Union
from pathlib import Path
from importlib import import_module, reload

DYNAMICS_MODULES, COSTS_MODULES = {}, {}

####################################################################################################


def find_all_dynamics_and_costs(root_path: Optional[Union[Path, str]] = None):
    """Locate all python files in the `dynamics` and `costs` folder given a package root path."""
    root_path = Path(root_path) if root_path is not None else Path(__file__).absolute().parent
    all_dynamics = [
        x
        for x in os.listdir(root_path / "dynamics")
        if Path(x).suffix == ".py" and x != "__init__.py"
    ]
    all_costs = [
        x for x in os.listdir(root_path / "costs") if Path(x).suffix == ".py" and x != "__init__.py"
    ]
    return all_dynamics, all_costs


def find_pkg_source_dir(root_path: Optional[Union[Path, str]] = None):
    """Find where the source of the ff_path_planning package is located. This
    allows hot-reloading dynamics and cost definitions during development."""
    root_path = Path(root_path) if root_path is not None else Path(__file__).absolute().parent
    ros_ws_path = root_path.parents[5]
    msg = f"We're attempting to guess the ROS workspace path is {ros_ws_path}. Is this correct?"

    assert (ros_ws_path / "install").exists() and (ros_ws_path / "src").exists(), msg
    this_pkg_name = root_path.parts[-1]
    pkg_paths = sum(
        [
            [ros_ws_path / "src" / root / dir for dir in dirs if dir == this_pkg_name]
            for (root, dirs, fnames) in os.walk(ros_ws_path / "src")
        ],
        [],
    )
    pkg_path = sorted(pkg_paths, key=lambda x: len(x.parts))[0]  # pick the shortest path
    return pkg_path


def append_to_path(root_path: Optional[Union[Path, str]] = None):
    """Append the installed location of the ff_path_planning package and the
    local development (editable package)."""
    root_path = Path(root_path) if root_path is not None else Path(__file__).absolute().parent

    # append ROS install path first to the end ########################
    dynamics_path = Path(root_path) / "dynamics"
    costs_path = Path(root_path) / "costs"
    for path in [dynamics_path, costs_path]:
        if str(path) not in sys.path:
            sys.path.append(str(path))

    # insert the local development path first #########################
    #pkg_path = find_pkg_source_dir(root_path)
    #this_pkg_name = root_path.parts[-1]
    #msg = f"We're attempting to guess the package path is {pkg_path}. Is this correct?"
    #dynamics_path = pkg_path / this_pkg_name / "dynamics"
    #costs_path = pkg_path / this_pkg_name / "costs"
    #assert dynamics_path.exists() and costs_path.exists(), msg
    #for path in [dynamics_path, costs_path]:
    #    if str(path) not in sys.path:
    #        sys.path.insert(0, str(path))


def load_all_modules(root_path: Optional[Union[Path, str]] = None):
    all_dynamics, all_costs = find_all_dynamics_and_costs(root_path)

    #pkg_source = find_pkg_source_dir(root_path)
    #all_dynamics_dev, all_costs_dev = find_all_dynamics_and_costs(pkg_source / pkg_source.parts[-1])
    #all_dynamics += all_dynamics_dev
    #all_costs += all_costs_dev
    import pdb
    pdb.set_trace()

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

# this should happen every time the module is imported, **not** just when it's called as a script
append_to_path()
load_all_modules()

####################################################################################################
