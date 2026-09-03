# Every launch file must import, produce a LaunchDescription, and reference
# only launch arguments it declares. Substitutions evaluate lazily, so an
# undeclared LaunchConfiguration survives generate_launch_description() and
# fails only when that launch is run. Any OpaqueFunction is executed, since a
# launch built around one holds all of its argument reads there.
#
# Needs no camera and no ROS graph.

import importlib.util
import os

import pytest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution

LAUNCH_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "launch")


def _launch_files():
    found = []
    for root, _dirs, files in os.walk(LAUNCH_DIR):
        for name in sorted(files):
            if name.endswith(".launch.py"):
                found.append(os.path.join(root, name))
    return sorted(found)


def _label(path):
    return os.path.relpath(path, LAUNCH_DIR)


def _load(path):
    spec = importlib.util.spec_from_file_location("eys3d_launch_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _reachable(obj, seen=None):
    """Every object reachable from a LaunchDescription through its members.

    Bounded by `seen` rather than a depth limit: a limit silently truncates,
    and several launches nest deeper than any round number.
    """
    if seen is None:
        seen = set()
    if id(obj) in seen:
        return
    seen.add(id(obj))
    yield obj
    if isinstance(obj, (list, tuple, set)):
        for item in obj:
            for found in _reachable(item, seen):
                yield found
    elif isinstance(obj, dict):
        for key, value in obj.items():
            for found in _reachable(key, seen):
                yield found
            for found in _reachable(value, seen):
                yield found
    elif hasattr(obj, "__dict__"):
        for value in vars(obj).values():
            for found in _reachable(value, seen):
                yield found


def _expand_opaque(description):
    """Entities an OpaqueFunction produces, which the description does not hold.

    A launch built around OpaqueFunction keeps every argument read inside the
    callback, so walking only what generate_launch_description() returns checks
    nothing at all for those files.
    """
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    produced = []
    for entity in description.entities:
        if isinstance(entity, OpaqueFunction):
            produced += entity.execute(context) or []
    return produced


def _configuration_name(config):
    parts = config.variable_name
    if isinstance(parts, list):
        return "".join(p.text for p in parts if isinstance(p, TextSubstitution))
    return str(parts)


def test_launch_directory_is_not_empty():
    assert _launch_files(), "no .launch.py found under %s" % LAUNCH_DIR


@pytest.mark.parametrize("path", _launch_files(), ids=_label)
def test_generates_launch_description(path):
    module = _load(path)
    assert hasattr(module, "generate_launch_description"), \
        "%s defines no generate_launch_description()" % _label(path)

    description = module.generate_launch_description()
    assert description is not None, \
        "%s returned None instead of a LaunchDescription" % _label(path)
    assert description.entities, \
        "%s produced an empty LaunchDescription" % _label(path)


@pytest.mark.parametrize("path", _launch_files(), ids=_label)
def test_every_launch_configuration_is_declared(path):
    description = _load(path).generate_launch_description()
    entities = [description] + _expand_opaque(description)

    declared, used = set(), set()
    for obj in _reachable(entities):
        if isinstance(obj, DeclareLaunchArgument):
            declared.add(obj.name)
        elif isinstance(obj, LaunchConfiguration):
            # A configuration carrying its own default cannot fail at
            # runtime; this is what excludes the launch system's own
            # (sigterm_timeout, launch-prefix) without naming them.
            if getattr(obj, "_LaunchConfiguration__default", None) is not None:
                continue
            used.add(_configuration_name(obj))

    undeclared = sorted(name for name in used - declared if name)
    assert not undeclared, \
        "%s reads launch argument(s) it never declares: %s" \
        % (_label(path), ", ".join(undeclared))
