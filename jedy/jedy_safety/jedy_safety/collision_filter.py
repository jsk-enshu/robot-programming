"""Self-collision filter for Jedy, plugged into actuo's real bus servicer.

    ros2 run actuo_ros actuo_bus_node --real \\
        --config .../jedy_four_dof_kondo.yaml --shm-name jedy_four_dof \\
        --command-filter jedy_safety.collision_filter:build

This is the part that knows Jedy has links and where they are, which is exactly
why it is here and not in actuo: link geometry stays out of the driver layer,
so actuo offers a seam (``actuo.core.contracts.CommandFilter``) and the robot's
own package fills it, with actuo.safety's gate and monitor.

WHAT THIS CAN AND CANNOT PROMISE, on the robot as wired on 2026-08-01
---------------------------------------------------------------------
Measured over the right arm's whole joint range: **no pose of the driven joints
alone self-collides.** Every collision the arms can reach involves a hand link,
and the hand's servo (id 14/15) is not on the bus -- nothing reports it, so the
model places it at 0. The filter is therefore judging a link whose real pose is
a guess. It is honest plumbing and it is not yet a safety guarantee; that
changes when the hand is either driven or removed from the URDF.
"""

from __future__ import annotations

import os
from typing import List, Optional, Sequence

import numpy as np

from actuo.core.contracts import CommandFilter, FilterResult
from actuo.safety import CollisionGate, SelfCollisionMonitor

#: Groups the gate decelerates/stops independently. Each arm judges only its own
#: links' proximity, so one arm approaching the body does not slow the other.
_GROUP_PREFIXES = {
    "rarm": ("rarm", "right_hand"),
    "larm": ("larm", "left_hand"),
    "head": ("head",),
}


def _urdf_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("jedy_description"),
                        "urdf", "jedy_four_dof.urdf")


class JedyCollisionFilter(CommandFilter):
    """Holds the last collision-free command.

    Wraps ``actuo.safety.CollisionGate`` and adapts its result to actuo's
    ``FilterResult``. The gate keeps the safety semantics -- latching, fail
    safe, hold-last-safe -- and this class adds nothing to them, deliberately:
    two places deciding what "safe" means is how they come to disagree.
    """

    def __init__(self, joint_names: Sequence[str], urdf: Optional[str] = None,
                 decelerate: bool = False, latched: bool = False) -> None:
        from skrobot.model import RobotModel

        self.joint_names = list(joint_names)
        robot = RobotModel()
        robot.load_urdf_file(urdf or _urdf_path())
        self.robot = robot

        rest = robot.angle_vector().copy()
        self.monitor = SelfCollisionMonitor(robot, reference_angle_vector=rest)

        named = [link.name for link in robot.link_list
                 if getattr(link, "collision_mesh", None) is not None]
        groups = {name: [n for n in named if n.startswith(prefixes)]
                  for name, prefixes in _GROUP_PREFIXES.items()}
        groups = {k: v for k, v in groups.items() if v}
        # Enables the per-group managers, which are what BOTH the distance
        # queries and the fast boolean path use. Without them a cycle costs
        # 114 ms against a 10 ms budget; with them, 0.42 ms.
        self.monitor.enable_distance(groups)

        start = np.zeros(len(self.joint_names))
        if decelerate:
            scale_groups = {
                name: {"joints": [j for j in self.joint_names
                                  if j.startswith(name)],
                       "links": links}
                for name, links in groups.items()
            }
            self.gate = CollisionGate(
                self.monitor, self.joint_names, base_angle_vector=rest,
                initial_safe=start, mode="scale", scale_groups=scale_groups,
                latched=latched)
        else:
            self.gate = CollisionGate(
                self.monitor, self.joint_names, base_angle_vector=rest,
                initial_safe=start, mode="stop", latched=latched)

        # Which path the gate got is a 250x difference in per-cycle cost, so it
        # is stated at startup rather than discovered in a loop-rate graph.
        print("[jedy_safety] collision filter ready: {} groups, {} structural "
              "pairs ignored, fast path {}".format(
                  len(groups), len(self.monitor.ignored_pairs),
                  "ON" if self.gate.fast_path else "OFF (every cycle pays for "
                  "pair names -- expect ~100 ms)"), flush=True)

    def filter(self, command: np.ndarray) -> FilterResult:
        result = self.gate.filter(np.asarray(command, dtype=float))
        if not result.stopped:
            return FilterResult(applied=np.asarray(result.applied, dtype=float))
        pairs = ", ".join(sorted("{}~{}".format(*sorted(p))
                                 for p in result.colliding_pairs))
        tail = ("Latched: it will not resume until "
                "<node>/filter/release is called."
                if self.gate.latched else
                "It will resume as soon as the command is reachable again.")
        return FilterResult(
            applied=np.asarray(result.applied, dtype=float),
            gated=True,
            reason="self-collision: holding the last collision-free pose ({}). "
                   "{}".format(pairs or "no pair names", tail))

    def reset(self, command: np.ndarray) -> None:
        self.gate.reset(np.asarray(command, dtype=float))


def build(joint_names: List[str]) -> JedyCollisionFilter:
    """Stop at the boundary, resume when the command is reachable again.

    The default, and the one to use while iterating. Note what the latch does
    and does not buy: BOTH modes refuse to apply a colliding command -- the
    gate holds the last collision-free pose either way -- so latching adds a
    required human acknowledgement, not extra prevention. In a trial-and-error
    session that acknowledgement is two extra steps (move the controller, call
    a service) for every mistake, and mistakes are the point of the session.

    Use :func:`build_latched` when someone must notice that it happened.
    """
    return JedyCollisionFilter(joint_names)


def build_latched(joint_names: List[str]) -> JedyCollisionFilter:
    """Stop and STAY stopped until ``<node>/filter/release`` is called.

    The conservative, hrpsys-like behaviour. Worth the friction when the robot
    is doing something unattended or repeated, because a stop that clears
    itself is a stop nobody has to look at -- and the path back to a pose that
    is itself safe may still sweep through the one that was not.
    """
    return JedyCollisionFilter(joint_names, latched=True)


def build_decelerating(joint_names: List[str]) -> JedyCollisionFilter:
    """Slow smoothly with distance instead of stopping at contact.

    The gentlest to work near: the arm feels the boundary rather than hitting
    a wall, and there is nothing to clear afterwards.
    """
    return JedyCollisionFilter(joint_names, decelerate=True)
