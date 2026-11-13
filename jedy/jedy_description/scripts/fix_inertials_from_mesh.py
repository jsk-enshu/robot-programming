#!/usr/bin/env python3
"""Replace inertials whose centre of mass lies outside the link itself.

    python fix_inertials_from_mesh.py ../urdf/jedy_gz.xacro            # report
    python fix_inertials_from_mesh.py ../urdf/jedy_gz.xacro --write    # rewrite

The upstream Jedy URDF (jsk-enshu/robot-programming, jedy_description) gives
the arm links and head_link1 centres of mass 5-18 cm away from links 4 cm
across, outside the link's own collision AND visual mesh: they were exported
in another frame. A simulator that takes the URDF (Gazebo, MuJoCo, and
scikit-robot's torque_vector) then hangs those masses in the wrong place,
while one that rebuilds placeholder inertias from the mesh (scikit-robot's
urdf_to_usd for Isaac) does not, and the simulators disagree about gravity.

For each link whose <inertial><origin xyz> is outside the bounding box of its
collision mesh and of its visual mesh (with a 2 mm margin), this computes the
centre of mass and the inertia tensor (about that centre, in the link frame)
of the collision mesh at uniform density, keeping the URDF mass, and writes
them in place of the old origin and inertia, with a comment saying where the
numbers came from. A mesh that is not watertight is replaced by its convex
hull first (the same rule urdf_to_usd uses). The values are an estimate:
the real mass distribution (servo inside, brackets, cables) is not uniform.

``--min-inertia`` raises each principal moment to at least that value [kg m^2]
(the principal axes are kept). The links here are 7-50 g and a few cm, so
their moments are ~1e-6; Gazebo (DART, 1 ms) goes to NaN on the first step
with those. The floor changes no gravity torque (that needs only the mass
and the COM), only how fast a link spins up; the comment records it.
"""

import argparse
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

#: a COM this far outside the mesh's bounding box counts as outside [m]
MARGIN = 0.002


def resolve(filename, description_dir):
    prefix = 'package://jedy_description/'
    if not filename.startswith(prefix):
        raise SystemExit('cannot resolve {}'.format(filename))
    return os.path.join(description_dir, filename[len(prefix):])


def load_mesh(link, kind, description_dir):
    meshes = []
    for element in link.findall(kind):
        mesh = element.find('geometry/mesh')
        if mesh is None:
            continue
        origin = element.find('origin')
        if origin is not None and (
                any(abs(float(v)) > 0 for v in origin.get('xyz', '0 0 0')
                    .split())
                or any(abs(float(v)) > 0 for v in origin.get('rpy', '0 0 0')
                       .split())):
            raise SystemExit('{}: {} with a non-zero origin is not handled'
                             .format(link.get('name'), kind))
        loaded = trimesh.load(resolve(mesh.get('filename'), description_dir),
                              force='mesh')
        meshes.append(loaded)
    if not meshes:
        return None
    return trimesh.util.concatenate(meshes)


def outside(point, mesh):
    lo, hi = mesh.bounds
    return bool(np.any(point < lo - MARGIN) or np.any(point > hi + MARGIN))


def mesh_inertial(mesh, mass, min_inertia=0.0):
    """(com, 3x3 inertia about com in the link frame, watertight).

    Uniform density; principal moments raised to ``min_inertia``.
    """
    solid = mesh if mesh.is_watertight else mesh.convex_hull
    solid = solid.copy()
    solid.density = mass / solid.volume
    inertia = np.asarray(solid.moment_inertia)
    moments, axes = np.linalg.eigh(inertia)
    moments = np.maximum(moments, min_inertia)
    inertia = axes.dot(np.diag(moments)).dot(axes.T)
    return np.asarray(solid.center_mass), inertia, \
        solid.is_watertight and mesh.is_watertight


def fmt(v):
    return '{:.6g}'.format(float(v))


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('xacro')
    p.add_argument('--write', action='store_true')
    p.add_argument('--min-inertia', type=float, default=0.0,
                   help='floor on each principal moment [kg m^2]')
    args = p.parse_args()
    description_dir = os.path.dirname(os.path.dirname(
        os.path.abspath(args.xacro)))
    text = open(args.xacro).read()
    root = ET.fromstring(text)
    fixed = []
    for link in root.iter('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        name = link.get('name')
        com = np.array([float(v) for v in
                        inertial.find('origin').get('xyz').split()])
        mass = float(inertial.find('mass').get('value'))
        collision = load_mesh(link, 'collision', description_dir)
        visual = load_mesh(link, 'visual', description_dir)
        if collision is None or visual is None:
            continue
        if not (outside(com, collision) and outside(com, visual)):
            continue
        new_com, inertia, watertight = mesh_inertial(
            collision, mass, args.min_inertia)
        print('{:14s} mass {:.4f}  COM {} -> {}  ({})'.format(
            name, mass, np.round(com, 4), np.round(new_com, 4),
            'watertight mesh' if watertight else 'convex hull'))
        fixed.append(name)
        if not args.write:
            continue
        block = re.compile(
            r'(  <link name="{}">\n)    <inertial>\n.*?    </inertial>\n'
            .format(re.escape(name)), re.S)
        if len(block.findall(text)) != 1:
            raise SystemExit('{}: inertial block not found exactly once'
                             .format(name))
        new = (
            '\\1    <!-- inertial: the upstream origin ({}) was outside this '
            'link\'s meshes.\n'
            '         COM and inertia (about the COM, link frame) are those '
            'of the collision mesh{} at\n'
            '         uniform density with the upstream mass{}; estimate,\n'
            '         UNVERIFIED (jedy_description/scripts/'
            'fix_inertials_from_mesh.py). -->\n'
            '    <inertial>\n'
            '      <origin xyz="{} {} {}" rpy="0 0 0"/>\n'
            '      <mass value="{}"/>\n'
            '      <inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" '
            'izz="{}"/>\n'
            '    </inertial>\n').format(
                ' '.join(fmt(v) for v in com),
                '' if watertight else '\'s convex hull',
                '' if not args.min_inertia else
                ', principal moments floored at {:g} for DART'.format(
                    args.min_inertia),
                *(fmt(v) for v in new_com), inertial.find('mass').get('value'),
                *(fmt(inertia[i, j]) for i, j in
                  ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))))
        text = block.sub(new, text)
    if args.write and fixed:
        with open(args.xacro, 'w') as f:
            f.write(text)
        print('rewrote {} inertials in {}'.format(len(fixed), args.xacro))


if __name__ == '__main__':
    main()
