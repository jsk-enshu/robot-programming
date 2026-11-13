"""Finish the converted CAD meshes, then check every one has geometry.

    python check_cad_meshes.py MESH_DIR EXPECTED_COUNT

Material names: convert-urdf-mesh (trimesh's COLLADA writer) names every
material of every DAE "defaultmaterial". OGRE, which Gazebo renders with,
keeps materials by name, so parts of different colours could be drawn with
one of them. Each material is renamed <file>_<n> here.

Wheel rollers: the CAD colours them 0.29 grey, which renders as a light grey
lost against the yellow frame; on the robot they are black. They are made
near-black (ROLLER_RGB) here, in the material and in the vertex colours.

Geometry: a Draco-compressed glTF read without DracoPy does not fail: it
comes back with the right number of faces and every vertex at (0, 0, 0), and
convert-urdf-mesh still exits 0 and writes empty DAEs. Checked here, so such a
build stops instead of producing an invisible robot.
"""

import glob
import os
import re
import sys

import numpy as np
import trimesh

MATERIAL = re.compile(r'(<material id="[^"]*" name=")([^"]*)(")')
#: The CAD's colour of the wheel rollers, and what they are made instead.
ROLLER_CAD_RGB = (0.29411764705882354,) * 3
ROLLER_RGB = (0.06, 0.06, 0.06)
WHEEL_SUFFIX = '_wheel_link.dae'


def name_materials(text, stem):
    """Give each material a name of its own; the text and how many."""
    count = [0]

    def rename(match):
        count[0] += 1
        return '{}{}_{}{}'.format(match.group(1), stem, count[0] - 1, match.group(3))
    return MATERIAL.sub(rename, text), count[0]


def recolour(text, old, new):
    """Replace one RGB in the materials' colours and in the vertex colours.

    Returns
    -------
    text : str
    materials : int
        Material colours (ambient, diffuse) replaced.
    """
    old_s = ' '.join(repr(float(v)) for v in old)
    new_s = ' '.join(repr(float(v)) for v in new)
    materials = text.count(old_s + ' 1.0</color>')
    text = text.replace(old_s + ' 1.0</color>', new_s + ' 1.0</color>')

    # Vertex colours: the float_array of each source a <triangles> reads as COLOR.
    colour_ids = set(re.findall(r'semantic="COLOR"[^>]*source="#([^"]+)"', text))

    def swap(match):
        values = np.array(match.group(2).split(), dtype=float).reshape(-1, 3)
        hit = np.all(np.isclose(values, old, atol=1e-6), axis=1)
        values[hit] = new
        return match.group(1) + ' '.join(repr(float(v)) for v in values.ravel()) + match.group(3)
    for sid in colour_ids:
        text = re.sub(r'(<source id="%s">\s*<float_array[^>]*>)([^<]*)(</float_array>)'
                      % re.escape(sid), swap, text, count=1)
    return text, materials


def main():
    mesh_dir, expected = sys.argv[1], int(sys.argv[2])
    paths = sorted(glob.glob(os.path.join(mesh_dir, '*.dae')))
    if len(paths) != expected:
        sys.exit('expected {} visual meshes in {}, found {}'.format(expected, mesh_dir, len(paths)))
    flat, named, rollers = [], 0, 0
    for path in paths:
        stem = os.path.splitext(os.path.basename(path))[0]
        with open(path, encoding='utf-8') as f:
            text = f.read()
        text, n = name_materials(text, stem)
        named += n
        if path.endswith(WHEEL_SUFFIX):
            text, n = recolour(text, ROLLER_CAD_RGB, ROLLER_RGB)
            if n == 0:
                sys.exit('{}: no roller colour {} to replace -- did the CAD model change?'.format(
                    os.path.basename(path), ROLLER_CAD_RGB))
            rollers += 1
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        mesh = trimesh.load(path, force='mesh')
        if len(mesh.faces) == 0 or np.ptp(mesh.vertices, axis=0).max() <= 0:
            flat.append(os.path.basename(path))
    if flat:
        sys.exit('meshes without geometry (Draco not decoded?): {}'.format(', '.join(flat)))
    print('jedy_description: {} CAD meshes checked, {} materials named, '
          'rollers of {} wheels made black'.format(len(paths), named, rollers))


if __name__ == '__main__':
    main()
