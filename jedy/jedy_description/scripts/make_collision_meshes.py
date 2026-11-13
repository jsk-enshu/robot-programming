#!/usr/bin/env python3
"""Give every link a CoACD collision mesh, and point the URDF at it.

Why this exists, measured on this robot: the URDF shipped with the *visual*
mesh as its collision geometry, 1,018,358 triangles in total (base_link alone
332,517). One self-collision check on that model took **1.0 second**. The bus
servicer's cycle is 10 ms, so a collision gate could not run at all -- it was
100x too slow, and no amount of tuning fixes a triangle count.

CoACD decomposes each mesh into approximate convex parts, which is what a
collision checker actually wants: convex-convex is the fast path, and the parts
approximate the shape far better than a single hull of a concave part would.
Each link gets its parts as several <collision> elements; the <visual> geometry
is left completely alone, so nothing about how the robot LOOKS changes.

Run it from this package (it needs skrobot + coacd + trimesh)::

    <venv>/bin/python scripts/make_collision_meshes.py urdf/jedy_fullbody.urdf

It rewrites the URDF in place and writes meshes to meshes/collision/<link>_N.stl.
Idempotent: a link whose collision already points into meshes/collision/ is
skipped unless --force is given.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import xml.etree.ElementTree as ET

#: Decimate before decomposing. CoACD is superlinear in input size and a
#: 332k-triangle input costs minutes; the shape it recovers from 20k is the same
#: to well inside the margin any collision gate uses. This is a speed knob for
#: the CONVERSION, not for the result -- the output size is set by CoACD's own
#: threshold, not by this.
DECIMATE_TO = 20000

COLLISION_DIR = os.path.join("meshes", "collision")


def _mesh_path(package_dir, uri):
    """`package://jedy_description/meshes/x.dae` -> absolute path."""
    prefix = "package://"
    if not uri.startswith(prefix):
        return None
    _, _, relative = uri[len(prefix):].partition("/")
    return os.path.join(package_dir, relative)


def decompose(mesh, quality, log):
    import trimesh
    from skrobot.utils import convex_decomposition

    if len(mesh.faces) > DECIMATE_TO:
        before = len(mesh.faces)
        mesh = mesh.simplify_quadric_decimation(face_count=DECIMATE_TO)
        log("      decimated {:,} -> {:,} triangles for CoACD".format(
            before, len(mesh.faces)))
    parts = convex_decomposition(mesh, quality=quality)
    return [trimesh.Trimesh(vertices=p.vertices, faces=p.faces) for p in parts]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf")
    parser.add_argument("--quality", default="balanced",
                        choices=["balanced", "fine"])
    parser.add_argument("--force", action="store_true",
                        help="re-decompose links already pointing at "
                             "meshes/collision/")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    urdf_path = os.path.abspath(args.urdf)
    package_dir = os.path.dirname(os.path.dirname(urdf_path))
    out_dir = os.path.join(package_dir, COLLISION_DIR)
    if not args.dry_run:
        os.makedirs(out_dir, exist_ok=True)

    import trimesh

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    def log(message):
        print(message, flush=True)

    total_before = total_after = 0
    changed = 0
    for link in root.findall("link"):
        name = link.attrib.get("name", "?")
        collisions = link.findall("collision")
        if not collisions:
            continue

        uris = [c.find("geometry/mesh") for c in collisions]
        uris = [m.attrib.get("filename") for m in uris if m is not None]
        if not uris:
            continue
        if any(COLLISION_DIR in u for u in uris) and not args.force:
            log("{:38s} already decomposed, skipping".format(name))
            continue

        source = _mesh_path(package_dir, uris[0])
        if source is None or not os.path.exists(source):
            log("{:38s} SKIPPED: no such mesh {}".format(name, uris[0]))
            continue

        mesh = trimesh.load(source, force="mesh")
        started = time.time()
        parts = decompose(mesh, args.quality, log)
        after = sum(len(p.faces) for p in parts)
        total_before += len(mesh.faces)
        total_after += after
        log("{:38s} {:>9,} -> {:>3} parts, {:>7,} triangles  ({:.1f}s)".format(
            name, len(mesh.faces), len(parts), after, time.time() - started))
        if args.dry_run:
            continue

        # Keep the ORIGIN of the collision element being replaced: it places the
        # mesh in the link frame, and the decomposition is in that same frame.
        origin = collisions[0].find("origin")
        for element in collisions:
            link.remove(element)
        for index, part in enumerate(parts):
            filename = "{}_{}.stl".format(name, index)
            part.export(os.path.join(out_dir, filename))
            collision = ET.SubElement(link, "collision")
            if origin is not None:
                collision.append(origin)
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(geometry, "mesh", {
                "filename": "package://{}/{}/{}".format(
                    os.path.basename(package_dir), COLLISION_DIR, filename)})
        changed += 1

    log("")
    log("{} link(s) rewritten: {:,} -> {:,} collision triangles".format(
        changed, total_before, total_after))
    if not args.dry_run and changed:
        tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
        log("wrote {}".format(urdf_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
