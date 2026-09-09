#!/usr/bin/env python3
"""Generate Nav2 occupancy maps (PGM + YAML) directly from Gazebo world SDFs.

No SLAM run needed. First a ground-height raster is built from near-horizontal
collision faces (lowest surface wins, so floors beat roofs). Every cell with a
ground surface is free. Then any face occupying the band ``--z-min``..``--z-max``
*above the local ground* is projected as occupied; this keeps sloped outdoor
terrain free while walls, shelves, tree trunks and cliffs become obstacles.
Small standalone models (furniture, docks) are filled by their convex hull so
hollow boxes don't leave free interiors. Everything else stays unknown.

Requires: numpy, pillow, pyyaml, trimesh, pycollada (for .dae meshes).
The system ROS python lacks trimesh; run with the conda interpreter or
``pip install trimesh pycollada``.

Usage::

    python3 generate_world_maps.py --all
    python3 generate_world_maps.py --world office --world construction
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import warnings
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml
from PIL import Image, ImageDraw

OCCUPIED = 0
FREE = 254
UNKNOWN = 205

WORLDS = ('construction', 'office', 'orchard', 'pipeline', 'solar_farm', 'warehouse')

_HOME = os.path.expanduser('~')
_DEFAULT_WORLD_DIRS = [
    os.path.join(_HOME, 'clearpath_ws/install/clearpath_gz/share/clearpath_gz/worlds'),
    os.path.join(_HOME, 'clearpath_ws/src/clearpath_simulator/clearpath_gz/worlds'),
]
_DEFAULT_RESOURCE_DIRS = [
    os.path.join(_HOME, 'clearpath_ws/install/clearpath_gz/share/clearpath_gz/meshes'),
    os.path.join(_HOME, 'clearpath_ws/install/clearpath_gz/share/clearpath_gz/worlds'),
    os.path.join(_HOME, 'clearpath_ws/src/clearpath_simulator/clearpath_gz/meshes'),
]
_FUEL_CACHE = os.path.join(_HOME, '.gz/fuel')


# --------------------------------------------------------------------------- SDF helpers
def _text(elem, default=''):
    return (elem.text or default).strip() if elem is not None else default


def _floats(elem, default):
    txt = _text(elem)
    if not txt:
        return list(default)
    return [float(v) for v in txt.split()]


def pose_matrix(pose_elem) -> np.ndarray:
    """SDF <pose>x y z roll pitch yaw</pose> -> 4x4 (fixed-axis RPY)."""
    x, y, z, roll, pitch, yaw = _floats(pose_elem, (0, 0, 0, 0, 0, 0))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = (x, y, z)
    return mat


class UriResolver:
    def __init__(self, resource_dirs):
        env_dirs = [d for d in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':') if d]
        self.resource_dirs = list(resource_dirs) + env_dirs
        self.warned = set()

    def _warn(self, msg):
        if msg not in self.warned:
            self.warned.add(msg)
            print(f'  [warn] {msg}', file=sys.stderr)

    def model_dir(self, uri: str):
        """Resolve an <include><uri> to a model directory (Fuel cache or resource path)."""
        uri = uri.strip()
        if uri.startswith(('http://', 'https://')):
            # https://fuel.gazebosim.org/1.0/Owner/models/Name[/version]
            parts = uri.rstrip('/').split('/')
            try:
                idx = parts.index('models')
                owner, name = parts[idx - 1].lower(), parts[idx + 1].lower()
            except (ValueError, IndexError):
                self._warn(f'unparseable fuel uri {uri}')
                return None
            name = name.replace('%20', ' ')
            base = os.path.join(_FUEL_CACHE, 'fuel.gazebosim.org', owner, 'models', name)
            versions = sorted(
                (d for d in glob.glob(os.path.join(base, '*')) if os.path.isdir(d)),
                key=lambda d: int(os.path.basename(d)) if os.path.basename(d).isdigit() else -1,
            )
            if versions:
                return versions[-1]
            self._warn(f'fuel model not cached: {uri} (expected under {base})')
            return None
        if uri.startswith('model://'):
            rel = uri[len('model://'):]
            for root in self.resource_dirs:
                cand = os.path.join(root, rel)
                if os.path.isdir(cand):
                    return cand
        if os.path.isdir(uri):
            return uri
        self._warn(f'cannot resolve model uri {uri}')
        return None

    def mesh_path(self, uri: str, model_dir):
        uri = uri.strip()
        if uri.startswith('model://'):
            rel = uri[len('model://'):]
            # model://ModelName/meshes/x.dae inside a Fuel model -> strip model name
            if model_dir:
                inner = rel.split('/', 1)[1] if '/' in rel else rel
                for cand in (os.path.join(model_dir, inner), os.path.join(model_dir, rel)):
                    if os.path.isfile(cand):
                        return cand
            for root in self.resource_dirs:
                cand = os.path.join(root, rel)
                if os.path.isfile(cand):
                    return cand
        elif uri.startswith('file://'):
            path = uri[len('file://'):]
            if os.path.isfile(path):
                return path
        elif uri.startswith(('http://', 'https://')):
            # https://fuel.gazebosim.org/1.0/owner/models/name/ver/files/meshes/X.obj
            if '/files/' in uri:
                rel = uri.split('/files/', 1)[1]
                mdir = model_dir or self.model_dir(uri.split('/files/', 1)[0])
                if mdir:
                    cand = os.path.join(mdir, rel)
                    if os.path.isfile(cand):
                        return cand
        else:
            for base in ([model_dir] if model_dir else []) + self.resource_dirs:
                cand = os.path.join(base, uri)
                if os.path.isfile(cand):
                    return cand
            if os.path.isfile(uri):
                return uri
        self._warn(f'cannot resolve mesh uri {uri}')
        return None


# --------------------------------------------------------------------------- geometry collection
class GeometryCollector:
    def __init__(self, resolver: UriResolver, prefer_collision=True):
        self.resolver = resolver
        self.prefer_collision = prefer_collision
        self.meshes: list[trimesh.Trimesh] = []
        self.has_ground_plane = False
        self._mesh_cache: dict[str, trimesh.Trimesh] = {}

    # -- entry points
    def add_world(self, world_sdf: str):
        root = ET.parse(world_sdf).getroot()
        world = root.find('world')
        if world is None:
            raise RuntimeError(f'{world_sdf} has no <world>')
        self._add_children(world, np.eye(4), model_dir=None)

    def _add_children(self, parent, parent_tf, model_dir):
        for model in parent.findall('model'):
            self._add_model(model, parent_tf, model_dir)
        for include in parent.findall('include'):
            self._add_include(include, parent_tf)

    def _add_include(self, include, parent_tf):
        uri = _text(include.find('uri'))
        mdir = self.resolver.model_dir(uri)
        tf = parent_tf @ pose_matrix(include.find('pose'))
        if not mdir:
            return
        sdf_file = self._model_sdf(mdir)
        if not sdf_file:
            print(f'  [warn] no model.sdf in {mdir}', file=sys.stderr)
            return
        root = ET.parse(sdf_file).getroot()
        model = root.find('model')
        if model is None:
            return
        # The include pose replaces the model's own top-level pose.
        self._add_model(model, tf, mdir, override_pose=True)

    @staticmethod
    def _model_sdf(mdir):
        cfg = os.path.join(mdir, 'model.config')
        if os.path.isfile(cfg):
            try:
                for sdf in ET.parse(cfg).getroot().findall('sdf'):
                    cand = os.path.join(mdir, _text(sdf))
                    if os.path.isfile(cand):
                        return cand
            except ET.ParseError:
                pass
        cand = os.path.join(mdir, 'model.sdf')
        return cand if os.path.isfile(cand) else None

    def _add_model(self, model, parent_tf, model_dir, override_pose=False):
        tf = parent_tf if override_pose else parent_tf @ pose_matrix(model.find('pose'))
        for link in model.findall('link'):
            link_tf = tf @ pose_matrix(link.find('pose'))
            # Visual-only links (e.g. water surfaces) have no physics, so skip them.
            shapes = link.findall('collision' if self.prefer_collision else 'visual')
            for shape in shapes:
                self._add_shape(shape, link_tf, model_dir)
        # nested models / includes
        self._add_children(model, tf, model_dir)

    def _add_shape(self, shape, link_tf, model_dir):
        geom = shape.find('geometry')
        if geom is None:
            return
        tf = link_tf @ pose_matrix(shape.find('pose'))
        mesh_elem = geom.find('mesh')
        if mesh_elem is not None:
            path = self.resolver.mesh_path(_text(mesh_elem.find('uri')), model_dir)
            if not path:
                return
            scale = _floats(mesh_elem.find('scale'), (1, 1, 1))
            base = self._load_mesh(path)
            if base is None:
                return
            m = base.copy()
            if any(abs(s - 1.0) > 1e-9 for s in scale):
                m.apply_scale(scale)
            m.apply_transform(tf)
            self.meshes.append(m)
            return
        box = geom.find('box')
        if box is not None:
            size = _floats(box.find('size'), (1, 1, 1))
            m = trimesh.creation.box(extents=size)
            m.apply_transform(tf)
            self.meshes.append(m)
            return
        cyl = geom.find('cylinder')
        if cyl is not None:
            r = float(_text(cyl.find('radius'), '0.5'))
            h = float(_text(cyl.find('length'), '1.0'))
            m = trimesh.creation.cylinder(radius=r, height=h, sections=24)
            m.apply_transform(tf)
            self.meshes.append(m)
            return
        sph = geom.find('sphere')
        if sph is not None:
            r = float(_text(sph.find('radius'), '0.5'))
            m = trimesh.creation.icosphere(subdivisions=2, radius=r)
            m.apply_transform(tf)
            self.meshes.append(m)
            return
        if geom.find('plane') is not None:
            # Infinite ground plane: the whole map extent is drivable unless blocked.
            self.has_ground_plane = True
        # heightmap / polyline: unsupported, ignored

    def _load_mesh(self, path):
        if path in self._mesh_cache:
            return self._mesh_cache[path]
        try:
            loaded = trimesh.load(path, force='mesh', process=False)
        except Exception as exc:  # noqa: BLE001
            print(f'  [warn] failed to load {path}: {exc}', file=sys.stderr)
            self._mesh_cache[path] = None
            return None
        if isinstance(loaded, trimesh.Scene):
            loaded = trimesh.util.concatenate(tuple(loaded.dump()))
        if not isinstance(loaded, trimesh.Trimesh) or loaded.faces.shape[0] == 0:
            print(f'  [warn] {path} has no triangle geometry', file=sys.stderr)
            self._mesh_cache[path] = None
            return None
        self._mesh_cache[path] = loaded
        return loaded

    def combined(self) -> trimesh.Trimesh:
        if not self.meshes:
            raise RuntimeError('no geometry collected')
        return trimesh.util.concatenate(self.meshes)


# --------------------------------------------------------------------------- rasterisation
class Grid:
    def __init__(self, xmin, ymin, xmax, ymax, resolution):
        self.res = resolution
        self.xmin, self.ymin = xmin, ymin
        self.width = int(math.ceil((xmax - xmin) / resolution))
        self.height = int(math.ceil((ymax - ymin) / resolution))
        self.ymax = ymin + self.height * resolution

    def to_px(self, xy: np.ndarray) -> np.ndarray:
        """World XY (n,2) -> pixel (col,row) with row 0 at top (max y)."""
        col = (xy[:, 0] - self.xmin) / self.res
        row = (self.ymax - xy[:, 1]) / self.res
        return np.stack([col, row], axis=1)


def _fill_faces(draw, grid, faces, colour, width):
    for face in faces:
        pts = [tuple(p) for p in grid.to_px(face[:, :2])]
        draw.polygon(pts, fill=colour, outline=colour)
        # polygon() drops degenerate (edge-on wall) triangles; lines keep them.
        draw.line(pts + pts[:1], fill=colour, width=width)


def _convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """Andrew's monotone chain; returns hull vertices in CCW order."""
    pts = np.unique(points, axis=0)
    if len(pts) < 3:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def _paint_ground_face(ground: np.ndarray, px: np.ndarray, z: np.ndarray):
    """Min-blend one triangle (pixel coords ``px`` (3,2), heights ``z`` (3,)) into ``ground``."""
    h, w = ground.shape
    c0 = max(int(np.floor(px[:, 0].min())), 0)
    c1 = min(int(np.ceil(px[:, 0].max())), w - 1)
    r0 = max(int(np.floor(px[:, 1].min())), 0)
    r1 = min(int(np.ceil(px[:, 1].max())), h - 1)
    if c1 < c0 or r1 < r0:
        return
    (x0, y0), (x1, y1), (x2, y2) = px
    det = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    if abs(det) > 1e-9:
        cols = np.arange(c0, c1 + 1) + 0.5
        rows = np.arange(r0, r1 + 1) + 0.5
        X, Y = np.meshgrid(cols, rows)
        l1 = ((X - x0) * (y2 - y0) - (x2 - x0) * (Y - y0)) / det
        l2 = ((x1 - x0) * (Y - y0) - (X - x0) * (y1 - y0)) / det
        l0 = 1.0 - l1 - l2
        eps = -1e-6
        inside = (l0 >= eps) & (l1 >= eps) & (l2 >= eps)
        if inside.any():
            zi = l0 * z[0] + l1 * z[1] + l2 * z[2]
            sub = ground[r0:r1 + 1, c0:c1 + 1]
            sub[inside] = np.fmin(sub[inside], zi[inside])
    # Sub-pixel triangles may miss every pixel centre; always mark the centroid cell.
    cc = int(np.clip(px[:, 0].mean(), 0, w - 1))
    rc = int(np.clip(px[:, 1].mean(), 0, h - 1))
    ground[rc, cc] = np.fmin(ground[rc, cc], z.mean())


def build_ground_raster(meshes, grid: Grid, ground_min, ground_max, ground_plane):
    """Per-cell ground height (m); NaN where nothing walkable exists.

    Near-horizontal faces are barycentrically interpolated and min-blended, so
    the *lowest* surface wins (floor under a table, terrain under a roof) and
    sloped terrain is continuous rather than stepped.
    """
    ground = np.full((grid.height, grid.width), 0.0 if ground_plane else np.nan)
    n_faces = 0
    for mesh in meshes:
        tri = mesh.triangles
        nz = np.abs(mesh.face_normals[:, 2])
        zmax_face = tri[:, :, 2].max(axis=1)
        zmin_face = tri[:, :, 2].min(axis=1)
        mask = (nz > 0.5) & (zmax_face <= ground_max) & (zmin_face >= ground_min)
        faces = tri[mask]
        px_all = grid.to_px(faces[:, :, :2].reshape(-1, 2)).reshape(-1, 3, 2)
        for px, face in zip(px_all, faces):
            _paint_ground_face(ground, px, face[:, 2])
        n_faces += len(faces)
    return ground, n_faces


def _fill_unknown_ground(ground: np.ndarray, iterations: int) -> np.ndarray:
    """Extend known ground into neighbouring unknown cells (min of neighbours)."""
    g = ground.copy()
    for _ in range(iterations):
        nan = np.isnan(g)
        if not nan.any():
            break
        padded = np.pad(g, 1, constant_values=np.nan)
        stack = np.stack([
            padded[:-2, 1:-1], padded[2:, 1:-1], padded[1:-1, :-2], padded[1:-1, 2:],
            padded[:-2, :-2], padded[:-2, 2:], padded[2:, :-2], padded[2:, 2:],
        ])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)  # all-NaN neighbourhoods
            neigh = np.nanmin(stack, axis=0)
        g[nan] = neigh[nan]
    return g


def _sample_ground(ground: np.ndarray, grid: Grid, pts_xy: np.ndarray) -> np.ndarray:
    px = grid.to_px(pts_xy)
    col = np.clip(px[:, 0].astype(int), 0, grid.width - 1)
    row = np.clip(px[:, 1].astype(int), 0, grid.height - 1)
    return ground[row, col]


def rasterise(meshes, grid: Grid, z_min, z_max, ground_min, ground_max, line_width,
              ground_plane: bool, small_object_max: float):
    ground, n_floor = build_ground_raster(meshes, grid, ground_min, ground_max, ground_plane)
    ground_ext = _fill_unknown_ground(ground, iterations=int(round(0.5 / grid.res)))

    img = Image.new('L', (grid.width, grid.height), UNKNOWN)
    img_arr = np.array(img)
    img_arr[~np.isnan(ground)] = FREE
    img = Image.fromarray(img_arr, mode='L')
    draw = ImageDraw.Draw(img)

    n_obs = 0
    obstacle_layers = []
    for mesh in meshes:
        tri = mesh.triangles  # (n,3,3)
        nz = np.abs(mesh.face_normals[:, 2])

        # Local ground under each face: lowest known value at its vertices / centroid.
        sample_pts = np.concatenate([tri[:, :, :2].reshape(-1, 2), tri[:, :, :2].mean(axis=1)])
        g = _sample_ground(ground_ext, grid, sample_pts)
        g = np.concatenate([g[:len(tri) * 3].reshape(-1, 3), g[len(tri) * 3:, None]], axis=1)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            g_face = np.nanmin(g, axis=1)
        has_ground = ~np.isnan(g_face)
        rel_max = tri[:, :, 2].max(axis=1) - g_face
        rel_min = tri[:, :, 2].min(axis=1) - g_face

        in_slab = has_ground & (rel_max >= z_min) & (rel_min <= z_max)
        footprint = mesh.bounds[1, :2] - mesh.bounds[0, :2]
        if in_slab.any() and footprint.max() <= small_object_max:
            # Furniture / shelves / docks: fill the whole footprint of the part inside
            # the slab, so hollow boxes don't leave a "free" interior.
            hull = _convex_hull_2d(tri[in_slab].reshape(-1, 3)[:, :2])
            obstacle_layers.append(('hull', hull))
            n_obs += int(in_slab.sum())
            continue

        steep = in_slab & (nz <= 0.7)                            # walls, posts, trunks, cliffs
        flat_high = in_slab & (nz > 0.7) & (rel_min >= z_min)    # shelf boards, table tops
        obstacle_layers.append(('faces', tri[steep | flat_high]))
        n_obs += int((steep | flat_high).sum())

    for kind, data in obstacle_layers:
        if kind == 'hull':
            if len(data) >= 3:
                pts = [tuple(p) for p in grid.to_px(data)]
                draw.polygon(pts, fill=OCCUPIED, outline=OCCUPIED)
                draw.line(pts + pts[:1], fill=OCCUPIED, width=line_width)
        else:
            _fill_faces(draw, grid, data, OCCUPIED, line_width)
    return img, n_floor, n_obs


def compute_bounds(mesh: trimesh.Trimesh, z_max, margin):
    """XY extent of geometry that is relevant to the map (ignore sky / tall roofs)."""
    tri = mesh.triangles
    zmin_face = tri[:, :, 2].min(axis=1)
    relevant = tri[zmin_face < z_max + 3.0]
    if len(relevant) == 0:
        relevant = tri
    pts = relevant.reshape(-1, 3)[:, :2]
    xmin, ymin = pts.min(axis=0) - margin
    xmax, ymax = pts.max(axis=0) + margin
    return xmin, ymin, xmax, ymax


# --------------------------------------------------------------------------- driver
def find_world_sdf(world: str, world_dirs):
    for d in world_dirs:
        cand = os.path.join(d, f'{world}.sdf')
        if os.path.isfile(cand):
            return cand
    raise FileNotFoundError(f'{world}.sdf not found in {world_dirs}')


def generate(world: str, args, resolver: UriResolver):
    sdf = find_world_sdf(world, args.world_dir)
    print(f'[{world}] parsing {sdf}')
    collector = GeometryCollector(resolver, prefer_collision=not args.use_visuals)
    collector.add_world(sdf)
    mesh = collector.combined()
    print(f'[{world}] {len(collector.meshes)} shapes, {len(mesh.faces)} triangles')

    xmin, ymin, xmax, ymax = compute_bounds(mesh, args.ground_max, args.margin)
    if args.max_extent:
        half = args.max_extent / 2
        xmin, xmax = max(xmin, -half), min(xmax, half)
        ymin, ymax = max(ymin, -half), min(ymax, half)
    grid = Grid(xmin, ymin, xmax, ymax, args.resolution)
    print(f'[{world}] extent x[{xmin:.1f},{xmax:.1f}] y[{ymin:.1f},{ymax:.1f}] '
          f'-> {grid.width}x{grid.height} px @ {args.resolution} m')

    img, n_floor, n_obs = rasterise(
        collector.meshes, grid, args.z_min, args.z_max, args.ground_min, args.ground_max,
        args.line_width, collector.has_ground_plane, args.small_object_max)
    print(f'[{world}] {n_floor} floor faces, {n_obs} obstacle faces'
          f'{", ground plane" if collector.has_ground_plane else ""}')

    os.makedirs(args.out, exist_ok=True)
    pgm = os.path.join(args.out, f'{world}.pgm')
    yml = os.path.join(args.out, f'{world}.yaml')
    img.save(pgm)
    meta = {
        'image': f'{world}.pgm',
        'mode': 'trinary',
        'resolution': float(args.resolution),
        'origin': [round(float(grid.xmin), 4), round(float(grid.ymin), 4), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    with open(yml, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)
    print(f'[{world}] wrote {pgm} and {yml}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', action='append', choices=WORLDS, help='world to map (repeatable)')
    parser.add_argument('--all', action='store_true', help='generate all Clearpath worlds')
    parser.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps'))
    parser.add_argument('--resolution', type=float, default=0.05, help='m/pixel')
    parser.add_argument('--z-min', type=float, default=0.15,
                        help='obstacle band lower bound, relative to local ground (m)')
    parser.add_argument('--z-max', type=float, default=1.0,
                        help='obstacle band upper bound, relative to local ground (m)')
    parser.add_argument('--ground-min', type=float, default=-20.0,
                        help='surfaces below this absolute z are never ground')
    parser.add_argument('--ground-max', type=float, default=8.0,
                        help='surfaces above this absolute z are never ground (roofs)')
    parser.add_argument('--margin', type=float, default=1.0, help='padding around geometry (m)')
    parser.add_argument('--max-extent', type=float, default=400.0, help='clip map to this square size (m)')
    parser.add_argument('--line-width', type=int, default=1)
    parser.add_argument('--small-object-max', type=float, default=25.0,
                        help='shapes with a footprint up to this size (m) are filled by convex hull')
    parser.add_argument('--use-visuals', action='store_true',
                        help='use <visual> geometry (default: <collision>, visual-only links ignored)')
    parser.add_argument('--world-dir', action='append', default=None)
    parser.add_argument('--resource-dir', action='append', default=None)
    args = parser.parse_args()

    worlds = list(WORLDS) if args.all else (args.world or [])
    if not worlds:
        parser.error('specify --world <name> or --all')
    args.world_dir = args.world_dir or _DEFAULT_WORLD_DIRS
    args.out = os.path.abspath(args.out)
    resolver = UriResolver(args.resource_dir or _DEFAULT_RESOURCE_DIRS)

    failures = []
    for world in worlds:
        try:
            generate(world, args, resolver)
        except Exception as exc:  # noqa: BLE001
            failures.append(world)
            print(f'[{world}] FAILED: {exc}', file=sys.stderr)
    if failures:
        sys.exit(f'failed: {", ".join(failures)}')


if __name__ == '__main__':
    main()
