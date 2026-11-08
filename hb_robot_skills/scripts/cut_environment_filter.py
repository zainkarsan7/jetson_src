"""Geometry used by the on-demand UniLiDAR capture (metres)."""
import numpy as np
from scipy.spatial import cKDTree


def capsule(points, start, end, radius):
    axis = end - start
    offset = points - start
    t = np.clip((offset @ axis) / max(axis @ axis, 1e-16), 0., 1.)
    delta = offset - t[:, None] * axis
    return np.einsum('ij,ij->i', delta, delta) <= radius * radius


class Extrusion:
    """Mask the union of the actual triangular end-cap prisms, including concavities."""
    def __init__(self, vertices, triangles, margin):
        self.margin = margin
        if not len(vertices) or not len(triangles) or not np.isfinite(vertices).all():
            raise ValueError("Invalid workpiece mesh")
        self.low, self.high = vertices[:, 2].min(), vertices[:, 2].max()
        if self.high <= self.low or not np.all(
                np.isclose(vertices[:, 2], self.low, atol=1e-6, rtol=0) |
                np.isclose(vertices[:, 2], self.high, atol=1e-6, rtol=0)):
            raise ValueError("Expected a finite local-Z profile extrusion")
        faces = vertices[triangles]
        self.cap = faces[np.all(np.abs(faces[:, :, 2] - self.low) < 1e-6, axis=1), :, :2]
        if not len(self.cap):
            raise ValueError("Workpiece has no triangulated lower cap")

    def contains(self, points):
        axial = np.maximum.reduce([self.low - points[:, 2], points[:, 2] - self.high,
                                   np.zeros(len(points))])
        near = axial <= self.margin
        xy = points[near, :2]
        inside = np.zeros(len(xy), dtype=bool)
        radius = np.sqrt(np.maximum(0., self.margin ** 2 - axial[near] ** 2))
        for triangle in self.cap:
            sides = []
            for i in range(3):
                a, b = triangle[i], triangle[(i+1) % 3]
                edge = b - a
                sides.append(edge[0]*(xy[:, 1]-a[1]) - edge[1]*(xy[:, 0]-a[0]))
                inside |= capsule(xy, a, b, radius)
            inside |= np.all(np.array(sides) >= 0, axis=0) | np.all(np.array(sides) <= 0, axis=0)
        near[near] = inside
        return near


class MeshSurface:
    """Index surface samples once, preserving openings in the mount and bracket."""
    def __init__(self, stl_file, spacing=0.012):
        dtype = [('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')]
        faces = np.frombuffer(stl_file.read_bytes()[84:], dtype=dtype)['v']
        samples = []
        for a, b, c in faces:
            n = max(1, int(np.ceil(max(np.linalg.norm(a-b), np.linalg.norm(a-c),
                                       np.linalg.norm(b-c)) / spacing)))
            i, j = np.tril_indices(n+1)
            samples.append(a + ((i-j)/n)[:, None]*(b-a) + (j/n)[:, None]*(c-a))
        self.tree = cKDTree(np.concatenate(samples))
        self.spacing = spacing

    def contains(self, local_points, padding):
        distance, _ = self.tree.query(local_points, distance_upper_bound=padding+self.spacing)
        return np.isfinite(distance)


def occupied_cells(scans, size, min_scans=2):
    votes = [np.unique(np.floor(scan / size).astype(np.int32), axis=0) for scan in scans]
    cells, counts = np.unique(np.concatenate(votes), axis=0, return_counts=True)
    return cells[counts >= min_scans]


def voxel_runs(cells):
    """Coalesce consecutive Z cells without filling gaps."""
    if len(cells):
        split = np.any(np.diff(cells, axis=0) != [0, 0, 1], axis=1)
        for run in np.split(cells, np.flatnonzero(split)+1):
            yield (run[0]+run[-1]+1)*0.5, len(run)
