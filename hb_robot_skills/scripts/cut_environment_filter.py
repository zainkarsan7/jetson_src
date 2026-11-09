"""Geometry used by the on-demand UniLiDAR capture (metres)."""
import numpy as np
from scipy.spatial import cKDTree
import struct

def capsule(points, start, end, radius):
    axis = end - start
    offset = points - start
    t = np.clip((offset @ axis) / max(axis @ axis, 1e-16), 0., 1.)
    delta = offset - t[:, None] * axis
    return np.einsum('ij,ij->i', delta, delta) <= radius * radius




def stl_vertices(path):
    """Read binary or ASCII STL triangle vertices, in mesh-local metres."""
    data = path.read_bytes()

    if len(data) >= 84:
        count = struct.unpack_from('<I', data, 80)[0]
        if 84 + 50 * count == len(data):
            dtype = np.dtype([
                ('normal', '<f4', (3,)),
                ('vertices', '<f4', (3, 3)),
                ('attribute', '<u2')
            ])
            faces = np.frombuffer(data, dtype=dtype, count=count, offset=84)
            return faces['vertices'].reshape(-1, 3).astype(np.float64)

    # ASCII STL fallback.
    points = []
    for line in data.decode('utf-8').splitlines():
        fields = line.strip().split()
        if len(fields) == 4 and fields[0].lower() == 'vertex':
            points.append([float(v) for v in fields[1:]])

    if not points:
        raise ValueError(f'Cannot read STL: {path}')

    return np.asarray(points, dtype=np.float64)


def capsule_from_mesh(vertices):
    """Return local-frame endpoints and radius enclosing all vertices."""
    points = np.asarray(vertices, dtype=np.float64)

    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError('Invalid mesh vertices')

    if not np.isfinite(points).all():
        raise ValueError('Non-finite mesh vertices')

    center = points.mean(axis=0)
    centered = points - center

    covariance = centered.T @ centered / len(points)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)

    axis = eigenvectors[:, -1]
    projections = centered @ axis

    # Full-length segment, deliberately conservative.
    a = center + projections.min() * axis
    b = center + projections.max() * axis

    offset = points - a
    segment = b - a

    t = np.clip(
        (offset @ segment) / max(segment @ segment, 1e-16),
        0.0, 1.0
    )

    closest = a + t[:, None] * segment
    radius = np.linalg.norm(points - closest, axis=1).max()

    return a, b, float(radius)


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
