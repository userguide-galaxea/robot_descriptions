"""ctypes wrapper for the preview's local binary-STL BVH renderer."""
import ctypes as C
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np


class MeshOcclusion:
    def __init__(self, package, urdf='urdf/A1Z_G1Z_CAMERA.urdf',
                 excluded_links=('camera_link', 'camera_optical_frame')):
        source = Path(__file__).with_suffix('.cpp')
        digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
        suffix = '.dylib' if sys.platform == 'darwin' else '.so'
        library = Path(tempfile.gettempdir()) / f'a1z-mesh-occlusion-{digest}{suffix}'
        if not library.exists():
            with tempfile.TemporaryDirectory(prefix='a1z-mesh-build-') as build:
                output = Path(build) / library.name
                subprocess.run(['c++', '-std=c++17', '-O3', '-shared', '-fPIC', '-pthread',
                                str(source), '-o', str(output)], check=True)
                output.replace(library)
        self.lib = C.CDLL(str(library))
        self.lib.mesh_load.argtypes, self.lib.mesh_load.restype = [C.c_char_p], C.c_void_p
        self.lib.mesh_free.argtypes = [C.c_void_p]
        f32 = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
        u8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS')
        self.lib.mesh_occlude.argtypes = [C.POINTER(C.c_void_p), f32, C.c_int,
                                         f32, f32, C.c_int, f32, u8, u8]
        self.lib.mesh_occlude.restype = None
        self.lib.mesh_occlude_near.argtypes = self.lib.mesh_occlude.argtypes + [f32]
        self.lib.mesh_occlude_near.restype = None
        self.meshes = []
        robot = ET.parse(package / urdf).getroot()
        for link in robot.findall('link'):
            # Housing STL does not model transparent optical surfaces. The
            # camera itself is omitted to avoid treating the lens as opaque.
            if link.get('name') in excluded_links:
                continue
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                filename = package / mesh.get('filename').split('/', 3)[3]
                scale = np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
                if not np.array_equal(scale, np.ones(3)):
                    # Bake mesh scale into vertices: the native ray tracer uses
                    # rigid transforms, so a scaled matrix would be incorrect.
                    data = filename.read_bytes()
                    digest = hashlib.sha256(data + scale.tobytes()).hexdigest()[:20]
                    scaled = Path(tempfile.gettempdir()) / f'preview-scaled-{digest}.stl'
                    if not scaled.exists():
                        count = int.from_bytes(data[80:84], 'little')
                        dtype = np.dtype([('normal', '<f4', (3,)),
                                          ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
                        records = np.frombuffer(data, dtype=dtype, offset=84, count=count).copy()
                        records['vertices'] *= scale
                        # Normals are recomputed from triangles by mesh_load.
                        scaled.write_bytes(data[:84] + records.tobytes())
                    filename = scaled
                handle = self.lib.mesh_load(str(filename).encode())
                if not handle:
                    self.close()
                    raise RuntimeError(f'Cannot load binary STL: {filename}')
                self.meshes.append((link.get('name'), visual, handle))
        self.handles = (C.c_void_p * len(self.meshes))(*(m[2] for m in self.meshes))

    def apply(self, poses, origin, rays, distances, image, visual_transform,
              near_by_link=None, excluded_links=()):
        meshes = [m for m in self.meshes if m[0] not in excluded_links]
        handles = (C.c_void_p * len(meshes))(*(m[2] for m in meshes))
        matrices = np.ascontiguousarray([
            poses[name] @ visual_transform(visual) for name, visual, _ in meshes
        ], dtype=np.float32)
        mask = np.zeros(len(rays), dtype=np.uint8)
        minimums = np.array([max(.0001, (near_by_link or {}).get(name, .0001))
                             for name, _, _ in meshes], dtype=np.float32)
        self.lib.mesh_occlude_near(handles, matrices, len(meshes),
                              np.ascontiguousarray(origin, dtype=np.float32),
                              np.ascontiguousarray(rays, dtype=np.float32), len(rays),
                              distances, image, mask, minimums)
        return mask

    def close(self):
        for _, _, handle in self.meshes:
            self.lib.mesh_free(handle)
        self.meshes = []
