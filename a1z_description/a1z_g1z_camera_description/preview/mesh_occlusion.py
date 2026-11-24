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
    def __init__(self, package):
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
        self.meshes = []
        robot = ET.parse(package / 'urdf/A1Z_G1Z_CAMERA.urdf').getroot()
        for link in robot.findall('link'):
            # Housing STL does not model transparent optical surfaces. The
            # camera itself is omitted to avoid treating the lens as opaque.
            if link.get('name') in ('camera_link', 'camera_optical_frame'):
                continue
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                filename = package / mesh.get('filename').split('/', 3)[3]
                handle = self.lib.mesh_load(str(filename).encode())
                if not handle:
                    self.close()
                    raise RuntimeError(f'Cannot load binary STL: {filename}')
                self.meshes.append((link.get('name'), visual, handle))
        self.handles = (C.c_void_p * len(self.meshes))(*(m[2] for m in self.meshes))

    def apply(self, poses, origin, rays, distances, image, visual_transform):
        matrices = np.ascontiguousarray([
            poses[name] @ visual_transform(visual) for name, visual, _ in self.meshes
        ], dtype=np.float32)
        mask = np.zeros(len(rays), dtype=np.uint8)
        self.lib.mesh_occlude(self.handles, matrices, len(self.meshes),
                              np.ascontiguousarray(origin, dtype=np.float32),
                              np.ascontiguousarray(rays, dtype=np.float32), len(rays),
                              distances, image, mask)
        return mask

    def close(self):
        for _, _, handle in self.meshes:
            self.lib.mesh_free(handle)
        self.meshes = []
