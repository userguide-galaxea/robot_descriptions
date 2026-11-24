"""Offline geometric checks and example images; no ROS nodes are launched."""
import ctypes as C
from pathlib import Path
import struct
import tempfile
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

from mesh_occlusion import MeshOcclusion
from run_preview import PKG, Preview, load_chain


def make_offline_preview():
    # Initialize only the pure geometry fields; do not initialize a ROS node.
    node = Preview.__new__(Preview)
    node.width, node.height = 960, 600
    calibration = yaml.safe_load((PKG / 'config/camera_info.yaml').read_text())
    node.K = np.array(calibration['camera_matrix']['data']).reshape(3, 3)
    node.K[:2] /= 2
    node.D = np.array(calibration['distortion_coefficients']['data'])
    node.chain = load_chain()
    node.robot = ET.parse(PKG / 'urdf/A1Z_G1Z_CAMERA.urdf').getroot()
    node.angles = {}
    node.rays, node.valid = node.make_rays()
    node.occluder = None
    node.occlusion_mask = np.zeros(len(node.rays), np.uint8)
    return node


def check_triangle_intersections(occluder, output):
    fixture = output / 'test-triangle.stl'
    record = struct.pack('<12fH', 1, 0, 0, 1, -1, -1, 1, 1, -1, 1, 0, 1, 0)
    fixture.write_bytes(b'\0' * 80 + struct.pack('<I', 1) + record)
    handle = occluder.lib.mesh_load(str(fixture).encode())
    assert handle, 'Failed to load triangle fixture'
    handles = (C.c_void_p * 1)(handle)
    try:
        cases = [(0, False, [1, 0, 0], 1),
                 (1, False, [1, 0, 0], 2),
                 (0, True, [0, 1, 0], 1)]
        for translation, rotation, ray, expected in cases:
            matrix = np.eye(4, dtype=np.float32)
            if rotation:
                matrix[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
            matrix[0, 3] = translation
            rays = np.array([ray, [0, 0, 1], ray], np.float32)
            distance = np.array([10, 10, .5], np.float32)
            rgb = np.zeros((3, 3), np.uint8)
            mask = np.zeros(3, np.uint8)
            occluder.lib.mesh_occlude(
                handles, matrix, 1, np.zeros(3, np.float32), rays, 3,
                distance, rgb, mask)
            assert abs(distance[0] - expected) < 1e-6, distance
            assert mask.tolist() == [1, 0, 0], mask
    finally:
        occluder.lib.mesh_free(handle)
    print('PASS: triangle hit/miss, scene depth and translated/rotated meshes', flush=True)


def save_image(path, rgb):
    assert cv2.imwrite(str(path), rgb[:, :, ::-1]), f'Cannot save {path}'


def main():
    output = Path(tempfile.mkdtemp(prefix='a1z-occlusion-check-'))
    print('Verification images:', output, flush=True)
    node = make_offline_preview()
    baseline = node.render(node.pose())
    save_image(output / 'a1z-no-occlusion.png', baseline)
    node.occluder = MeshOcclusion(PKG)
    try:
        assert len(node.occluder.meshes) == 9
        check_triangle_intersections(node.occluder, output)
        start = time.perf_counter()
        image = node.render(node.pose())
        mask = node.occlusion_mask.reshape(node.height, node.width)
        print(f'Zero-pose render: {time.perf_counter() - start:.3f} s; '
              f'occluded pixels: {int(mask.sum())}', flush=True)
        assert mask.sum() > 0
        assert np.array_equal(image[mask == 0], baseline[mask == 0])
        assert np.allclose(node.link_poses()['camera_optical_frame'], node.pose(), atol=1e-12)
        assert np.allclose(node.pose()[:3, 3], [.18354, 0, .23698], atol=1e-12)
        save_image(output / 'a1z-self-occlusion.png', image)
        assert cv2.imwrite(str(output / 'a1z-occlusion-mask.png'), mask * 255)

        node.angles = {'arm_joint2': 1., 'arm_joint3': -1.,
                       'gripper_finger_left_joint': .04}
        poses = node.link_poses()
        assert abs(poses['gripper_finger_rIght_link'][1, 3] + .015) < 1e-7
        assert np.allclose(poses['camera_optical_frame'], node.pose(), atol=1e-12)
        changed = node.render(node.pose())
        assert not np.array_equal(image, changed)
        save_image(output / 'a1z-self-occlusion-bent.png', changed)
        print('PASS: actual STL self-occlusion, preserved scene pixels, '
              'joint FK and finger mimic', flush=True)
    finally:
        node.occluder.close()


if __name__ == '__main__':
    main()
