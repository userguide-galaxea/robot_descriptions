"""Offline checks for three-view FK, fisheye calibration and whole-robot occlusion."""
from pathlib import Path
import ctypes as C
import struct
import tempfile
import time
import cv2
import numpy as np
from run_preview import make_views


def check_optical_channel(occluder, output):
    """Near optical surfaces may be skipped without skipping farther body hits."""
    fixture = output/'optical-channel.stl'
    records = [struct.pack('<12fH', 1, 0, 0, x, -1, -1, x, 1, -1, x, 0, 1, 0)
               for x in (.01, .1)]
    fixture.write_bytes(b'\0'*80 + struct.pack('<I', 2) + b''.join(records))
    handle = occluder.lib.mesh_load(str(fixture).encode())
    assert handle
    try:
        for minimum, scene_depth, expected, hit in [(.0001, 1., .01, 1),
                                                    (.03, 1., .1, 1),
                                                    (.03, .05, .05, 0)]:
            depth = np.array([scene_depth], np.float32)
            mask, rgb = np.zeros(1, np.uint8), np.zeros((1, 3), np.uint8)
            occluder.lib.mesh_occlude_near(
                (C.c_void_p*1)(handle), np.eye(4, dtype=np.float32), 1,
                np.zeros(3, np.float32), np.array([[1., 0, 0]], np.float32), 1,
                depth, rgb, mask, np.array([minimum], np.float32))
            np.testing.assert_allclose(depth[0], expected, atol=1e-7)
            assert mask[0] == hit
    finally:
        occluder.lib.mesh_free(handle)
    print('PASS: optical near surface skipped, farther body and scene depth preserved')


def main():
    output = Path(tempfile.mkdtemp(prefix='lemo-preview-check-'))
    views, occluder = make_views(scale=4)
    try:
        assert len(occluder.meshes) == 21  # platform + 9 arm meshes and 1 camera housing per side
        check_optical_channel(occluder, output)
        images, masks = {}, {}
        for camera, view in views.items():
            start = time.perf_counter()
            pose = view.pose()
            renderer = view.occluder
            view.occluder = None
            baseline = view.render(pose)
            view.occluder = renderer
            image = view.render(pose)
            mask = view.occlusion_mask.reshape(view.height, view.width).copy()
            assert mask.sum() > 0
            assert np.array_equal(image[mask == 0], baseline[mask == 0])
            assert not np.array_equal(image, baseline)
            images[camera], masks[camera] = image, mask
            if camera == 'head':
                assert mask.sum() < view.valid.sum()/2, 'Head optical channel is blocked'
            assert cv2.imwrite(str(output/f'{camera}-zero.png'), image[:, :, ::-1])
            assert cv2.imwrite(str(output/f'{camera}-mask.png'), mask*255)
            print(f'{camera}: {time.perf_counter()-start:.3f}s; occluded '
                  f'{int(mask.sum())}/{int(view.valid.sum())} valid pixels', flush=True)
        # Each wrist housing must actually contribute visible pixels to head.
        head = views['head']
        renderer = head.occluder
        for side in ('L', 'R'):
            class WithoutHousing:
                def apply(self, poses, origin, rays, distances, image, visual_transform):
                    return occluder.apply(poses, origin, rays, distances, image,
                                          visual_transform, near_by_link={'base_link': .030},
                                          excluded_links=(f'{side}_camera_link',))
            head.occluder = WithoutHousing()
            without = head.render(head.pose())
            changed = np.count_nonzero(np.any(without != images['head'], axis=2))
            assert changed > 10, f'{side} wrist housing not visible from head'
            print(f'Visible {side} wrist housing: {changed} head pixels', flush=True)
        head.occluder = renderer
        np.testing.assert_allclose(views['head'].pose()[:3, 3], [.04749, 0, .55059], atol=1e-12)
        np.testing.assert_allclose(views['head'].pose()[:3, 2],
                                   [np.cos(np.deg2rad(59)), 0, -np.sin(np.deg2rad(59))],
                                   atol=1e-12)
        for side in ('L', 'R'):
            view = views[side]
            relative = np.linalg.inv(view.link_poses()[f'{side}_arm_link6']) @ view.pose()
            np.testing.assert_allclose(relative[:3, 3], [.08504, 0, .05898], atol=1e-12)
            np.testing.assert_allclose(relative[:3, 2],
                                       [np.cos(np.deg2rad(28)), 0, -np.sin(np.deg2rad(28))],
                                       atol=1e-12)
            np.testing.assert_array_equal(view.K, views['head'].K)
            np.testing.assert_array_equal(view.D, views['head'].D)
        origin = views['head'].pose()[:3, 3].astype(np.float32)
        ray = views['head'].pose()[:3, 2].astype(np.float32).reshape(1, 3)
        distance, rgb = np.array([10.], np.float32), np.zeros((1, 3), np.uint8)
        views['head'].occluder.apply(views['head'].link_poses(), origin, ray, distance, rgb,
                                     views['head'].visual_transform)
        assert distance[0] > .03, 'Opaque integrated camera surface still blocks head centre ray'
        head_link = views['head'].robot.find("link[@name='head_camera_link']")
        assert head_link.find('visual') is None and head_link.find('collision') is None
        # Independently move each arm: both must be visible from the head.
        for side in ('L','R'):
            head = views['head']
            head.angles = {f'{side}_arm_joint2': .8, f'{side}_arm_joint3': -.6}
            moved = head.render(head.pose())
            changed = np.count_nonzero(np.any(moved != images['head'],axis=2))
            assert changed > 100, f'{side} arm is not visible in the head image'
            print(f'Head view changed by {side} arm: {changed} pixels',flush=True)
            head.angles = {}
        # A left-arm movement moves only its wrist pose; right/head extrinsics
        # stay fixed, but those images must still rerender for mutual occlusion.
        original_poses = {c: v.pose().copy() for c, v in views.items()}
        joints = {'L_arm_joint2': 1., 'L_arm_joint3': -1.,
                  'L_gripper_finger_left_joint': .04,
                  'R_arm_joint2': -.4, 'R_arm_joint3': .7,
                  'R_gripper_finger_left_joint': .025}
        for view in views.values():
            view.angles.update(joints)
        np.testing.assert_allclose(views['head'].pose(), original_poses['head'])
        for camera in ('L','R'):
            view = views[camera]
            assert not np.allclose(view.pose(), original_poses[camera])
            image = view.render(view.pose())
            assert not np.array_equal(image, images[camera])
            assert cv2.imwrite(str(output/f'{camera}-bent.png'), image[:, :, ::-1])
        poses=views['L'].link_poses()
        for camera, opening in [('L',.04),('R',.025)]:
            relative = np.linalg.inv(poses[f'{camera}_arm_link6']) @ poses[f'{camera}_gripper_finger_rIght_link']
            np.testing.assert_allclose(relative[:3,3], [.0727,.025-opening,-.018],atol=1e-12)
        print('PASS: 21 meshes, 3 views, scene depth, joint FK, head extrinsics and both finger mimics')
        print('Verification images:', output)
    finally:
        occluder.close()


if __name__ == '__main__':
    main()
