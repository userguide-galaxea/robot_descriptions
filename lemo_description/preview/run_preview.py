#!/usr/bin/env python3
"""LEMO ROS 2 preview: three calibrated fisheye views and whole-robot occlusion."""
import argparse
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, Image, CameraInfo
from visualization_msgs.msg import MarkerArray

PKG = Path(__file__).resolve().parents[1]
NAME = 'lemo_description'
CAMERAS = ('L', 'R', 'head')
# Reuse the A1Z ray model, scene, FK and BVH implementation without copying it.
SHARED = PKG.parent/'a1z_description/a1z_g1z_camera_description/preview'
if not SHARED.is_dir():
    SHARED = PKG.parent/'a1z_g1z_camera_description/preview'
if not SHARED.is_dir():
    from ament_index_python.packages import get_package_share_directory
    SHARED = Path(get_package_share_directory('a1z_g1z_camera_description'))/'preview'
sys.path.insert(0, str(SHARED))
spec = importlib.util.spec_from_file_location('a1z_preview_core', SHARED/'run_preview.py')
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)
from mesh_occlusion import MeshOcclusion


class ViewOcclusion:
    """Exclude only the observing camera's own opaque optical surfaces.

    This is a preview approximation, not a measured lens aperture. More distant
    base surfaces and every arm/finger surface retain normal depth testing.
    """
    def __init__(self, occluder, camera):
        self.occluder, self.camera = occluder, camera

    def apply(self, poses, origin, rays, distances, image, visual_transform):
        return self.occluder.apply(poses, origin, rays, distances, image,
                                   visual_transform,
                                   near_by_link={'base_link': .030} if self.camera == 'head' else None,
                                   excluded_links=(f'{self.camera}_camera_link',))


class CameraView:
    """Pure geometry adapter for the shared A1Z renderer (no ROS node)."""
    make_rays = core.Preview.make_rays
    link_poses = core.Preview.link_poses
    visual_transform = staticmethod(core.Preview.visual_transform)
    render = core.Preview.render
    markers = core.Preview.markers

    def __init__(self, camera, scale, robot, occluder):
        self.frame = f'{camera}_camera_optical_frame'
        calibration = yaml.safe_load((PKG/f'config/{camera}_camera_info.yaml').read_text())
        self.width = calibration['image_width']//scale
        self.height = calibration['image_height']//scale
        self.K = np.array(calibration['camera_matrix']['data']).reshape(3, 3)
        self.K[:2] /= scale
        self.D = np.array(calibration['distortion_coefficients']['data'])
        self.robot, self.occluder, self.angles = robot, occluder, {}
        self.rays, self.valid = self.make_rays()
        self.occlusion_mask = np.zeros(len(self.rays), np.uint8)

    def pose(self):
        return self.link_poses()[self.frame]


def make_views(scale=2, self_occlusion=True):
    robot = ET.parse(PKG/'urdf/LEMO.urdf').getroot()
    occluder = MeshOcclusion(PKG, 'urdf/LEMO.urdf', ()) if self_occlusion else None
    views = {camera: CameraView(camera, scale, robot, occluder) for camera in CAMERAS}
    if occluder:
        for camera, view in views.items():
            view.occluder = ViewOcclusion(occluder, camera)
    return views, occluder


class LemoPreview(Node):
    def __init__(self, scale, self_occlusion):
        super().__init__('lemo_fisheye_preview')
        self.views, self.occluder = make_views(scale, self_occlusion)
        self.dirty, self.images = True, {}
        self.publishers_by_camera = {}
        for camera in CAMERAS:
            topic = f'/lemo_camera_preview/{camera}'
            self.publishers_by_camera[camera] = (
                self.create_publisher(Image, topic+'/image_raw', 1),
                self.create_publisher(CameraInfo, topic+'/camera_info', 1),
                self.create_publisher(Image, topic+'/self_occlusion_mask', 1))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.create_publisher(MarkerArray, '/lemo_camera_preview/scene', qos)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_timer(.5, self.update)
        self.get_logger().info(f'Three fisheye views, {1920//scale}x{1200//scale}; '
                               f'whole-robot self-occlusion={self_occlusion}')

    def joints(self, msg):
        updated = dict(zip(msg.name, msg.position))
        angles = next(iter(self.views.values())).angles
        if any(abs(angles.get(k, 0)-v) > 1e-6 for k, v in updated.items()):
            self.dirty = True
        for view in self.views.values():
            view.angles.update(updated)

    def update(self):
        # Any arm motion can change ALL views, including the fixed head camera.
        if self.dirty:
            self.images = {camera: view.render(view.pose()) for camera, view in self.views.items()}
            self.dirty = False
        stamp = self.get_clock().now().to_msg()
        scene = None
        for camera, view in self.views.items():
            image = Image()
            image.header.stamp, image.header.frame_id = stamp, view.frame
            image.height, image.width = view.height, view.width
            image.encoding, image.step = 'rgb8', view.width*3
            image.data = self.images[camera].tobytes()
            info = CameraInfo()
            info.header = image.header
            info.height, info.width, info.distortion_model = view.height, view.width, 'equidistant'
            info.k, info.d = view.K.flatten().tolist(), view.D.tolist()
            info.r = np.eye(3).flatten().tolist()
            info.p = np.column_stack([view.K, np.zeros(3)]).flatten().tolist()
            mask = Image()
            mask.header = image.header
            mask.height, mask.width = view.height, view.width
            mask.encoding, mask.step = 'mono8', view.width
            mask.data = (view.occlusion_mask*255).tobytes()
            for publisher, message in zip(self.publishers_by_camera[camera], (image, info, mask)):
                publisher.publish(message)
            markers = view.markers(stamp)
            axis = markers.markers[-1]
            axis.ns = f'{camera}_optical_axis'
            if scene is None:
                scene = markers
            else:
                scene.markers.append(axis)
        self.scene_pub.publish(scene)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scale', type=int, choices=(1, 2, 4), default=2)
    parser.add_argument('--no-self-occlusion', action='store_true')
    parser.add_argument('--render-only', type=Path, help='Save three zero-pose images and masks into this directory')
    args = parser.parse_args()
    if args.render_only:
        import cv2
        args.render_only.mkdir(parents=True, exist_ok=True)
        views, occluder = make_views(args.scale, not args.no_self_occlusion)
        try:
            for camera, view in views.items():
                image = view.render(view.pose())
                assert cv2.imwrite(str(args.render_only/f'{camera}.png'), image[:, :, ::-1])
                assert cv2.imwrite(str(args.render_only/f'{camera}-mask.png'),
                                   view.occlusion_mask.reshape(view.height, view.width)*255)
        finally:
            if occluder:
                occluder.close()
        return
    # Keep preview joint states separate from other running ROS applications.
    os.environ.setdefault('ROS_DOMAIN_ID', str(50 + os.getpid()%50))
    print(f'Preview ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}', flush=True)
    children, node = [], None
    with tempfile.TemporaryDirectory(prefix='lemo-camera-preview-') as tmp:
        overlay = Path(tmp)
        # Keep ROS logs in the writable preview workspace as well.
        os.environ['ROS_LOG_DIR'] = str(overlay/'logs')
        marker = overlay/'share/ament_index/resource_index/packages'/NAME
        marker.parent.mkdir(parents=True)
        marker.touch()
        (overlay/'share'/NAME).symlink_to(PKG, target_is_directory=True)
        os.environ['AMENT_PREFIX_PATH'] = str(overlay)+os.pathsep+os.environ.get('AMENT_PREFIX_PATH', '')
        params = overlay/'robot.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'robot_description': (PKG/'urdf/LEMO.urdf').read_text()}}}))
        logfile = Path(tempfile.gettempdir())/'lemo-camera-preview.log'
        print(f'ROS/RViz log: {logfile}', flush=True)
        try:
            with logfile.open('w') as log:
                for package, executable, extra in [
                    ('robot_state_publisher', 'robot_state_publisher', ['--ros-args', '--params-file', str(params)]),
                    ('joint_state_publisher_gui', 'joint_state_publisher_gui', ['--ros-args', '--params-file', str(params)]),
                    ('rviz2', 'rviz2', ['-d', str(PKG/'preview/camera_preview.rviz')])]:
                    children.append(subprocess.Popen(['ros2', 'run', package, executable, *extra],
                                                     stdout=log, stderr=log, start_new_session=True))
                rclpy.init(args=[])
                node = LemoPreview(args.scale, not args.no_self_occlusion)
                while rclpy.ok() and all(p.poll() is None for p in children):
                    rclpy.spin_once(node, timeout_sec=.2)
                failed = [(p.args[2], p.returncode) for p in children if p.poll() not in (None, 0)]
                if failed:
                    raise RuntimeError(f'Preview process failed: {failed}; see {logfile}')
        except KeyboardInterrupt:
            pass
        finally:
            if node:
                if node.occluder:
                    node.occluder.close()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for process in children:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in children:
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=4)


if __name__ == '__main__':
    main()
