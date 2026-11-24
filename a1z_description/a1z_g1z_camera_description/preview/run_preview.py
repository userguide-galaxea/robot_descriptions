#!/usr/bin/env python3
"""ROS 2/RViz preview with a synthetic analytic fisheye test scene.

No hardware commands are sent. This is an optical/kinematic preview, not a
physics simulator. Robot and finger self-occlusion uses the actual visual STLs.
"""
from pathlib import Path
import argparse
import math
import os
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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

PKG = Path(__file__).resolve().parents[1]
NAME = 'a1z_g1z_camera_description'
# Test scene in base_link: lower corner, upper corner, RGB.
BOXES = [([.43, -.20, 0], [.53, -.10, .10], [220, 70, 55]),
         ([.60, .05, 0], [.72, .17, .16], [50, 150, 225]),
         ([.30, .10, 0], [.37, .17, .07], [230, 180, 45])]


def transform(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    out = np.eye(4)
    out[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                   [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                   [-sp, cp*sr, cp*cr]]
    out[:3, 3] = xyz
    return out


def load_chain():
    robot = ET.parse(PKG / 'urdf/A1Z_G1Z_CAMERA.urdf').getroot()
    parents = {j.find('child').get('link'): j for j in robot.findall('joint')}
    chain, child = [], 'camera_optical_frame'
    while child != 'base_link':
        j = parents[child]
        o = j.find('origin')
        fixed = transform(list(map(float, o.get('xyz', '0 0 0').split())),
                          list(map(float, o.get('rpy', '0 0 0').split())))
        axis = j.find('axis')
        chain.append((j.get('name'), j.get('type'), fixed,
                      np.array(list(map(float, axis.get('xyz').split()))) if axis is not None else None))
        child = j.find('parent').get('link')
    return list(reversed(chain))


class Preview(Node):
    def __init__(self, scale, self_occlusion=True):
        super().__init__('a1z_fisheye_preview')
        calibration = yaml.safe_load((PKG / 'config/camera_info.yaml').read_text())
        self.width, self.height = calibration['image_width'] // scale, calibration['image_height'] // scale
        self.K = np.array(calibration['camera_matrix']['data']).reshape(3, 3)
        self.K[:2] /= scale
        self.D = np.array(calibration['distortion_coefficients']['data'])
        self.chain = load_chain()
        self.robot = ET.parse(PKG / 'urdf/A1Z_G1Z_CAMERA.urdf').getroot()
        self.angles = {}
        self.dirty = True
        self.previous_image = None
        self.rays, self.valid = self.make_rays()
        self.occluder = None
        if self_occlusion:
            from mesh_occlusion import MeshOcclusion
            self.occluder = MeshOcclusion(PKG)
        self.occlusion_mask = np.zeros(len(self.rays), dtype=np.uint8)
        self.image_pub = self.create_publisher(Image, '/camera_preview/image_raw', 1)
        self.mask_pub = self.create_publisher(Image, '/camera_preview/self_occlusion_mask', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_preview/camera_info', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.create_publisher(MarkerArray, '/camera_preview/scene', qos)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_timer(.5, self.update)
        self.get_logger().info(f'Fisheye preview {self.width}x{self.height}; equidistant K/D; robot self-occlusion={self_occlusion}.')
        self.get_logger().info('Manufacturer diagonal FOV limits rays to 110 deg from +Z; this is not a calibrated valid-pixel mask.')

    def make_rays(self):
        v, u = np.mgrid[:self.height, :self.width]
        x = (u-self.K[0, 2])/self.K[0, 0]
        y = (v-self.K[1, 2])/self.K[1, 1]
        rd = np.hypot(x, y)
        # Invert the actual fisheye polynomial with a monotonic lookup table.
        theta = np.linspace(0, math.radians(110), 30000)
        distorted = theta*(1+sum(k*theta**(2*i+2) for i, k in enumerate(self.D)))
        if not np.all(np.diff(distorted) > 0):
            raise ValueError('Nonmonotonic fisheye polynomial within the nominal lens domain')
        t = np.interp(rd, distorted, theta)
        gain = np.sin(t)/np.maximum(rd, 1e-12)
        rays = np.stack([x*gain, y*gain, np.cos(t)], axis=-1)
        return rays.astype(np.float32).reshape(-1, 3), (rd <= distorted[-1]).reshape(-1)

    def joints(self, msg):
        updated = dict(zip(msg.name, msg.position))
        if any(abs(self.angles.get(k, 0)-v) > 1e-6 for k, v in updated.items()):
            self.dirty = True
        self.angles.update(updated)

    def pose(self):
        T = np.eye(4)
        for name, kind, fixed, axis in self.chain:
            T = T @ fixed
            if kind in ('revolute', 'continuous'):
                a = axis/np.linalg.norm(axis)
                q = self.angles.get(name, 0)
                skew = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                motion = np.eye(4)
                motion[:3, :3] = np.eye(3)+math.sin(q)*skew+(1-math.cos(q))*(skew@skew)
                T = T @ motion
        return T

    @staticmethod
    def visual_transform(visual):
        o = visual.find('origin')
        if o is None:
            return np.eye(4)
        return transform(list(map(float, o.get('xyz', '0 0 0').split())),
                         list(map(float, o.get('rpy', '0 0 0').split())))

    def link_poses(self):
        poses = {'base_link': np.eye(4)}
        pending = list(self.robot.findall('joint'))
        while pending:
            progress = False
            for joint in pending[:]:
                parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                if parent not in poses:
                    continue
                o = joint.find('origin')
                T = transform(list(map(float, o.get('xyz', '0 0 0').split())),
                              list(map(float, o.get('rpy', '0 0 0').split())))
                mimic = joint.find('mimic')
                q = self.angles.get(joint.get('name'), 0.)
                if mimic is not None:
                    q = (self.angles.get(mimic.get('joint'), 0.)*float(mimic.get('multiplier', '1'))
                         + float(mimic.get('offset', '0')))
                motion = np.eye(4)
                if joint.get('type') != 'fixed':
                    axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
                    axis /= np.linalg.norm(axis)
                    if joint.get('type') == 'prismatic':
                        motion[:3, 3] = axis*q
                    else:
                        a = axis
                        skew = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                        motion[:3, :3] = np.eye(3)+math.sin(q)*skew+(1-math.cos(q))*(skew@skew)
                poses[child] = poses[parent] @ T @ motion
                pending.remove(joint)
                progress = True
            if not progress:
                raise ValueError('Disconnected URDF joint tree')
        return poses

    def render(self, T):
        # Rays start in optical (+X right, +Y down, +Z forward), then enter base.
        origin = T[:3, 3].astype(np.float32)
        rays = self.rays @ T[:3, :3].astype(np.float32).T
        count = len(rays)
        distance = np.full(count, np.inf, dtype=np.float32)
        image = np.tile(np.array([125, 155, 190], dtype=np.uint8), (count, 1))
        # Ground plane z=0, 10 cm checker pattern, matching RViz markers.
        good = np.abs(rays[:, 2]) > 1e-8
        t = np.full(count, np.inf, dtype=np.float32)
        np.divide(-origin[2], rays[:, 2], out=t, where=good)
        hit = good & (t > 0) & (t < 10)
        where = np.flatnonzero(hit)
        points = origin + rays[where]*t[where, None]
        checker = (np.floor(points[:, 0]/.1)+np.floor(points[:, 1]/.1)).astype(int) % 2
        image[where] = np.where(checker[:, None] == 0, [205, 205, 205], [110, 110, 110])
        distance[hit] = t[hit]
        # A patterned target wall x=1 m.
        good = np.abs(rays[:, 0]) > 1e-8
        t = np.full(count, np.inf, dtype=np.float32)
        np.divide(1-origin[0], rays[:, 0], out=t, where=good)
        idx = np.flatnonzero(good & (t > 0) & (t < distance))
        pos = origin + rays[idx]*t[idx, None]
        ok = (np.abs(pos[:, 1]) < .6) & (pos[:, 2] >= 0) & (pos[:, 2] < .8)
        idx, pos = idx[ok], pos[ok]
        checker = (np.floor(pos[:, 1]/.1)+np.floor(pos[:, 2]/.1)).astype(int) % 2
        image[idx] = np.where(checker[:, None] == 0, [230, 230, 230], [65, 65, 65])
        distance[idx] = t[idx]
        for low, high, color in BOXES:
            lower, upper = np.array(low), np.array(high)
            near = np.full(count, -np.inf, dtype=np.float32)
            far = np.full(count, np.inf, dtype=np.float32)
            normal = np.zeros(count, dtype=np.int8)
            for a in range(3):
                parallel = np.abs(rays[:, a]) < 1e-8
                inv = np.zeros(count, dtype=np.float32)
                np.divide(1, rays[:, a], out=inv, where=~parallel)
                t1, t2 = (lower[a]-origin[a])*inv, (upper[a]-origin[a])*inv
                enter, leave = np.minimum(t1, t2), np.maximum(t1, t2)
                enter[parallel] = -np.inf
                leave[parallel] = np.inf if lower[a] <= origin[a] <= upper[a] else -np.inf
                normal[enter > near] = a
                near, far = np.maximum(near, enter), np.minimum(far, leave)
            t = np.where(near > 1e-5, near, far)
            hit = (far >= near) & (t > 1e-5) & (t < distance)
            shade = np.array([.80, .65, 1.0])[normal[hit]]
            image[hit] = (np.array(color)*shade[:, None]).astype(np.uint8)
            distance[hit] = t[hit]
        if self.occluder is not None:
            self.occlusion_mask = self.occluder.apply(
                self.link_poses(), origin, rays, distance, image, self.visual_transform)
            self.occlusion_mask[~self.valid] = 0
        image[~self.valid] = [25, 25, 25]
        return image.reshape(self.height, self.width, 3)

    def markers(self, stamp):
        out = MarkerArray()
        def cube(pos, size, color):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = 'test_scene', len(out.markers), Marker.CUBE, Marker.ADD
            m.pose.orientation.w = 1.
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
            m.scale.x, m.scale.y, m.scale.z = map(float, size)
            m.color.r, m.color.g, m.color.b = [c/255 for c in color]
            m.color.a = 1.
            out.markers.append(m)
        for low, high, color in BOXES:
            cube((np.array(low)+high)/2, np.array(high)-low, color)
        for x in range(-6, 16):
            for y in range(-10, 10):
                c = 205 if (x+y) % 2 == 0 else 110
                cube([x*.1+.05, y*.1+.05, -.002], [.1, .1, .004], [c]*3)
        for y in range(-6, 6):
            for z in range(8):
                c = 230 if (y+z) % 2 == 0 else 65
                cube([1., y*.1+.05, z*.1+.05], [.002, .1, .1], [c]*3)
        ray = Marker()
        # Express this marker in base to avoid a transient TF timing error
        # when a fresh image precedes the corresponding joint TF update.
        ray.header.frame_id = 'base_link'
        ray.header.stamp = stamp
        ray.ns, ray.id, ray.type = 'optical_axis', 0, Marker.ARROW
        ray.pose.orientation.w = 1.
        ray.scale.x, ray.scale.y, ray.scale.z = .003, .008, .015
        ray.color.b, ray.color.a = 1., 1.
        T = self.pose()
        start, end = T[:3, 3], T[:3, 3] + .4*T[:3, 2]
        ray.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (start, end)]
        out.markers.append(ray)
        return out

    def update(self):
        if self.dirty or self.previous_image is None:
            self.previous_image = self.render(self.pose())
            self.dirty = False
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, 'camera_optical_frame'
        msg.height, msg.width, msg.encoding, msg.step = self.height, self.width, 'rgb8', self.width*3
        msg.data = self.previous_image.tobytes()
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width, info.distortion_model = self.height, self.width, 'equidistant'
        info.k, info.d = self.K.flatten().tolist(), self.D.tolist()
        info.r = np.eye(3).flatten().tolist()
        info.p = np.column_stack([self.K, np.zeros(3)]).flatten().tolist()
        self.image_pub.publish(msg)
        mask = Image()
        mask.header = msg.header
        mask.height, mask.width, mask.encoding, mask.step = self.height, self.width, 'mono8', self.width
        mask.data = (self.occlusion_mask*255).tobytes()
        self.mask_pub.publish(mask)
        self.info_pub.publish(info)
        self.scene_pub.publish(self.markers(stamp))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scale', type=int, choices=[1, 2, 4], default=2,
                        help='Preview downsample factor; K scales with output (default 960x600)')
    parser.add_argument('--render-only', type=Path, help='Save one zero-pose image without launching RViz')
    parser.add_argument('--no-self-occlusion', action='store_true', help='Use the analytic scene without robot occlusion for comparison')
    args = parser.parse_args()
    if args.render_only:
        import cv2
        rclpy.init(args=[])
        node = Preview(args.scale, not args.no_self_occlusion)
        image = node.render(node.pose())
        if not cv2.imwrite(str(args.render_only), image[:, :, ::-1]):
            raise RuntimeError('Cannot save preview image')
        node.destroy_node()
        rclpy.shutdown()
        return
    children = []
    with tempfile.TemporaryDirectory(prefix='a1z-camera-preview-') as tmp:
        overlay = Path(tmp)
        marker = overlay/'share/ament_index/resource_index/packages'/NAME
        marker.parent.mkdir(parents=True)
        marker.touch()
        (overlay/'share'/NAME).symlink_to(PKG, target_is_directory=True)
        os.environ['AMENT_PREFIX_PATH'] = str(overlay)+os.pathsep+os.environ.get('AMENT_PREFIX_PATH', '')
        params = overlay/'robot.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'robot_description': (PKG/'urdf/A1Z_G1Z_CAMERA.urdf').read_text()}}}))
        logfile = Path(tempfile.gettempdir())/'a1z-camera-preview.log'
        print(f'ROS/RViz log: {logfile}', flush=True)
        try:
            with logfile.open('w') as log:
                for package, executable, extra in [
                    ('robot_state_publisher', 'robot_state_publisher', ['--ros-args', '--params-file', str(params)]),
                    ('joint_state_publisher_gui', 'joint_state_publisher_gui', ['--ros-args', '--params-file', str(params)]),
                    ('rviz2', 'rviz2', ['-d', str(PKG/'preview/camera_preview.rviz')])]:
                    children.append(subprocess.Popen(['ros2', 'run', package, executable, *extra], stdout=log, stderr=log, start_new_session=True))
                rclpy.init(args=[])
                node = Preview(args.scale, not args.no_self_occlusion)
                while rclpy.ok() and all(p.poll() is None for p in children):
                    rclpy.spin_once(node, timeout_sec=.2)
                failed = [(p.args[2], p.returncode) for p in children if p.poll() not in (None, 0)]
                if failed:
                    raise RuntimeError(f'Preview process failed: {failed}; see {logfile}')
        except KeyboardInterrupt:
            pass
        finally:
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


if __name__ == '__main__':
    main()
