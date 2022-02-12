#!/usr/bin/env /usr/bin/python
import sys
import os
import uuid
import cv2
import numpy as np

import io
import pybase64 as base64
import json
import subprocess

from scipy.spatial.transform import Rotation as RotationS
pyrobot_path = os.path.expanduser("~/pyrobot/src")
sys.path.append(pyrobot_path)
from pyrobot.core import Robot
from pyrobot import util

import pcl
import rospy
import std_msgs.msg
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import SharedArray as shm_array
from pykdtree.kdtree import KDTree
import tf

import socket
import time

_OCTO_CACHE = os.path.split(os.path.abspath(__file__))[0] + '/octo_cache.npy'

robot_name = sys.argv[1]
camera_frame = sys.argv[2]

bot = Robot(robot_name)
bot.arm.set_max_velocity_scaling_factor(1.0)
bot.arm.set_max_acceleration_scaling_factor(0.75)

octomap_pc_topic = "/my_point_cloud_for_octomap"
retries = 3
DEPTH_MEDIAN = 0

_CAM_INTRINSICS, _, _ = bot.camera.get_intrinsics()
_CAM_INTRINSICS_inv = np.linalg.inv(_CAM_INTRINSICS) # (3,3)
listener = tf.TransformListener()

if "tm700" in robot_name:
    # Use modbus
    import serial
    USB_PORT = '/dev/ttyUSB0'
    class Gripper:
        def __init__(self, usb_port):
            self.connect = serial.Serial(
                port=usb_port,
                baudrate=19200,
                parity=serial.PARITY_NONE,
                bytesize=8,
                stopbits=1,
                timeout=0.05,
                write_timeout=2.0,
            )
            self.origin()

        def __del__(self):
            self.connect.close()

        def close(self):
            self._write_word('010620400000', self.connect)  # clear position
            self._write_word('010620400028', self.connect)  # run position 2

        def open(self):
            self._write_word('010620400000', self.connect)  # clear position
            self._write_word('010620400018', self.connect)  # run position 1

        def close_abs(self):
            self._write_word('010620140064', self.connect)   # \u901f\u5ea6
            self._write_word('0110200200020400001A90', self.connect)  # \u4f4d\u7f6e
            self._write_word('0106201E0001', self.connect)   # \u555f\u52d5

        def open_abs(self):
            self._write_word('010620140064', self.connect)   # \u901f\u5ea6
            self._write_word('0110200200020400001A90', self.connect)  # \u4f4d\u7f6e
            self._write_word('0106201E0001', self.connect)   # \u555f\u52d5

        def origin(self):
            self._write_word('0106201E0003', self.connect)  # run origin

        def _write_word(self, code, connect):
            LRC = 255
            for i in range(len(code) // 2):
                s = code[i*2:i*2+2]
                s = int(s, 16)
                LRC -= s
            LRC += 1
            lrc_code = str(hex(LRC))[2:].upper()

            code = ':' + code + lrc_code + '\r\n'
            ret = connect.write(code.encode('ascii'))
            time.sleep(0.5)
            out = connect.read(1000)
            return out
    gripper_client = Gripper(USB_PORT)
    def open_gripper():
        gripper_client.open()
    def close_gripper(depth_compensation=False):
        gripper_client.close()
else:
    HOST = "172.16.11.3"    # The remote host
    PORT = 29999
    VAL_PORT = 30001



    def execute_urp(program_name, sleep=0.1):
        error = 1
        for _ in xrange(retries):
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            s.recv(128)
            s.send(('load /programs/%s.urp\n'%program_name).encode())
            s.recv(128)
            s.send('play\n'.encode())
            ret = s.recv(128)
            s.close()
            time.sleep(sleep)
            if not ret.startswith(b'Fail'):
                error = 0
                break
        return error

    def start_remote_ROS_control():
        return execute_urp('remoteROS', sleep=0.33333)

    def open_gripper():
        execute_urp('open', sleep=0.9)
        start_remote_ROS_control()

    def close_gripper(depth_compensation=False):
        if depth_compensation:
            execute_urp('close_d', sleep=0.8)
        else:
            execute_urp('close', sleep=0.8)
        start_remote_ROS_control()

def get_xyz_transform(listener, target_frame, source_frame):
    '''
    listener: tf listener
    target_frame: base
    source_frame: camera
    ---
    tuple: (trans, quat)
    -- trans: tranlation matrix (3,), tx,ty,tz
    -- quat: quaternion (4,) x,y,z,w
    '''
    try:
        now = rospy.Time.now()
        listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(10))
        trans, quat = listener.lookupTransform(target_frame, source_frame, now)
    except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            raise RuntimeError('Cannot fetch the transform from {0:s} to {1:s}'.format(source_frame, target_frame))
    return trans, quat

def transform_pts(pts, trans, quat):
    '''
    pts: (N,3) -- x,y,z
    trans: (3,) -- translation matrix
    quat: (4,) -- quanternion (rotation)
    ---
    rpts: (N,3) -- transformed x,y,z
    '''
    R = RotationS.from_quat(quat)
    return R.apply(pts) + np.asarray(trans,dtype=pts.dtype)[np.newaxis]

def transform_pts_wrt_base_link(pts, listener, target_frame, source_frame, return_transform=False):
    trans, quat = get_xyz_transform(listener, target_frame, source_frame)
    return transform_pts(pts, trans, quat) if not return_transform else (transform_pts(pts, trans, quat), trans, quat)

def get_xyz(bot):
    '''
    bot: Robot tm700
    ---
    rpts: ndarray (H, W, 3) -- x, y, z
    '''
    if DEPTH_MEDIAN > 1:
        depth = np.median([bot.camera.get_depth() for _ in xrange(DEPTH_MEDIAN)], axis=0)
    else:
        depth = bot.camera.get_depth()
    depth = depth / 1e3
    y = np.arange(0, depth.shape[0])
    x = np.arange(0, depth.shape[1])
    xv, yv = np.meshgrid(x, y)
    xy = np.append(xv[np.newaxis], yv[np.newaxis], axis=0) # (2, H, W) -- xy
    xy_homogeneous = np.pad(xy, ((0,1),(0,0),(0,0)), mode='constant', constant_values=1) # (3, H, W) -- x, y, 1
    xy_homogeneous_shape = xy_homogeneous.shape
    xy_h_flat = xy_homogeneous.reshape(3, -1) # (3, H*W) -- x, y, 1
    xy_h_flat_t = np.dot(_CAM_INTRINSICS_inv, xy_h_flat) # (3,3) x (3, H*W) -> (3, H*W)
    xy_homogeneous_t = xy_h_flat_t.reshape(xy_homogeneous_shape) # (3, H, W)
    xyz = np.transpose(xy_homogeneous_t*depth[np.newaxis], (1,2,0)) # (H, W, 3) -- x, y, z
    return xyz

def get_transformed_xyz(bot, listener, target_frame='base_link'):
    xyz = get_xyz(bot) # (H, W, 3)
    xyz_shape = xyz.shape

    xyz_f = xyz.reshape(-1, 3) # (H*W, 3)
    xyz_t = transform_pts_wrt_base_link(xyz_f, listener, target_frame=target_frame, source_frame=camera_frame)
    xyz_t = xyz_t.reshape(xyz_shape)

    return xyz_t

def mtx2quat(rot_mtx):
    '''
    rot_mtx: A 3x3 rotation matrix
    ---
    Return: quat
    '''
    rot_4x4 = np.pad(rot_mtx[:3,:3], ((0,1),(0,1)), mode='constant', constant_values=0.0)
    rot_4x4[3,3] = 1.0
    quat = tf.transformations.quaternion_from_matrix(rot_4x4).astype(np.float64)
    return quat



def reset_octomap():
 rospy.wait_for_service('/clear_octomap')
 try:
     collision_map_prox = rospy.ServiceProxy('/clear_octomap', Empty())
     resp = collision_map_prox()
 except rospy.ServiceException, e:
     return False
 return True

def get_inlier_indices(cloud, n_neighbors=10, radius=0.01):
    tree = KDTree(cloud)
    dist, idx = tree.query(cloud, k=n_neighbors)
    inliers = np.all(dist<radius, axis=1)
    return inliers

class OctomapHandler(object):
    def __init__(self, pc_topic = octomap_pc_topic, **kwargs):
        self.octomap_pub = rospy.Publisher(pc_topic, PointCloud2, queue_size=10)
        self.empty = np.array([[-20, -20, -20]])
        self.occupation = np.array([[-20, -20, -20]])
        self.vox_size = 0.0025

    def save(self):
        np.save(_OCTO_CACHE, self.occupation)

    def load(self):
        if os.path.exists(_OCTO_CACHE):
            try:
                self.occupation = np.load(_OCTO_CACHE)
            except Exception as e:
                return str(e)
            return _OCTO_CACHE
        return "Not exists"

    def update(self):
        pc = get_transformed_xyz(bot, listener).reshape(-1, 3).astype(np.float32)
        pc = pc[np.linalg.norm(pc, axis=1, ord=2) < 1.2]
        pc = pcl.PointCloud(pc)
        sor = pc.make_voxel_grid_filter()
        sor.set_leaf_size(self.vox_size, self.vox_size, self.vox_size)
        pc = np.array(sor.filter(), dtype=np.float32).reshape(-1, 3)
        inliers = get_inlier_indices(pc)
        pc = pc[inliers]

        self.occupation = pc
        del sor, inliers

    def add_wall(self, y, w=1.2, h=1.2):
        x = np.linspace(-w/2.0, w/2.0, 100)
        z = np.linspace(-h/2.0, h/2.0, 100)
        xgrid, zgrid = np.meshgrid(x, z)
        ygrid = np.full_like(xgrid, y)
        xyz = np.stack((xgrid, ygrid, zgrid)).reshape(3, -1).T # (3, 100, 100) -> (3, 10000) -> (10000, 3)
        self.occupation = np.append(self.occupation, xyz, axis=0)

    def _apply(self, cell):
        for _ in xrange(2):
            reset_octomap()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        msg = pcl2.create_cloud_xyz32(header, cell)
        self.octomap_pub.publish(msg)

    def enable(self):
        self._apply(self.occupation)

    def disable(self):
        self._apply(self.empty)

def encode_numpy(arr):
    with io.BytesIO() as fp:
        np.save(fp, arr, allow_pickle=False, fix_imports=False)
        return base64.b64encode(fp.getvalue())

def decode_numpy(s):
    return np.load(io.BytesIO(base64.b64decode(s, validate=True)), allow_pickle=False, fix_imports=False)

def write_shm(arr, shm):
    shm[...] = arr[...]

def read_shm(s):
    shm = shm_array.attach(s)
    return np.copy(shm)

def encode_jpgb64(im):
    (flag, im_encode) = cv2.imencode(".jpg", im[...,::-1]) # RGB2BGR for storing
    im_bytes = im_encode.tobytes()
    im_b64 = base64.b64encode(im_bytes)
    return im_b64

def decode_jpgb64(data_b64):
    # BGR2RGB
    return cv2.imdecode(np.fromstring(base64.b64decode(data_b64, validate=True), dtype=np.uint8), cv2.IMREAD_COLOR)[...,:3][...,::-1]

client_trigger_word = "tr@nsfer data 0023: "
def msg(s):
    sys.stdout.write(client_trigger_word+s+'\n') # python 2.7 print in bytes
    sys.stdout.flush()

if __name__ == '__main__':

    # Additional plugins
    octomap_handler = OctomapHandler()

    # Create SHM for IPC
    shm_uuid = str(uuid.uuid4())
    rgb = bot.camera.get_rgb().astype(np.uint8)
    rgb_shm_path = "shm://cml_grasping_rgb-" + shm_uuid
    try:
        shm_array.delete(rgb_shm_path)
    except OSError:
        pass
    shm_rgb = shm_array.create(rgb_shm_path, rgb.shape, dtype=rgb.dtype)
    del rgb
    depth = bot.camera.get_depth().astype(np.float32)
    depth_shm_path = "shm://cml_grasping_depth-" + shm_uuid
    try:
        shm_array.delete(depth_shm_path)
    except OSError:
        pass
    shm_depth = shm_array.create(depth_shm_path, depth.shape, dtype=depth.dtype)
    del depth
    xyz = get_transformed_xyz(bot, listener).astype(np.float32)
    #xyz = np.array([0.0]) # DELETEME
    xyz_shm_path = "shm://cml_grasping_xyz-" + shm_uuid
    try:
        shm_array.delete(xyz_shm_path)
    except OSError:
        pass
    shm_xyz = shm_array.create(xyz_shm_path, xyz.shape, dtype=xyz.dtype)
    del xyz

    while True:
        try:
            line = sys.stdin.readline()[:-1]  # remove \n
            if len(line) == 0:
                break
            if line.startswith("get_image"):
                img = bot.camera.get_rgb().astype(np.uint8)
                write_shm(img, shm_rgb)
                msg(rgb_shm_path)
                del img
            elif line.startswith("get_depth"):
                if DEPTH_MEDIAN > 1:
                    depth = np.median([bot.camera.get_depth() for _ in xrange(DEPTH_MEDIAN)], axis=0)
                else:
                    depth = bot.camera.get_depth()
                depth = depth.astype(np.float32)
                write_shm(depth, shm_depth)
                msg(depth_shm_path)
                del depth
            elif line.startswith("get_jpg"):
                img = bot.camera.get_rgb().astype(np.uint8)
                b64 = encode_jpgb64(img)
                msg(b64)
                del img, b64
            elif line.startswith("get_xyz_rgb"):
                params = line.split()
                xyz = None
                rgb = None
                prefetch = False
                if len(params)>0 and (params[0][-8:]=="prefetch"):
                    prefetch = True
                if len(params)==2: # get_xyz_rgb target_frame
                    if prefetch:
                        msg(xyz_shm_path + " " + rgb_shm_path)
                    xyz = get_transformed_xyz(bot, listener, target_frame=params[1]).astype(np.float32)
                    rgb = bot.camera.get_rgb().astype(np.uint8)
                    write_shm(xyz, shm_xyz)
                    write_shm(rgb, shm_rgb)
                    if not prefetch:
                        msg(xyz_shm_path + " " + rgb_shm_path)
                elif len(params)==1: # get_xyz_rgb
                    if prefetch:
                        msg(xyz_shm_path + " " + rgb_shm_path)
                    xyz = get_transformed_xyz(bot, listener).astype(np.float32)
                    rgb = bot.camera.get_rgb().astype(np.uint8)
                    write_shm(xyz, shm_xyz)
                    write_shm(rgb, shm_rgb)
                    if not prefetch:
                        msg(xyz_shm_path + " " + rgb_shm_path)
                else: # Error
                    msg("Error: Wrong arguments in get_xyz_rgb!")
                del params, xyz, rgb, s_xyz, s_rgb
            elif line.startswith("get_xyz"):
                params = line.split()
                xyz = None
                s = None
                prefetch = False
                if len(params)>0 and (params[0][-8:]=="prefetch"):
                    prefetch = True
                if len(params)==2: # get_xyz target_frame
                    if prefetch:
                        msg(xyz_shm_path)
                    xyz = get_transformed_xyz(bot, listener, target_frame=params[1]).astype(np.float32)
                    write_shm(xyz, shm_xyz)
                    if not prefetch:
                        msg(xyz_shm_path)
                elif len(params)==1: # get_xyz
                    if prefetch:
                        msg(xyz_shm_path)
                    xyz = get_transformed_xyz(bot, listener).astype(np.float32)
                    write_shm(xyz, shm_xyz)
                    if not prefetch:
                        msg(xyz_shm_path)
                else: # Error
                    msg("Error: Wrong arguments in get_xyz!")
                del params, xyz, s
            elif line.startswith("get_ee_pose"):
                trans, rot_mat = bot.arm.get_ee_pose("base_link")[:2]
                pose = np.append(rot_mat, trans, axis=1).astype(np.float64)
                b64 = encode_numpy(pose)
                msg(b64)
                del trans, rot_mat, pose, b64
            elif line.startswith("compute_ik"):
                params = line.split()
                trans = None
                quat = None
                qinit = None
                joint_pos = None
                b64 = None
                if len(params)==3: # compute_ik trans rot_mtx
                    trans = decode_numpy(params[1]).reshape(3, 1).astype(np.float64)
                    quat  = mtx2quat(decode_numpy(params[2])) # float64
                    joint_pos = bot.arm.compute_ik(trans, quat, numerical=True)
                    if joint_pos is None: # No solution
                        joint_pos = np.array([], dtype=np.float64)
                    b64 = encode_numpy(joint_pos)
                    msg(b64)
                elif len(params)==4: # compute_ik trans rot_mtx qinit
                    trans = decode_numpy(params[1]).reshape(3, 1).astype(np.float64)
                    quat  = mtx2quat(decode_numpy(params[2])) # float64
                    qinit = decode_numpy(params[3]).astype(np.float64)
                    joint_pos = bot.arm.compute_ik(trans, quat, numerical=True, qinit=qinit)
                    if joint_pos is None: # No solution
                        joint_pos = np.array([], dtype=np.float64)
                    b64 = encode_numpy(joint_pos)
                    msg(b64)
                else: # error
                    msg("Error: Wrong arguments in compute_ik!")
                del params, trans, quat, qinit, joint_pos, b64
            elif line.startswith("get_joint_angles"):
                b64 = encode_numpy(bot.arm.get_joint_angles())
                msg(b64)
                del b64
            elif line.startswith("set_joint_positions_nowait"):
                params = line.split()
                joint_pos = None
                success = False
                if len(params)==2: # set_joint_positions joint_pos
                    msg("success")
                    joint_pos = decode_numpy(params[1]).astype(np.float64)
                    bot.arm.set_joint_positions(joint_pos)
                else:
                    msg("Error: Wrong arguments in set_joint_positions")
                del params, joint_pos, success
            elif line.startswith("set_joint_positions_noplan"):
                params = line.split()
                joint_pos = None
                success = False
                if len(params)==2: # set_joint_positions joint_pos
                    msg("success")
                    joint_pos = decode_numpy(params[1]).astype(np.float64)
                    bot.arm.set_joint_positions(joint_pos, plan=False)
                else:
                    msg("Error: Wrong arguments in set_joint_positions")
                del params, joint_pos, success
            elif line.startswith("set_joint_positions"):
                params = line.split()
                joint_pos = None
                success = False
                if len(params)==2: # set_joint_positions joint_pos
                    joint_pos = decode_numpy(params[1]).astype(np.float64)
                    try:
                        success = bot.arm.set_joint_positions(joint_pos)
                    except:
                        success = False
                    if success:
                        msg("success")
                    else:
                        msg("fail")
                else:
                    msg("Error: Wrong arguments in set_joint_positions")
                del params, joint_pos, success
            elif line.startswith("update_octomap"):
                msg("Updating octomap")
                octomap_handler.update()
            elif line.startswith("load_octomap"):
                msg(octomap_handler.load())
            elif line.startswith("save_octomap"):
                octomap_handler.save()
                msg("Octomap saved")
            elif line.startswith("enable_octomap"):
                octomap_handler.enable()
                msg("Enabled octomap")
            elif line.startswith("disable_octomap"):
                octomap_handler.disable()
                msg("Disabled octomap")
            elif line.startswith("add_wall"):
                params = line.split()
                if len(params)==2:
                    y = float(params[1])
                    octomap_handler.add_wall(y)
                    msg("Added wall")
                    del y
                else:
                    msg("Error: Wrong arguments in add_wall")
                del params
            elif line.startswith("open_gripper"):
                msg("Opening gripper")
                open_gripper()
            elif line.startswith("close_gripper_d"):
                msg("Closing gripper (depth_compensation on)")
                close_gripper(depth_compensation=True)
            elif line.startswith("close_gripper"):
                msg("Closing gripper")
                close_gripper()
            else:
                msg("Error: Not implemented / Wrong arguments.")
        except:
            break
    try:
        shm_array.delete(rgb_shm_path)
    except OSError:
        pass
    try:
        shm_array.delete(depth_shm_path)
    except OSError:
        pass
    try:
        shm_array.delete(xyz_shm_path)
    except OSError:
        pass
