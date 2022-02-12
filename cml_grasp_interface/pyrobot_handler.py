import sys
import os
import numpy as np
import cv2
import pcl
import io
import pybase64 as base64
import json
import subprocess
import SharedArray as shm_array
import time


default_proc_path = os.path.split(os.path.abspath(__file__))[0] + "/bot_env.sh"
tm_proc_path = os.path.split(os.path.abspath(__file__))[0] + "/bot_env_tm.sh"
#query_time_fd = open("query_time.log", "a")


def encode_numpy(arr):
    with io.BytesIO() as fp:
        np.save(fp, arr, allow_pickle=False, fix_imports=False)
        return base64.b64encode(fp.getvalue())


def decode_numpy(s):
    return np.load(io.BytesIO(base64.b64decode(s, validate=True)), allow_pickle=False, fix_imports=False)


def read_shm(s, copy=True):
    if copy:
        return np.copy(shm_array.attach(s.decode()))
    else:
        return shm_array.attach(s.decode())  # Careful

def decode_jpgb64(data_b64):
    # BGR2RGB
    return cv2.imdecode(np.fromstring(base64.b64decode(data_b64, validate=True),
                                      dtype=np.uint8),
                        cv2.IMREAD_COLOR)[..., :3][..., ::-1]


class PyRobotHandler(object):
    def __init__(self, proc=default_proc_path, offline=False, octomap=True, bot="ur3", camera="camera_color_optical_frame"):
        if "tm700" in bot:
            proc = tm_proc_path
        self.camera_frame = camera.encode()
        self.sub_process = subprocess.Popen((proc, bot, camera),
                                            stdout=subprocess.PIPE,
                                            stdin=subprocess.PIPE)
        self.expect = b"tr@nsfer data 0023: "
        self.offline = offline
        self.octomap = octomap

    def query(self, command, timeout=30):
        #_s = time.time()
        assert isinstance(command, bytes)
        self.sub_process.stdin.write(command+b'\n')
        self.sub_process.stdin.flush()
        ts_start = time.time()
        success = False
        while time.time() - ts_start < timeout:
            head = self.sub_process.stdout.read(len(self.expect))
            if head == self.expect:
                success = True
                recv = self.sub_process.stdout.readline()[:-1] # remove \n
                break
            else:
                if len(head)>0 and not head[-1] == '\n':
                    next(self.sub_process.stdout) # skip this line
        if not success:
            raise RuntimeError("Operation timed out!")
        #query_time_fd.write("query time: %.4f (%s)\n"%(time.time()-_s, command[:10].decode()))
        #query_time_fd.flush()
        return recv

    def get_image(self, copy=True):
        recv = self.query(b"get_image")
        return read_shm(recv, copy=copy)

    def get_depth(self, copy=True):
        recv = self.query(b"get_depth")
        return read_shm(recv, copy=copy)

    def get_jpg(self):
        recv = self.query(b"get_jpg")
        return recv  # base64

    def load_prefetch(self, recv, copy=True):
        return read_shm(recv, copy=copy)

    def get_xyz(self, target_frame=None, copy=True, prefetch=False):
        command = b"get_xyz"
        if prefetch:
            command = command + b"_prefetch"
        if not target_frame is None:
            assert isinstance(target_frame, bytes)
            command = command + b" " + target_frame
        recv = self.query(command)
        if prefetch:
            return recv
        else:
            return read_shm(recv, copy=copy)

    def get_xyz_rgb(self, target_frame=None, copy_xyz=True, copy_rgb=True, prefetch=False):
        command = b"get_xyz_rgb"
        if prefetch:
            command = command + b"_prefetch"
        if not target_frame is None:
            assert isinstance(target_frame, bytes)
            command = command + b" " + target_frame
        recv = self.query(command)
        recv_split = recv.split()
        assert len(recv_split) == 2
        s_xyz, s_rgb = recv_split
        if prefetch:
            return s_xyz, s_rgb
        else:
            xyz = read_shm(s_xyz, copy=copy_xyz)
            rgb = read_shm(s_rgb, copy=copy_rgb)
            return xyz, rgb

    def get_ee_pose(self):
        if self.offline:
            return None
        recv = self.query(b"get_ee_pose")
        pose = decode_numpy(recv)
        return pose

    def get_joint_angles(self):
        if self.offline:
            return None
        recv = self.query(b"get_joint_angles")
        joints = decode_numpy(recv)
        if len(joints) != 6:
            return None
        else:
            return joints

    def set_joint_positions(self, joints, nowait=False, noplan=False):
        if self.offline:
            return True
        b64 = encode_numpy(joints)
        if nowait:
            recv = self.query(b"set_joint_positions_nowait " + b64)
        elif noplan:
            recv = self.query(b"set_joint_positions_noplan " + b64)
        else:
            recv = self.query(b"set_joint_positions " + b64)
        if recv.startswith(b"success"):
            return True
        else:
            return False

    def compute_ik(self, trans, rot_mtx, qinit=None):
        trans_b64 = encode_numpy(trans)
        rot_mtx_b64 = encode_numpy(rot_mtx)
        if qinit is None:
            recv = self.query(b"compute_ik " + trans_b64 +
                              b" " + rot_mtx_b64)
        else:
            qinit_b64 = encode_numpy(qinit)
            recv = self.query(b"compute_ik " + trans_b64 +
                              b" " + rot_mtx_b64 +
                              b" " + qinit_b64)
        joints = decode_numpy(recv)
        if len(joints) != 6:
            return None
        else:
            return joints

    def load_octomap(self):
        if self.offline or (not self.octomap):
            return
        recv = self.query(b"load_octomap")

    def save_octomap(self):
        if self.offline or (not self.octomap):
            return
        recv = self.query(b"save_octomap")

    def update_octomap(self):
        if self.offline or (not self.octomap):
            return
        recv = self.query(b"update_octomap")

    def disable_octomap(self):
        if self.offline or (not self.octomap):
            return
        recv = self.query(b"disable_octomap")

    def enable_octomap(self):
        if self.offline or (not self.octomap):
            return
        recv = self.query(b"enable_octomap")

    def add_wall(self, y):
        if self.offline or (not self.octomap):
            return
        recv = self.query(b"add_wall %.4f"%y)

    def get_xyz_transform(self, target, source):
        recv = self.query(b"get_xyz_transform " + target + b" " + source)
        return decode_numpy(recv)

    def open_gripper(self, force=False):
        if self.offline:
            return
        if force:
            command = b"open_gripper_force"
        else:
            command = b"open_gripper"
        recv = self.query(command)

    def close_gripper(self, depth_compensation=False, force=False):
        if self.offline:
            return
        if depth_compensation:
            command = b"close_gripper_d"
        else:
            if force:
                command = b"close_gripper_force"
            else:
                command = b"close_gripper"
        recv = self.query(command)

    def __del__(self):
        self.sub_process.stdin.close()
        self.sub_process.terminate()
