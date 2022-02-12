import sys
import os
import cv2
import json
import torch
torch.backends.cudnn.benchmark = True
torch.multiprocessing.set_start_method('forkserver')
from gdn.representation.euler_scene_rp import *
from gdn.detector.pointnet2_s4g.backbone import Pointnet2MSG
from nms import decode_euler_feature
from nms import initEigen, sanity_check
from nms import crop_index, generate_gripper_edge
from scipy.spatial.transform import Rotation
initEigen(0)
#from mayavi import mlab
import numpy as np
import pcl

import io
import base64

from .pyrobot_handler import PyRobotHandler

if __name__ == '__main__':
    h = PyRobotHandler(bot=sys.argv[1], camera=sys.argv[2])
    joint = h.get_joint_angles()
    print(joint)
    path = os.path.split(__file__)[0]+'/'+sys.argv[3]
    dir_ = os.path.split(path)[0]
    if not os.path.exists(dir_):
        os.makedirs(dir_)
    np.save(path, joint, allow_pickle=True, fix_imports=True)

