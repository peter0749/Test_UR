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
    target = "."
    bot_name = "ur3"
    cam_name = "camera_color_optical_frame"
    if len(sys.argv) >= 2:
        target = sys.argv[1]
    if len(sys.argv) >= 3:
        bot_name = sys.argv[2]
    if len(sys.argv) >= 4:
        cam_name = sys.argv[3]
    h = PyRobotHandler(bot=bot_name, camera=cam_name)
    mins = np.full(3,  np.inf, dtype=np.float64)
    maxs = np.full(3, -np.inf, dtype=np.float64)
    while True:
        try:
            pose = h.get_ee_pose()
            mins = np.minimum(mins, pose[:,3])
            maxs = np.maximum(maxs, pose[:,3])
            print(mins, maxs)
        except KeyboardInterrupt:
            break
    np.save(os.path.split(__file__)[0]+"/"+target+"/ee_minmax.npy", np.append(mins, maxs))

