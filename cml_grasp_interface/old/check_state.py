import os
import sys
import shutil
import cv2
import json
import numpy as np
import time
from matplotlib import pyplot as plt

from .pyrobot_handler import PyRobotHandler

_SCRIPT_DIR = os.path.split(os.path.abspath(__file__))[0]

bot = PyRobotHandler()
shot_joint = np.load(_SCRIPT_DIR + '/shot_joint.npy')
init_joint = np.load(_SCRIPT_DIR + '/init_joint.npy')
drop_joint = np.load(_SCRIPT_DIR + '/drop_joint.npy')
bot.set_joint_positions(init_joint)
bot.set_joint_positions(shot_joint)
bot.set_joint_positions(drop_joint)
