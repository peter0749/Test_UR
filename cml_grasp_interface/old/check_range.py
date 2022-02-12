import os
import sys
import shutil
import cv2
import json
import numpy as np
import time

from .pyrobot_handler import PyRobotHandler

_SCRIPT_DIR = os.path.split(os.path.abspath(__file__))[0]

bot = PyRobotHandler()
shot_joint = np.load(_SCRIPT_DIR + '/shot_joint.npy')
init_joint = np.load(_SCRIPT_DIR + '/init_joint.npy')
bot.set_joint_positions(init_joint)
bot.open_gripper()
bot.close_gripper()
bot.set_joint_positions(shot_joint)
time.sleep(2)
while True:
    sup_im = bot.get_image()[...,::-1]
    cv2.imshow("whatever", sup_im)
    if cv2.waitKey(1) == ord('q'):
        break
bot.set_joint_positions(init_joint)
bot.open_gripper()
