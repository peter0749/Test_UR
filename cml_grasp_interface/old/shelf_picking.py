import open3d
import sys
import shutil
import json
import torch
from gdn.representation.euler_scene_rp import *
from gdn.detector.pointnet2_s4g.backbone import Pointnet2MSG
from nms import initEigen
from nms import crop_index, generate_gripper_edge
initEigen(0)
import numpy as np

from .grasp_detector import GraspHandler, subsample, eprint
from .pyrobot_handler import PyRobotHandler

_VERBOSE = True
_DEBUG = False
_PLOT = False
_SAVE_PLOT = False
_OFFLINE = False
_CONSTRAINT_RANGE = True
_USE_OCTOMAP = False
_STATE_DIR = os.path.split(os.path.abspath(__file__))[0] + '/shelf_states'

plot_cnt = 0
if _PLOT:
    from mayavi import mlab
    figsize = (1280, 1024)
    #figsize = (640, 480)
    fig = mlab.figure(bgcolor=(0,0,0), size=figsize)

def break_down_steps_linear(bot, trans_a, trans_b, rot, initial_joints, steps=5):
    joints_pipeline = [initial_joints,]
    for d in np.linspace(0, 1, steps):
        t = d * trans_b + (1.-d) * trans_a
        last_joints = joints_pipeline[-1]
        next_joints = bot.compute_ik(t, rot, qinit=last_joints)
        if next_joints is None:
            joints_pipeline = []
            break
        joints_pipeline.append(next_joints)
    if len(joints_pipeline) > 0:
        del joints_pipeline[0]
    return joints_pipeline

def exec_joints_pipeline(bot, pipeline):
    result = False
    for joints in pipeline:
        #if not bot.set_joint_positions(joints):
        #    return False
        result = bot.set_joint_positions(joints)
    return result

class GraspHandlerShelf(GraspHandler):
    def __init__(self, bot, config_path, weights_path, state_dir=_STATE_DIR):

        self.approach_step = 0.08
        self.lift_up = 0.15
        self.max_up = 0.80

        with open(config_path, 'r') as fp:
            self.config = json.load(fp)
            self.input_points = self.config['input_points']
            self.gripper_length = self.config['hand_height']
        self.bot = bot
        self.model = Pointnet2MSG(self.config, activation_layer=EulerActivation())
        self.model = self.model.cuda()
        self.model = self.model.eval()
        self.model.load_state_dict(torch.load(weights_path)['base_model'])
        self.representation = EulerRepresentation(self.config)

        self.drop_joint = np.load(state_dir + '/drop_joint.npy')
        self.init_joint = np.load(state_dir + '/init_joint.npy')
        ee_minmax  = np.load(state_dir + '/ee_minmax.npy')
        self.ee_min     = ee_minmax[:3]
        self.ee_max     = ee_minmax[3:]
        del ee_minmax

    def take_from_shelf(self, cloud_base, n_candidate, n_nms, roi_mask=None, weighting=None, max_ik_iter=1000):
        global plot_cnt

        drop_joint = self.drop_joint
        init_joint = self.init_joint
        ee_min = self.ee_min
        ee_max = self.ee_max

        cloud_base = cloud_base.reshape(-1, 3)
        pc_npy, ind = subsample(cloud_base, leaf_size=0.005, voxelize=True)
        if not roi_mask is None:
            roi_mask = roi_mask.reshape(-1)
            roi_mask = roi_mask[ind]
        if not weighting is None:
            weighting = weighting.reshape(-1)
            weighting = weighting[ind]
        pc_npy_max = np.max(pc_npy, axis=0)
        pc_npy_min = np.min(pc_npy, axis=0)
        trans_to_frame = (pc_npy_max + pc_npy_min) / 2.0
        trans_to_frame[2] = np.min(pc_npy[:,2])
        trans_to_frame = trans_to_frame.reshape(1, 3)

        while pc_npy.shape[0]<self.input_points:
            new_ind = np.random.choice(len(pc_npy), self.input_points-len(pc_npy), replace=True)
            new_pts = pc_npy[new_ind,:]
            pc_npy = np.append(pc_npy, new_pts, axis=0)
            if not roi_mask is None:
                new_roi = roi_mask[new_ind]
                roi_mask = np.append(roi_mask, new_roi, axis=0)
            if not weighting is None:
                new_w = weighting[new_ind]
                weighting = np.append(weighting, new_w, axis=0)
        if pc_npy.shape[0]>self.input_points:
            new_ind = np.random.choice(len(pc_npy), self.input_points, replace=False)
            pc_npy = pc_npy[new_ind,:]
            if not roi_mask is None:
                roi_mask = roi_mask[new_ind]
            if not weighting is None:
                weighting = weighting[new_ind]

        eprint("ready")
        pred_poses = self.grasp_proposal(pc_npy-trans_to_frame, n_candidate, n_nms, roi_mask=roi_mask, weighting=weighting)
        if len(pred_poses) == 0:
            eprint("No grasp")
            return False
        pred_poses[:,:,3] += trans_to_frame

        if _CONSTRAINT_RANGE:
            new_poses = []
            for pose in pred_poses:
                approach = pose[:,0:1]
                if np.any(pose[:,3] < ee_min) or np.any(pose[:,3] > ee_max):
                    continue
                if np.arccos(np.dot(approach.reshape(1,3), np.array([0, -1, 0]).reshape(3,1))) > np.radians(50):
                    continue
                if np.arccos(np.dot(approach.reshape(1,3), np.array([0, 0, -1]).reshape(3,1))) > np.radians(75):
                    continue
                new_poses.append(pose)
            del pred_poses
            pred_poses = new_poses

        valid_ik = []
        for pose in pred_poses[:max_ik_iter]:
            trans = pose[:,3].reshape(3, 1).astype(np.float64)
            approach = pose[:,0:1].astype(np.float64)
            trans_back = trans - approach * self.approach_step
            trans_up = np.copy(trans)
            trans_up[2,0] = min(trans_up[2,0] + self.lift_up, self.max_up)
            rot = pose[:3,:3]
            approach_pipeline = break_down_steps_linear(self.bot, trans_back, trans, rot, init_joint if _OFFLINE else None, steps=3)
            if len(approach_pipeline) == 0:
                eprint("No approach")
                del approach_pipeline
                continue
            up_pipeline = break_down_steps_linear(self.bot, trans, trans_up, rot, approach_pipeline[-1], steps=3)
            if len(up_pipeline) == 0:
                eprint("No up")
                del up_pipeline
                continue
            # TODO: Check joint_up -> init_joint -> drop_joint
            valid_ik.append((int((-trans.reshape(-1)[1]) // 0.03), -int(trans.reshape(-1)[2] // 0.01), approach_pipeline, up_pipeline, pose))

        success = False
        if len(valid_ik) > 0:
            valid_ik.sort(key = lambda x: x[0:2])
            for (y, z, approach_pipeline, up_pipeline, pose) in valid_ik:
                if exec_joints_pipeline(self.bot, approach_pipeline):
                    success = True
                    break

        if success:
            # close gripper
            self.bot.close_gripper(depth_compensation=True)

            # up
            # self.bot.set_joint_positions(joint_up)
            exec_joints_pipeline(self.bot, up_pipeline[1:])

            # back
            self.bot.set_joint_positions(init_joint)

            # drop
            self.bot.set_joint_positions(drop_joint)

            # open gripper
            self.bot.open_gripper()

            if _PLOT:
                pc_subset = pc_npy-trans_to_frame
                mlab.view(315, 60, 0.8)
                col = (pc_subset[:,2] - pc_subset[:,2].min()) / (pc_subset[:,2].max() - pc_subset[:,2].min()) + 0.33
                mlab.points3d(pc_subset[:,0], pc_subset[:,1], pc_subset[:,2], col, scale_factor=0.003, mode='sphere', colormap='plasma', opacity=1.0, figure=fig)
                for candidate_ik in valid_ik[:20]:
                    candidate_vis = np.copy(candidate_ik[-1])
                    candidate_vis[:,3] -= trans_to_frame.reshape(3)
                    gripper_inner_edge, gripper_outer1, gripper_outer2 = generate_gripper_edge(self.config['gripper_width'], self.config['hand_height'], candidate_vis, self.config['thickness_side'], 0.0)
                    gripper_l, gripper_r, gripper_l_t, gripper_r_t = gripper_inner_edge

                    mlab.plot3d([gripper_l[0], gripper_r[0]], [gripper_l[1], gripper_r[1]], [gripper_l[2], gripper_r[2]], tube_radius=0.003, color=(0.7,0.7,0.7), opacity=0.3, figure=fig)
                    mlab.plot3d([gripper_l[0], gripper_l_t[0]], [gripper_l[1], gripper_l_t[1]], [gripper_l[2], gripper_l_t[2]], tube_radius=0.003, color=(0.7,0.7,0.7), opacity=0.3,            figure=fig)
                    mlab.plot3d([gripper_r[0], gripper_r_t[0]], [gripper_r[1], gripper_r_t[1]], [gripper_r[2], gripper_r_t[2]], tube_radius=0.003, color=(0.7,0.7,0.7), opacity=0.3,            figure=fig)

                pose_vis = np.copy(pose)
                pose_vis[:,3] -= trans_to_frame.reshape(3)
                gripper_inner_edge, gripper_outer1, gripper_outer2 = generate_gripper_edge(self.config['gripper_width'], self.config['hand_height'], pose_vis, self.config['thickness_side'], 0.0)
                gripper_l, gripper_r, gripper_l_t, gripper_r_t = gripper_inner_edge

                mlab.plot3d([gripper_l[0], gripper_r[0]], [gripper_l[1], gripper_r[1]], [gripper_l[2], gripper_r[2]], tube_radius=0.005, color=(0,1,0), opacity=1.0, figure=fig)
                mlab.plot3d([gripper_l[0], gripper_l_t[0]], [gripper_l[1], gripper_l_t[1]], [gripper_l[2], gripper_l_t[2]], tube_radius=0.005, color=(0,1,0), opacity=1.0,            figure=fig)
                mlab.plot3d([gripper_r[0], gripper_r_t[0]], [gripper_r[1], gripper_r_t[1]], [gripper_r[2], gripper_r_t[2]], tube_radius=0.005, color=(0,1,0), opacity=1.0,            figure=fig)
                if _SAVE_PLOT:
                    plot_cnt += 1
                    mlab.savefig("scene/pc-%d.png"%plot_cnt, size=figsize)
                    shutil.copyfile("scene/pc-%d.png"%plot_cnt, "scene/pc-current.png")
                else:
                    filename = "/tmp/mlab-gdn-plot-%d.png"%os.getpid()
                    os.symlink("/dev/null", filename)
                    mlab.savefig(filename, size=figsize)
                    os.unlink(filename)
                mlab.clf(fig)
                del pc_subset

            return True
        return False

if __name__ == '__main__':
    config_path = sys.argv[1]
    weights_path = sys.argv[2]
    n_candidate = int(sys.argv[3])
    n_nms = int(sys.argv[4])

    x_limit = (-0.45, 0.28)
    y_limit = (-0.70, -0.30)
    z_limit = (0.19, 0.67)
    limit_pad = 0.00
    table_threshold = 0.20

    bot = PyRobotHandler(offline=_OFFLINE, octomap=False, bot="shelf_pick", camera="camera_shelf_color_optical_frame")
    grasper = GraspHandlerShelf(bot, config_path, weights_path)
    bot.set_joint_positions(grasper.init_joint)
    bot.open_gripper()
    nograsp_cnt = 0
    while True:
        if nograsp_cnt >= 3:
            input("Enter to continue...")
        cloud_base = bot.get_xyz(copy=False)
        # bounding box -> image_roi (mask) : (im_height, im_width) : np.boolean
        '''
        mask to crop point cloud
        '''
        crop_x = np.logical_and(cloud_base[...,0]>x_limit[0], cloud_base[...,0]<x_limit[1])
        crop_y = np.logical_and(cloud_base[...,1]>y_limit[0], cloud_base[...,1]<y_limit[1])
        crop_z = np.logical_and(cloud_base[...,2]>z_limit[0], cloud_base[...,2]<z_limit[1])
        crop = np.logical_and(crop_x, np.logical_and(crop_y, crop_z))
        '''
        crop the point cloud
        '''
        cloud_base_c = cloud_base[crop] # (im_height, im_width, 3) -> (???, 3)
        # image_roi = image_roi[crop] # (im_height, im_width, 3) -> (???, 3)

        roi = (cloud_base_c[...,2]>table_threshold) & \
              (cloud_base_c[...,1]>y_limit[0]+limit_pad) & \
              (cloud_base_c[...,1]<y_limit[1]-limit_pad) & \
              (cloud_base_c[...,0]>x_limit[0]+limit_pad) & \
              (cloud_base_c[...,0]<x_limit[1]-limit_pad)
        # roi = np.logical_and(roi, image_roi)
        success = grasper.take_from_shelf(cloud_base_c, n_candidate, n_nms, roi_mask=roi)
        if not success:
            nograsp_cnt += 1
        else:
            nograsp_cnt = 0

        # Return to init pose
        bot.set_joint_positions(grasper.init_joint, nowait=True)
