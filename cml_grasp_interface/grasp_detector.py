import open3d
import os
import sys
import shutil
import cv2
import json
import torch
from pointnet2.utils import pointnet2_utils
from gdn.representation.euler_scene_att import *
from gdn.detector.pointnet2_att.backbone import Pointnet2MSG
from gdn.utils.transform import HPR
from nms import decode_euler_feature
from nms import initEigen, sanity_check
from nms import crop_index, generate_gripper_edge
from scipy.spatial.transform import Rotation
initEigen(0)
import numpy as np
import numba as nb
import pcl
import io

from .pyrobot_handler import PyRobotHandler

_VERBOSE = True
_DEBUG = True
_PLOT = True
_SAVE_PLOT = True
_OFFLINE = False
_CONSTRAINT_RANGE = True
_USE_OCTOMAP = False
_SCRIPT_DIR = os.path.split(os.path.abspath(__file__))[0]

plot_cnt = 0
if _PLOT:
    from mayavi import mlab
    figsize = (1280, 1024)
    #figsize = (640, 480)
    fig = mlab.figure(bgcolor=(0,0,0), size=figsize)

def eprint(s):
    if _VERBOSE:
        sys.stderr.write(str(s)+'\n')

def get_inlier_indices(cloud, n_neighbors=10, radius=0.01):
    tree = KDTree(cloud)
    dist, idx = tree.query(cloud, k=n_neighbors)
    inliers = np.all(dist<radius, axis=1)
    return inliers

def subsample(cloud, leaf_size=0.005, max_npts=12000, remove_outliers=True, voxelize=False):
    if len(cloud) == 0:
        return np.empty((0,3), dtype=np.float32), np.array([], dtype=np.int32)
    if voxelize:
        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(cloud)
        voxel, remap = pc.voxel_down_sample_and_trace(leaf_size, cloud.min(axis=0), cloud.max(axis=0))
        #  remap voxel -> pc
        inds = np.max(remap, axis=1)
        points = np.asarray(voxel.points, dtype=np.float32)
        del pc, voxel, remap
    else:
        if len(cloud) > max_npts:
            inds = np.random.choice(len(cloud), max_npts, replace=False)
            points = cloud[inds]
        else:
            inds = np.arange(len(cloud), dtype=np.int32)
            points = cloud
    if remove_outliers:
        inliers = get_inlier_indices(points)
        inds = inds[inliers]
        points = points[inliers]
    return points, inds

class GraspHandler(object):
    def __init__(self, bot, config_path, weights_path, state_dir=_SCRIPT_DIR):
        with open(config_path, 'r') as fp:
            self.config = json.load(fp)
            self.input_points = self.config['input_points']
        self.bot = bot
        self.model = Pointnet2MSG(self.config, activation_layer=EulerActivation())
        self.model = self.model.cuda()
        self.model = self.model.eval()
        self.model.load_state_dict(torch.load(weights_path)['base_model'])
        self.representation = EulerRepresentation(self.config)

        self.approach_step = 0.03
        self.lift_up = 0.20
        self.ready_up = 0.20
        self.max_up = 0.40

        self.drop_joint = np.load(state_dir + '/drop_joint.npy')
        self.drop_joint2 = np.load(state_dir + '/drop_joint2.npy')
        self.init_joint = np.load(state_dir + '/init_joint.npy')
        self.shot_joint = np.load(state_dir + '/shot_joint.npy')
        ee_minmax  = np.load(state_dir + '/ee_minmax.npy')
        self.ee_min     = ee_minmax[:3]
        self.ee_max     = ee_minmax[3:]
        del ee_minmax

    def grasp_proposal(self, pc_npy, n_candidate, n_nms, roi_mask=None, weighting=None):

        with torch.no_grad():
            pc_batch = torch.from_numpy(pc_npy).unsqueeze(0).float()
            pc_batch_cuda = pc_batch.cuda()
            pred, ind, att = self.model(pc_batch_cuda)
            pc_subsampled = pointnet2_utils.gather_operation(pc_batch_cuda.transpose(1, 2).contiguous(), ind).cpu().transpose(1, 2).numpy()[0].astype(np.float32) # (k, 3)
            ind = ind[0].cpu().numpy() # (K, )
            pred = pred[0].cpu().numpy().astype(np.float32) # (k, 16, 17, 8)
            if not roi_mask is None:
                # roi_mask: (N,)
                roi_mask = roi_mask[ind] # (N, ) -> (k, )
                pred = pred[roi_mask] # (k, ) -> (k',)
                pc_subsampled = pc_subsampled[roi_mask] # (k,) -> (k',)
            pred_poses = np.asarray(
                decode_euler_feature(
                    pc_subsampled,
                    pred.reshape(1, -1),
                    *pred.shape[:-1],
                    self.config['hand_height'],
                    self.config['gripper_width'],
                    self.config['thickness_side'],
                    self.config['rot_th'],
                    self.config['trans_th'],
                    n_candidate, # max number of candidate
                    -np.inf, # threshold of candidate
                    n_nms,  # max number of grasp in NMS
                    8,    # number of threads
                    True  # use NMS
                ), dtype=np.float32)

            if len(pred_poses) == 0:
                return []

            approach = pred_poses[:,:,0] # (N, 3)
            pred_poses[:,:,3] = pred_poses[:,:,3] - approach * self.approach_step
            pred_poses = np.asarray(sanity_check(pc_npy, pred_poses, 0,
                    self.config['gripper_width'],
                    self.config['thickness'],
                    self.config['hand_height'] + self.approach_step,
                    self.config['thickness_side'],
                    8 # num threads
            ), dtype=np.float32)
            if len(pred_poses) == 0:
                return pred_poses
            approach = pred_poses[:,:,0] # (N, 3)
            pred_poses[:,:,3] = pred_poses[:,:,3] + approach * self.approach_step
            return pred_poses

    def grasp_and_shot(self, cloud_base, n_candidate, n_nms, roi_mask=None, weighting=None, sleep=0.5, step_degree=90.0, steps=3):
        drop_joint = self.drop_joint
        drop_joint2 = self.drop_joint2
        init_joint = self.init_joint
        shot_joint = self.shot_joint
        ee_min = self.ee_min
        ee_max = self.ee_max
        self.bot.open_gripper(force=True)

        shot_rotation = np.array([0, 0, 0, 0, 0, step_degree/180.0*np.pi], dtype=np.float64)
        cloud_base = cloud_base.reshape(-1, 3)
        pc_npy, ind = subsample(cloud_base, leaf_size=0.005)
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

        shot_imgs = []

        while pc_npy.shape[0]<self.input_points:
            new_ind = np.random.choice(len(pc_npy), self.input_points-len(pc_npy), replace=True)
            new_pts = pc_npy[new_ind,:]
            new_pts = new_pts + np.random.randn(*new_pts.shape) * 1e-6
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
            return []
        pred_poses[:,:,3] += trans_to_frame


        if _CONSTRAINT_RANGE:
            new_poses = []
            for pose in pred_poses:
                approach = pose[:,0:1]
                if np.any(pose[:,3] < ee_min) or np.any(pose[:,3] > ee_max):
                    continue
                if np.arccos(np.dot(approach.reshape(1,3), np.array([0, 1, 0]).reshape(3,1))) > np.radians(80):
                    continue
                if np.arccos(np.dot(approach.reshape(1,3), np.array([0, 0, -1]).reshape(3,1))) > np.radians(75):
                    continue
                new_poses.append(pose)
            del pred_poses
            pred_poses = new_poses

        success = False
        for pose in pred_poses:
            trans = pose[:,3].reshape(3, 1).astype(np.float64)
            approach = pose[:,0:1].astype(np.float64)

            trans_back = trans - approach * self.approach_step
            trans_up = np.copy(trans)
            trans_up[2,0] = min(trans_up[2,0] + self.lift_up, self.max_up)
            trans_back_up = np.copy(trans_back)
            trans_back_up[2,0] = min(trans_back_up[2,0] + self.ready_up, self.max_up)
            rot = pose[:3,:3]
            if _OFFLINE:
                joint_back_up = self.bot.compute_ik(trans_back_up, rot, qinit=init_joint)
            else:
                joint_back_up = self.bot.compute_ik(trans_back_up, rot)
            if joint_back_up is None:
                eprint("No back up")
                continue
            joint_back = self.bot.compute_ik(trans_back, rot, qinit=joint_back_up)
            if joint_back is None:
                eprint("No back")
                continue
            joint_target = self.bot.compute_ik(trans, rot, qinit=joint_back)
            if joint_target is None:
                eprint("No target")
                continue
            joint_up = self.bot.compute_ik(trans_up, rot, qinit=joint_target)
            if joint_up is None:
                eprint("No up")
                continue

            if self.bot.set_joint_positions(joint_back_up) and \
               self.bot.set_joint_positions(joint_back) and \
               self.bot.set_joint_positions(joint_target):
                success = True
                break

        if success:
            # close gripper
            self.bot.close_gripper(force=True)

            # up
            self.bot.set_joint_positions(joint_up)

            # take shot here
            shot_imgs = []
            mid = steps // 2
            for d in range(-mid, steps-mid):
                if not self.bot.set_joint_positions(shot_joint + d * shot_rotation):
                    continue
                time.sleep(sleep)
                img = self.bot.get_image()
                shot_imgs.append(img)

            # drop
            self.bot.set_joint_positions(drop_joint)
            #self.bot.set_joint_positions(drop_joint2)

            # open gripper
            self.bot.open_gripper(force=True)
            self.bot.set_joint_positions(drop_joint)

            # Return to init pose
            # self.bot.set_joint_positions(init_joint)

            return shot_imgs
        return shot_imgs

    def grasp_and_scan(self, cloud_base, n_candidate, n_nms, roi_mask=None, weighting=None, sleep=2.0, step_degree=45, steps=8, obj_xscale=0.12, obj_yscale=0.08, obj_zscale=0.08, leaf_size=0.001):
        drop_joint = self.drop_joint
        drop_joint2 = self.drop_joint2
        init_joint = self.init_joint
        shot_joint = self.shot_joint
        ee_min = self.ee_min
        ee_max = self.ee_max

        self.bot.open_gripper(force=True)

        shot_rotation = np.array([0, 0, 0, 0, 0, step_degree/180.0*np.pi], dtype=np.float64)
        cloud_base = cloud_base.reshape(-1, 3)
        pc_npy, ind = subsample(cloud_base, leaf_size=0.005)
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

        shot_imgs = []

        while pc_npy.shape[0]<self.input_points:
            new_ind = np.random.choice(len(pc_npy), self.input_points-len(pc_npy), replace=True)
            new_pts = pc_npy[new_ind,:]
            new_pts = new_pts + np.random.randn(*new_pts.shape) * 1e-6
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
            return []
        pred_poses[:,:,3] += trans_to_frame


        if _CONSTRAINT_RANGE:
            new_poses = []
            for pose in pred_poses:
                approach = pose[:,0:1]
                if np.any(pose[:,3] < ee_min) or np.any(pose[:,3] > ee_max):
                    continue
                if np.arccos(np.dot(approach.reshape(1,3), np.array([0, 1, 0]).reshape(3,1))) > np.radians(80):
                    continue
                if np.arccos(np.dot(approach.reshape(1,3), np.array([0, 0, -1]).reshape(3,1))) > np.radians(75):
                    continue
                new_poses.append(pose)
            del pred_poses
            pred_poses = new_poses

        success = False
        for pose in pred_poses:
            trans = pose[:,3].reshape(3, 1).astype(np.float64)
            approach = pose[:,0:1].astype(np.float64)

            trans_back = trans - approach * self.approach_step
            trans_up = np.copy(trans)
            trans_up[2,0] = min(trans_up[2,0] + self.lift_up, self.max_up)
            trans_back_up = np.copy(trans_back)
            trans_back_up[2,0] = min(trans_back_up[2,0] + self.ready_up, self.max_up)
            rot = pose[:3,:3]
            if _OFFLINE:
                joint_back_up = self.bot.compute_ik(trans_back_up, rot, qinit=init_joint)
            else:
                joint_back_up = self.bot.compute_ik(trans_back_up, rot)
            if joint_back_up is None:
                eprint("No back up")
                continue
            joint_back = self.bot.compute_ik(trans_back, rot, qinit=joint_back_up)
            if joint_back is None:
                eprint("No back")
                continue
            joint_target = self.bot.compute_ik(trans, rot, qinit=joint_back)
            if joint_target is None:
                eprint("No target")
                continue
            joint_up = self.bot.compute_ik(trans_up, rot, qinit=joint_target)
            if joint_up is None:
                eprint("No up")
                continue

            if self.bot.set_joint_positions(joint_back_up) and \
               self.bot.set_joint_positions(joint_back) and \
               self.bot.set_joint_positions(joint_target):
                success = True
                break

        if success:
            # close gripper
            self.bot.close_gripper(force=True)

            # up
            self.bot.set_joint_positions(joint_up)
            self.bot.set_joint_positions(init_joint)

            # take shot here
            shot_imgs = []
            mid = steps // 2
            for d in range(-mid, steps-mid):
                if not self.bot.set_joint_positions(shot_joint + d * shot_rotation):
                    continue
                time.sleep(sleep)
                pc_obj_w_gripper, rgb = bot.get_xyz_rgb()
                rgb = rgb.astype(np.float32) / 255.0 # [0-255] -> [0-1]
                pc_obj_w_gripper = pc_obj_w_gripper.reshape(rgb.shape[0], rgb.shape[1], 3)
                pc_obj_w_gripper = np.append(pc_obj_w_gripper, rgb, axis=2).reshape(-1, 6) # (H, W, 6) -> (N, 6)
                ee_pose = self.bot.get_ee_pose()

                pc_obj_w_gripper[:,:3] = np.dot((pc_obj_w_gripper[:,:3] - ee_pose[:,3].reshape(1, 3)), ee_pose[:,:3]).astype(np.float32)
                pc_obj_w_gripper = pc_obj_w_gripper[ pc_obj_w_gripper[:,0] > self.config['hand_height']-obj_xscale/2.0 ]
                pc_obj_w_gripper = pc_obj_w_gripper[ pc_obj_w_gripper[:,0] < self.config['hand_height']+obj_xscale/2.0 ]
                pc_obj_w_gripper = pc_obj_w_gripper[ pc_obj_w_gripper[:,1] > -obj_yscale/2.0 ]
                pc_obj_w_gripper = pc_obj_w_gripper[ pc_obj_w_gripper[:,1] < obj_yscale/2.0 ]
                pc_obj_w_gripper = pc_obj_w_gripper[ pc_obj_w_gripper[:,2] > -obj_zscale/2.0 ]
                pc_obj_w_gripper = pc_obj_w_gripper[ pc_obj_w_gripper[:,2] < obj_zscale/2.0 ]

                camera_tf = self.bot.get_xyz_transform(self.bot.camera_frame, b'finger_link')
                camera_center_wrt_gripper = camera_tf[:3,3] # (3,)

                inliers = get_inlier_indices(pc_obj_w_gripper[:,:3], n_neighbors=100, radius=0.005)
                eprint("Inlier / Total: %.4f"%np.mean(inliers))
                pc_obj_w_gripper = pc_obj_w_gripper[ inliers  ]
                indices = subsample(pc_obj_w_gripper[:,:3], leaf_size=leaf_size, voxelize=True, remove_outliers=False)[1]
                pc_obj_w_gripper = pc_obj_w_gripper[indices]

                if len(pc_obj_w_gripper) == 0:
                    continue
                shot_imgs.append((pc_obj_w_gripper.astype(np.float32), ee_pose))


            # Back to up
            #self.bot.set_joint_positions(joint_up)

            # drop
            self.bot.set_joint_positions(drop_joint)
            #self.bot.set_joint_positions(drop_joint2)

            # open gripper
            self.bot.open_gripper(force=True)
            self.bot.set_joint_positions(drop_joint)

            # Return to init pose
            # self.bot.set_joint_positions(init_joint)

            return shot_imgs
        return shot_imgs

    def pick_and_place(self, cloud_base, n_candidate, n_nms, roi_mask=None, weighting=None):
        global plot_cnt

        drop_joint = self.drop_joint
        drop_joint2 = self.drop_joint2
        init_joint = self.init_joint
        shot_joint = self.shot_joint
        ee_min = self.ee_min
        ee_max = self.ee_max

        self.bot.open_gripper(force=True)
        #self.bot.set_joint_positions(init_joint)
        cloud_base = cloud_base.reshape(-1, 3)
        pc_npy, ind = subsample(cloud_base, leaf_size=0.0015, voxelize=True, remove_outliers=True)
        print("#points: %d"%len(pc_npy))
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
            #new_pts = new_pts + np.random.randn(*new_pts.shape) * 1e-6
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
                if np.arccos(np.dot(approach.reshape(1,3), np.array([0, 1, 0]).reshape(3,1))) > np.radians(80):
                    continue
                if np.arccos(np.dot(approach.reshape(1,3), np.array([0, 0, -1]).reshape(3,1))) > np.radians(75):
                    continue
                new_poses.append(pose)
            del pred_poses
            pred_poses = new_poses

        success = False
        for pose in pred_poses:
            trans = pose[:,3].reshape(3, 1).astype(np.float64)
            approach = pose[:,0:1].astype(np.float64)

            trans = trans - approach * 0.03 # fix offset # FIXME

            trans_back = trans - approach * self.approach_step
            trans_up = np.copy(trans)
            trans_up[2,0] = min(trans_up[2,0] + self.lift_up, self.max_up)
            trans_back_up = np.copy(trans_back)
            trans_back_up[2,0] = min(trans_back_up[2,0] + self.ready_up, self.max_up)
            rot = pose[:3,:3]
            if _OFFLINE:
                joint_back_up = self.bot.compute_ik(trans_back_up, rot, qinit=init_joint)
            else:
                joint_back_up = self.bot.compute_ik(trans_back_up, rot)
            if joint_back_up is None:
                eprint("No back up")
                continue
            joint_back = self.bot.compute_ik(trans_back, rot, qinit=joint_back_up)
            if joint_back is None:
                eprint("No back")
                continue
            joint_target = self.bot.compute_ik(trans, rot, qinit=joint_back)
            if joint_target is None:
                eprint("No target")
                continue
            joint_up = self.bot.compute_ik(trans_up, rot, qinit=joint_target)
            if joint_up is None:
                eprint("No up")
                continue

            if _PLOT:
                pc_subset = pc_npy-trans_to_frame
                mlab.view(315, 60, 0.8)
                col = (pc_subset[:,2] - pc_subset[:,2].min()) / (pc_subset[:,2].max() - pc_subset[:,2].min()) + 0.33
                mlab.points3d(pc_subset[:,0], pc_subset[:,1], pc_subset[:,2], col, scale_factor=0.003, mode='sphere', colormap='plasma', opacity=1.0, figure=fig)
                for candidate in pred_poses[:20]:
                    candidate_vis = np.copy(candidate)
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

            if self.bot.set_joint_positions(joint_back_up) and \
               self.bot.set_joint_positions(joint_back) and \
               self.bot.set_joint_positions(joint_target):
                success = True
                break

        if success:
            # close gripper
            self.bot.close_gripper(force=True)

            # up
            self.bot.set_joint_positions(joint_up)

            # drop
            self.bot.set_joint_positions(drop_joint)
            #self.bot.set_joint_positions(drop_joint2)

            # open gripper
            self.bot.open_gripper(force=True)
            self.bot.set_joint_positions(drop_joint)

            return True
        return False

@nb.njit
def gaussian1d(x, mu, sigma):
    g = np.exp(-(((x-mu)/sigma)**2)/2.0) / sigma / np.sqrt(2.0*np.pi)
    return g

@nb.njit
def gaussian2d(x,y,mu_x,mu_y,sigma_x,sigma_y):
    g_x = gaussian1d(x,mu_x,sigma_x)
    g_y = gaussian1d(y,mu_y,sigma_y)
    return g_x * g_y

def gaussian2d_kernel(shape, sigma_x=None, sigma_y=None, max_normalize=True, mag=6.0):
    if sigma_x is None:
        sigma_x = np.floor(shape[0] / mag)
    if sigma_y is None:
        sigma_y = np.floor(shape[1] / mag)
    xx, yy = np.meshgrid(np.arange(shape[0], dtype=np.float32), np.arange(shape[1], dtype=np.float32))
    cx = shape[0]/2.0
    cy = shape[1]/2.0
    z = gaussian2d(xx, yy, cx, cy, sigma_x, sigma_y)
    if max_normalize:
        z = z / z.max()
    return z

def grasp_wrapper(bbox, cloud_base, grasper, n_candidate=5000, n_nms=5000):
    bbox = np.array(bbox, dtype=np.int32)

    x_limit = (grasper.ee_min[0], grasper.ee_max[0])
    y_limit = (grasper.ee_min[1], grasper.ee_max[1])
    z_limit = (-0.09, 0.20)
    table_threshold = -0.09

    #bot = PyRobotHandler()
    #grasper = GraspHandler(bot, config_path, weights_path)
    #cloud_base, image = bot.get_xyz_rgb()
    # bounding box -> image_roi (mask) : (im_height, im_width) : np.boolean
    h, w, c = cloud_base.shape
    image_roi = np.zeros((h, w), dtype=bool)
    image_roi[bbox[1]:bbox[3], bbox[0]:bbox[2]] = True
    weighting_mask = None
    #weighting_mask = np.zeros((h, w), dtype=np.float32)
    #weighting_mask[bbox[1]:bbox[3], bbox[0]:bbox[2]] = gaussian2d_kernel((bbox[2]-bbox[0], bbox[3]-bbox[1]))

    #cv2.imwrite('roi_prev.png', image_roi * 255)
    #cv2.imwrite('weighting.png', weighting_mask * 255)

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
    cloud_base = cloud_base[crop] # (im_height, im_width, 3) -> (???, 3)
    image_roi = image_roi[crop] # (im_height, im_width, 3) -> (???, 3)

    roi = cloud_base[...,2]>table_threshold  # roi for detection
    roi = np.logical_and(roi, image_roi)

    # points = cloud_base.reshape(-1, 3)
    # temp_pc = pcl.PointCloud(points)
    # pcl.save(temp_pc, "cropped.ply", format = 'ply')
    # print ('Roi image is saved.')

    eprint('Grasp success? ', grasper.pick_and_place(cloud_base, n_candidate, n_nms, roi_mask=roi, weighting=weighting_mask))


def shot_wrapper(cloud_base, grasper, n_candidate=5000, n_nms=5000, step_degree=90.0, steps=3):

    x_limit = (grasper.ee_min[0], grasper.ee_max[0])
    y_limit = (grasper.ee_min[1], grasper.ee_max[1])
    z_limit = (-0.09, 0.20)
    table_threshold = -0.09

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
    cloud_base = cloud_base[crop] # (im_height, im_width, 3) -> (???, 3)

    roi = cloud_base[...,2]>table_threshold  # roi for detection

    imgs = grasper.grasp_and_shot(cloud_base, n_candidate, n_nms, roi_mask=roi, step_degree=step_degree, steps=steps)
    return imgs

def gripper_roi(x):
    roi = (x[:,0]>0.03) & \
    (x[:,0]<0.20) & \
    (np.abs(x[:,1])<0.08) & \
    (np.abs(x[:,2])<0.20)
    return roi

def o3d_icp(source_p, target_p, overlap_threshold=0.6, use_gripper_roi=False):

    if use_gripper_roi:
        source_p_roi = source_p[~gripper_roi(source_p[:,:3])]
        target_p_roi = target_p[~gripper_roi(target_p[:,:3])]
    else:
        source_p_roi = source_p
        target_p_roi = target_p

    radius_levels = [0.005, 0.003, 0.001, 0.0005]
    max_iters = [50, 100, 500, 3000]
    current_transformation = np.identity(4)

    source = open3d.geometry.PointCloud()
    target = open3d.geometry.PointCloud()
    source.points = open3d.utility.Vector3dVector(source_p_roi[:,:3])
    target.points = open3d.utility.Vector3dVector(target_p_roi[:,:3])
    #source.colors = open3d.utility.Vector3dVector(source_p_roi[:,3:])
    #target.colors = open3d.utility.Vector3dVector(target_p_roi[:,3:])

    for iters, radius in zip(max_iters, radius_levels):
        reg_p2p = open3d.registration.registration_icp(source, target, radius, current_transformation,
                                             open3d.registration.TransformationEstimationPointToPoint(),
                                             open3d.registration.ICPConvergenceCriteria(max_iteration=iters,
                                                                relative_fitness=1e-6, relative_rmse=1e-6)
                                             )
        current_transformation = reg_p2p.transformation

    evaluation_result = open3d.registration.evaluate_registration(source, target, 0.005, reg_p2p.transformation)
    eprint(evaluation_result.fitness)
    if evaluation_result.fitness < overlap_threshold:
        return np.empty((0, 6), dtype=np.float32)
    RT = reg_p2p.transformation[:3,:4]
    source_p_align = np.copy(source_p)
    source_p_align[:,:3] = np.dot(source_p[:,:3], RT[:3,:3].T) + RT[:3,3].reshape(1, 3) # (N, 3)
    return source_p_align

def merge_shots(shots):
    N = len(shots)
    # construct the binary merge tree
    leaf_level = int(np.ceil(np.log2(N)))
    num_node = int(2 ** (leaf_level+1) - 1)
    pc_merged = [ np.empty((0, 6), dtype=np.float32) for _ in range(num_node - N + 1)  ] # one empty + inner nodes
    pc_merged += [ x for x, _ in shots ] # leaves
    pc_merged += [ np.empty((0, 6), dtype=np.float32) ] # safe guard
    # Traverse the binary tree bottom-up
    for n in reversed(range(1, num_node+1)):
        if n % 2 == 0: # left child
            lch = n
            rch = n+1
            par = n // 2
            if len(pc_merged[lch]) == 0:
                pc_merged[par] = pc_merged[rch]
            elif len(pc_merged[rch]) == 0:
                pc_merged[par] = pc_merged[lch]
            else:
                lch2rch = o3d_icp(pc_merged[lch], pc_merged[rch])
                pc_merged[par] = np.append(lch2rch, pc_merged[rch], axis=0)
            del pc_merged[-1]
            del pc_merged[-1]
    ret = np.copy(pc_merged[1])
    del pc_merged
    return ret

if __name__ == '__main__':
    torch.backends.cudnn.benchmark = True
    torch.multiprocessing.set_start_method('forkserver')

    config_path = sys.argv[1]
    weights_path = sys.argv[2]
    n_candidate = int(sys.argv[3])
    n_nms = int(sys.argv[4])

    if not os.path.exists('./shots'):
        os.makedirs('./shots')

    #bot = PyRobotHandler(offline=_OFFLINE, octomap=_USE_OCTOMAP, bot="kinect", camera="kinect2_rgb_optical_frame")
    bot = PyRobotHandler(offline=_OFFLINE, octomap=_USE_OCTOMAP, bot="kinect", camera="rgb_camera_link")
    #bot = PyRobotHandler(offline=_OFFLINE, octomap=_USE_OCTOMAP, bot="ur3", camera="camera_color_optical_frame")
    #bot = PyRobotHandler(offline=_OFFLINE, octomap=_USE_OCTOMAP)
    grasper = GraspHandler(bot, config_path, weights_path)

    x_limit = (grasper.ee_min[0], grasper.ee_max[0])
    y_limit = (grasper.ee_min[1], grasper.ee_max[1])
    z_limit = (-0.09, 0.20)
    limit_pad = 0.03
    table_threshold = -0.10

    bot.set_joint_positions(grasper.init_joint)
    bot.disable_octomap()
    bot.update_octomap()
    bot.enable_octomap()
    bot.open_gripper(force=True)
    camera_pos = bot.get_xyz_transform(b'base_link', bot.camera_frame)[:3,3]

    shot_no = 0
    nograsp_cnt = 0
    input("Press ENTER to start...")
    while True:
        if nograsp_cnt >= 10:
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
        cloud_base_c = cloud_base_c[HPR(cloud_base_c, camera_pos, np.pi*0.6)]

        roi = (cloud_base_c[...,2]>table_threshold) & \
              (cloud_base_c[...,1]>y_limit[0]+limit_pad) & \
              (cloud_base_c[...,1]<y_limit[1]-limit_pad) & \
              (cloud_base_c[...,0]>x_limit[0]+limit_pad) & \
              (cloud_base_c[...,0]<x_limit[1]-limit_pad)
        # roi = np.logical_and(roi, image_roi)
        if _DEBUG:
            temp_pc = pcl.PointCloud(cloud_base_c[roi].astype(np.float32))
            pcl.save(temp_pc, "cropped.ply", format = 'ply')
            eprint ('Roi ply is saved.')
        '''
        shots = grasper.grasp_and_scan(cloud_base_c, n_candidate, n_nms, roi_mask=roi)
        if len(shots) == 0:
            nograsp_cnt += 1
            continue
        else:
            nograsp_cnt = 0

        pc_merged = merge_shots(shots).astype(np.float32)
        pc_cropped_aligned = np.copy(pc_merged)
        pc_cropped_aligned = pc_cropped_aligned[ gripper_roi(pc_cropped_aligned[:,:3])  ]

        shot_no += 1

        for n, (x, _) in enumerate(shots):
            pc = open3d.geometry.PointCloud()
            pc.points = open3d.utility.Vector3dVector(x[:,:3])
            pc.colors = open3d.utility.Vector3dVector(x[:,3:])
            open3d.io.write_point_cloud('./shots/obj-%d-%d.ply'%(shot_no, n), pc)

        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(pc_merged[:,:3])
        pc.colors = open3d.utility.Vector3dVector(pc_merged[:,3:])
        open3d.io.write_point_cloud('./shots/merged-%d.ply'%shot_no, pc)
        del pc

        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(pc_cropped_aligned[:,:3])
        pc.colors = open3d.utility.Vector3dVector(pc_cropped_aligned[:,3:])
        open3d.io.write_point_cloud('./shots/object-%d.ply'%shot_no, pc)

        '''
        success = grasper.pick_and_place(cloud_base_c, n_candidate, n_nms, roi_mask=roi)
        if success == 0:
            nograsp_cnt += 1
            continue
        else:
            nograsp_cnt = 0

        # Return to init pose
        bot.set_joint_positions(grasper.init_joint, nowait=True)
