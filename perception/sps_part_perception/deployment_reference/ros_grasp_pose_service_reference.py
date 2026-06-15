#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import torch
from sensor_msgs.msg import Image
from kuavo_msgs.msg._sensorsData import sensorsData
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
from dynamic_biped.srv import  GetTargetPartPoseInCamera, GetTargetPartPoseInCameraResponse
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from network.network_kuavo import get_resnet18_with_sigmoidencoder
from dataset.data_preprocesser import normalize_images
from torchvision.models import resnet18, ResNet18_Weights
from tf.transformations import quaternion_from_euler
import datetime
import atexit
import time
import traceback
import sys
from score_model import RandLANet
from utils_func import rgbd_mask_to_pointcloud_batch



              
def _excepthook(etype, value, tb):
    msg = "".join(traceback.format_exception(etype, value, tb))
    try:
        rospy.logerr("UNCAUGHT EXCEPTION:\n" + msg)
    except Exception:
        sys.stderr.write("UNCAUGHT EXCEPTION:\n" + msg + "\n")

sys.excepthook = _excepthook


class KuavoPoseService:
    def __init__(self, vis = False):
        rospy.init_node("kuavo_pose_service")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(self.device)
        self.bridge = CvBridge()
        
        self.class_names = ['checkerboard', 'seat_belt', 'striker']
        self.seg_model = self.load_model("seg")
        self.score_model = self.load_model("score")

        #self.vis = vis
        self.vis = True
        self.head_img = None
        self.head_depth = None
        self.headSeg = None
        self.headsegscore = None
        self.set_object = None
        self.marker_publish_rate = 1
        
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_cb)
        # rospy.Subscriber("/cam_h/color/image_raw", Image, self.rgb_cb)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb)
        # rospy.Subscriber("/cam_h/depth/image_raw", Image, self.depth_cb)
        rospy.Subscriber("/sensors_data_raw", sensorsData, self.joint_state_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pose_pub = rospy.Publisher("/kuavo_head_pose", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/target_parts_visualization', Marker, queue_size=10)
        self.pose_service = rospy.Service("/get_target_part_pose_in_camera", GetTargetPartPoseInCamera, self.handle_request)
        
        
        self.current_target_pose_in_world = None
        self.marker_timer = rospy.Timer(rospy.Duration(1.0/self.marker_publish_rate), self.vis_world_pose_in_rviz_callback)
        self.marker_size = [0.015, 0.015, 0.015]
        
        self.camera_base_link_name = 'camera_color_optical_frame'
        # self.camera_base_link_name = 'cam_h_color_optical_frame'
        self.base_link_name = 'base_link'
        self.world_link_name = 'odom'
        
        rospy.loginfo("Kuavo pose estimation service is ready.")
        rospy.spin()


    def load_model(self, name):
        def _load_state_dict_safely(model, ckpt_path):
            ckpt = torch.load(ckpt_path, map_location=self.device)
            # Support both training checkpoints and plain state_dict files.
            if isinstance(ckpt, dict):
                state = ckpt.get("state_dict") or ckpt.get("model") or ckpt
            else:
                state = ckpt
            # Strip DataParallel's module. prefix when present.
            if isinstance(state, dict) and any(k.startswith("module.") for k in state.keys()):
                state = {k.replace("module.", "", 1): v for k, v in state.items()}
            # Use non-strict loading so historical checkpoints remain usable.
            model.load_state_dict(state, strict=False)
            return model
        if name == 'score':
            model = RandLANet(7, 10, 16, 4, self.device)
            model = _load_state_dict_safely(model, "./ckpts/score_model.pth")
            return model.to(self.device).eval()
        elif name == 'seg':
            return YOLO("ckpts/yolo12_seg.pt").to(self.device).eval()
        else:
            raise ValueError(f"Unknown model name: {name}")
        
    
    #def rgb_cb(self, msg):
        #try:
        #    self.head_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #except Exception as e:
         #   rospy.logerr(f"RGB callback error: {e}")
    def rgb_cb(self, msg):
        try:
            self.head_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            rospy.logerr("rgb_cb exception:\n" + traceback.format_exc())        
        try:
            headImg = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            headDepth = self.head_depth
            if headImg is not None and headDepth is not None:
                # assert headImg.shape == (480, 640, 3)
                # assert headDepth.shape == (480, 640, 1)
                # assert headImg.shape == (480, 848, 3)
                # assert headDepth.shape == (480, 848, 1)
                self.head_img = headImg
                # self.head_img = cv2.resize(self.head_img,(640,480))
                masks = np.zeros((headImg.shape[0],headImg.shape[1]),dtype=np.uint8)
                if self.headSeg is not None:
                    masks = self.headSeg
                if self.vis:
                    overlay = headImg.copy()
                    dis_play_mask = (masks.copy() * 255).astype(np.uint8)
                    dis_play_mask = np.expand_dims(dis_play_mask, axis=-1)
                    dis_play_mask = np.repeat(dis_play_mask, 3,axis=-1)
                    # print(dis_play_mask.shape, overlay.shape)
                    overlay = cv2.addWeighted(overlay, 0.7, dis_play_mask, 0.3, 0)
                    cv2.imshow("Head seg", overlay)
                    cv2.moveWindow("Head seg", 0, 0)
                    cv2.waitKey(1)
      
        except CvBridgeError as e:
            rospy.logerr(f"Head image conversion error: {e}")
        except AssertionError as e:
            rospy.logerr(e)
            print("error")
        """
                cv2.imshow("Head seg", overlay)
                cv2.moveWindow("Head seg", 0, 0)
                cv2.waitKey(1)
        """

    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.head_depth = np.expand_dims(depth, -1)
            # self.head_depth = cv2.resize(self.head_depth,(640,480))
        except Exception:
            # rospy.logerr(f"Depth callback error: {e}")
            rospy.logerr("depth_cb exception:\n"+ traceback.format_exc())

    def joint_state_callback(self, msg): # Used in real
        try:
            joint_q = msg.joint_data.joint_q
            slice_robot = [(12, 19), (19, 26)]
            self.joint_q = np.concatenate((joint_q[slice_robot[0][0]:slice_robot[0][1]], joint_q[slice_robot[1][0]:slice_robot[1][1]]), axis=0)
            if self.joint_q[1] > 0.7:
                self.which_arm = "left"
            elif self.joint_q[8] < -0.7:
                self.which_arm = "right"
            else:
                self.which_arm = None
        except Exception:
            rospy.logerr("joint_state_callback exception:\n"+ traceback.format_exc())



    def handle_request(self, req):
        response = GetTargetPartPoseInCameraResponse()
        # self.set_object = "dang_ban"
        self.set_object = ["an_quan_dai", "men_ba_shou"]
        # self.set_object = ['an_quan_dai', 'dang_ban', 'dian_xian_sheng', 'men_ba_shou', 'ren_xing', 'su_liao', 'ti_xing', 'wan_qu', 'yuan_xing']
        if self.head_img is None or self.head_depth is None:
            rospy.logwarn("Image or depth not ready.")
            return response

        try:
            with torch.no_grad():
                seg_result = self.seg_model.predict(self.head_img)
                if not seg_result or seg_result[0] is None or seg_result[0].masks == None:
                    rospy.logwarn("No objects detected.")
                    return response
                # seg_result = self.seg_model.predict(self.head_img, conf=0.7)
                ori_masks = seg_result[0].masks.data.cpu().numpy()
                print(self.head_img.shape, ori_masks.shape)
                cls_pred = seg_result[0].boxes.cls.data.cpu().numpy().astype(int)
                conf = seg_result[0].boxes.conf.data.cpu().numpy()
                name = [seg_result[0].names[int(index)] for index in cls_pred]
                cls_names = seg_result[0].names
                print("all cls name:", cls_names)
                print("mask shape:", ori_masks.shape)
                print("cls shape:", cls_pred.shape)
                print("corresponding name", name)
                print("predicted class", cls_pred)
                print("predicted confidence", conf)

                if np.sum([n in self.set_object for n in name]) == 0:
                    rospy.logwarn(f"No self.set_object '{self.set_object}' detected.")
                    return response

                # Filter masks by target object name
                masks = ori_masks[[n in self.set_object for n in name]]
                correspond_name = name[[n in self.set_object for n in name]]
                print("object:",self.set_object," ",[n in self.set_object for n in name])
                # Prepare image and depth batch
                image_batch = np.repeat(self.head_img[np.newaxis, :, :, :], len(masks), axis=0)
                depth_batch = np.repeat(self.head_depth[np.newaxis, :, :], len(masks), axis=0)
                print("head",image_batch.shape,depth_batch.shape,masks.shape, self.head_img.shape)
                # Convert RGBD data to point cloud
                points = rgbd_mask_to_pointcloud_batch(image_batch, depth_batch, masks, camera_type="orbbec")
                
                # print("points.shape",points.shape)

                # Get mask scores using the score model
                mask_scores = torch.sigmoid(self.score_model(torch.tensor(points, dtype=torch.float32, device=self.device)))
                # print("mask_scores",mask_scores)
                if isinstance(mask_scores, np.ndarray):
                    scores_t = torch.from_numpy(mask_scores)
                elif isinstance(mask_scores, torch.Tensor):
                    scores_t = mask_scores.detach().cpu()    
                else:
                    raise TypeError(f"Unsupported mask_scores type: {type(mask_scores)}")
                scores_t = scores_t.float().view(-1)  
                if scores_t.numel() == 0:
                    rospy.logwarn("No mask scores produced; aborting this request.")
                    return response
                # Get the best mask index (highest score)
                best_idx = torch.argmax(scores_t, dim=0).item()
                best_mask = masks[best_idx]  # Get the best mask
                correspond_object = correspond_name[best_idx]
                best_score = mask_scores[best_idx].item()  # Get the score for the best mask
                # class_idx = cls_pred[best_idx]  # Get the class index of the best mask
                # class_name = name[class_idx] # masks is not origin mask, cls name may have problem


                self.headsegscore = best_score
                self.headSeg = best_mask
                print(self.headSeg.shape)
                print(f"Selected 'object' mask with the score of {best_score}, idx: {best_idx}, name: {self.set_object}")

                pose_info = self.estimate_pose_from_mask_depth(best_mask)
                print(f"Pose-Camera Pose before revise Pos: {pose_info['position']}, Quat: {pose_info['orientation']}")
                if not pose_info:
                    response.success = False
                    response.message = "Pose estimation failed."
                    rospy.logwarn("Pose estimation failed.")
                    return response
                
                target_base_pose = self.get_base_pose_from_camera_pose_info(pose_info)
                revised_target_base_pose = self.revise_grasp_pose_in_base(target_base_pose) # Add bias
                target_world_pose = self.get_world_pose_from_base_pose(revised_target_base_pose)
                revised_target_world_pose = self.revise_grasp_pose_in_world(target_world_pose) # Compute grasping orientation 
                # revised_camera_pose = self.get_camera_pose_from_world_pose(revised_target_world_pose)
                
                # rpc = revised_camera_pose.pose.position
                # rqc = revised_camera_pose.pose.orientation
                # print(f"Pose-Camera after revise Pos: {[rpc.x, rpc.y, rpc.z]}, Quat: {[rqc.x, rqc.y, rqc.z, rqc.w]}")
                # pw = target_world_pose.pose.position
                # qw = target_world_pose.pose.orientation
                pw = target_base_pose.pose.position
                qw = target_base_pose.pose.orientation
                print(f"Pose-Base before revise Pos: {[pw.x, pw.y, pw.z]}, Quat: {[qw.x, qw.y, qw.z, qw.w]}")
                # print(f"Pose-World before revise Pos: {[pw.x, pw.y, pw.z]}, Quat: {[qw.x, qw.y, qw.z, qw.w]}")
                # rpw = revised_target_world_pose.pose.position
                # rqw = revised_target_world_pose.pose.orientation
                rpw = revised_target_base_pose.pose.position
                rqw = revised_target_base_pose.pose.orientation
                print(f"Pose-Base after revise Pos: {[rpw.x, rpw.y, rpw.z]}, Quat: {[rqw.x, rqw.y, rqw.z, rqw.w]}")
                # print(f"Pose-World after revise Pos: {[rpw.x, rpw.y, rpw.z]}, Quat: {[rqw.x, rqw.y, rqw.z, rqw.w]}")
                
                error = False
                if revised_target_world_pose.pose.position.z > 1.1 or revised_target_world_pose.pose.position.z < 0.6:
                    raise RuntimeError('Error pose!!!!!!')
                    error = True
                
                # Important compatibility note:
                # The response field is still named pose_in_camera because the
                # historical service definition used that name. In this
                # deployment reference, the returned pose is a base-frame target
                # pose after the camera-to-base transform and grasp offset have
                # been applied. Rename the service field in future integrations
                # if this ambiguity matters.
                if not error:
                    response.pose_in_camera = revised_target_base_pose.pose  
                    response.success = True
                    response.message = f"Using Estimated pose with score {float(self.headsegscore):.3f}, object name: {correspond_object}"
                    self.broadcast_pose(revised_target_base_pose, "kuavo_grasp_target")
                    self.current_target_pose_in_world = revised_target_world_pose.pose  
                else:
                    response.pose_in_camera = []  
                    response.success = False
                    response.message = "Fail"
                # self.pose_pub.publish(world_pose)
                return response

        except Exception:
            
            rospy.logerr("handle_request exception:\n" + traceback.format_exc())
            return GetTargetPartPoseInCameraResponse()
    def revise_grasp_pose_in_base(self, base_pose):
        # revise position
        print("############\n",base_pose)
        print("which_arm",self.which_arm)
        if self.which_arm == 'right':
            print("🚀🚀🚀🚀🚀🚀")
            x_bias = 0.0
            y_bias = 0.0
            z_bias = 0.1
        if self.which_arm == 'left':
            x_bias = 0.0
            y_bias = 0.0
            z_bias = 0.1
        if self.which_arm is None:
            x_bias = 0.0
            y_bias = 0.0
            z_bias = 0.1
        
        #if self.current_target == 'checkerboard':
        #    z_bias += 0.20
        print("🚗🚗🚗🚗🚗🚗")
        base_pose.pose.position.x += x_bias
        base_pose.pose.position.y += y_bias
        base_pose.pose.position.z += z_bias
        return base_pose
    
    def revise_grasp_pose_in_world(self, world_pose):
        # revise position
        # x_bias = 0.07
        # y_bias = -0.10
        # z_bias = -0.02
        # x_bias = 0.1
        # y_bias = -0.
        # z_bias = -0.05
        # world_pose.pose.position.x += x_bias
        # world_pose.pose.position.y += y_bias
        # world_pose.pose.position.z += z_bias
        # revise orientation
        try:
            which_arm = self.which_arm
        except:
            raise RuntimeError('You have not picked anything up yet!!!!')
            
        used_shoulder_pose = self.get_link_position_in_world('zarm_l3_link' if which_arm=='left' else 'zarm_r3_link', self.world_link_name)
        used_elbow_pose = self.get_link_position_in_world('zarm_l4_link' if which_arm=='left' else 'zarm_r4_link', self.world_link_name)
        
        grasp_world_position = np.array([world_pose.pose.position.x, world_pose.pose.position.y, world_pose.pose.position.z])
        # grasp_rpy = self.compute_grasp_rpy_by_position(grasp_world_position, used_shoulder_pose, which_arm)
        grasp_rpy = self.compute_natural_grasp_rpy_wrt_link(grasp_world_position, used_elbow_pose, which_arm)
        grasp_quat = self.euler_to_quaternion(grasp_rpy)
        world_pose.pose.orientation.x = grasp_quat[0]
        world_pose.pose.orientation.y = grasp_quat[1]
        world_pose.pose.orientation.z = grasp_quat[2]
        world_pose.pose.orientation.w = grasp_quat[3]
        
        return world_pose

    def compute_natural_grasp_rpy_wrt_link(self, grasp_pos, link_pos, which_arm='right', axis_preference='y', z_bias=None):
        z_axis = link_pos - grasp_pos
        if z_bias is not None:
            z_axis = z_axis + z_bias
        z_axis /= np.linalg.norm(z_axis)
        if axis_preference == 'y':
            hint = np.array([0, 0, 1])
        elif axis_preference == 'x':
            hint = np.array([1, 0, 0])
        else:
            raise ValueError("axis_preference must be 'x' or 'y'")
        if abs(np.dot(z_axis, hint)) > 0.95:
            hint = np.array([0, 1, 0]) if axis_preference == 'y' else np.array([0, 0, 1])
        first_axis = np.cross(hint, z_axis)
        first_axis /= np.linalg.norm(first_axis)
        second_axis = np.cross(z_axis, first_axis)
        if axis_preference == 'y':
            x_axis = first_axis
            y_axis = second_axis
        else:
            x_axis = second_axis
            y_axis = first_axis
        rot_matrix = np.column_stack((x_axis, y_axis, z_axis))
        rot = R.from_matrix(rot_matrix)
        if which_arm == 'right':
            z_correction = R.from_euler('z', np.pi)
            rot = rot * z_correction
        return rot.as_euler('xyz', degrees=False)

    def compute_grasp_rpy_by_position(self, grasp_pos, shoulder_pos, which_hand):
        shoulder_proj = np.array([shoulder_pos[0], shoulder_pos[1], grasp_pos[2]])
        z_axis = shoulder_proj - grasp_pos
        z_axis /= np.linalg.norm(z_axis)
    
        y_hint = np.array([0, 0, 1])
        if abs(np.dot(z_axis, y_hint)) > 0.95:
            y_hint = np.array([1, 0, 0])
    
        x_axis = np.cross(y_hint, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
    
        rot_matrix = np.column_stack((x_axis, y_axis, z_axis))
        base_rot = R.from_matrix(rot_matrix)
    
        if which_hand == 'right':
            z_correction = R.from_euler('z', np.pi)
            base_rot = base_rot * z_correction
    
        return base_rot.as_euler('xyz', degrees=False)
    
    def get_world_pose_from_camera_pose_info(self, camera_pose_info):
        cam_pose = PoseStamped()
        cam_pose.header.stamp = rospy.Time.now()
        cam_pose.header.frame_id = self.camera_base_link_name
        cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z = camera_pose_info["position"]
        cam_pose.pose.orientation.x, cam_pose.pose.orientation.y, cam_pose.pose.orientation.z, cam_pose.pose.orientation.w = camera_pose_info["orientation"]
        tf = self.tf_buffer.lookup_transform(self.world_link_name, self.camera_base_link_name, rospy.Time(0), rospy.Duration(1.0))
        world_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, tf)
        return world_pose
    
    def get_base_pose_from_camera_pose_info(self, camera_pose_info):
        cam_pose = PoseStamped()
        cam_pose.header.stamp = rospy.Time.now()
        cam_pose.header.frame_id = self.camera_base_link_name
        cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z = camera_pose_info["position"]
        cam_pose.pose.orientation.x, cam_pose.pose.orientation.y, cam_pose.pose.orientation.z, cam_pose.pose.orientation.w = camera_pose_info["orientation"]
        tf = self.tf_buffer.lookup_transform(self.base_link_name, self.camera_base_link_name, rospy.Time(0), rospy.Duration(1.0))
        base_pose = tf2_geometry_msgs.do_transform_pose(cam_pose, tf)
        return base_pose
    
    def get_camera_pose_from_world_pose(self, world_pose):
        tf = self.tf_buffer.lookup_transform(self.camera_base_link_name, self.world_link_name, rospy.Time(0), rospy.Duration(1.0))
        cam_pose = tf2_geometry_msgs.do_transform_pose(world_pose, tf)
        return cam_pose

    def get_world_pose_from_base_pose(self,base_pose):
        tf = self.tf_buffer.lookup_transform(self.world_link_name, self.base_link_name, rospy.Time(0), rospy.Duration(1.0))
        world_pose = tf2_geometry_msgs.do_transform_pose(base_pose, tf)
        return world_pose
        
    def broadcast_pose(self, pose_msg, frame_id):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = pose_msg.header.frame_id
        transform_stamped.child_frame_id = frame_id
        transform_stamped.transform.translation.x = pose_msg.pose.position.x
        transform_stamped.transform.translation.y = pose_msg.pose.position.y
        transform_stamped.transform.translation.z = pose_msg.pose.position.z
        transform_stamped.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform_stamped)

    def estimate_pose_from_mask_depth(self, mask, num_samples=5000):
        # 609.224692 0.000000 332.381142 0.000000 608.546638 246.951210 0.000000 0.000000 1.000000

        # fx, fy = 606.830144, 606.926766
        # cx, cy = 336.671303, 249.751083
        
        fx, fy = 367.05, 366.95
        cx, cy = 319.51, 237.11
        
        depth = self.head_depth.astype(np.float32) / 1000.0
        ys, xs = np.where(mask > 0)
        
        if len(xs) < 10:
            return None
        
        if len(xs) > num_samples:
            step = max(1, len(xs) // num_samples)
            indices = np.arange(0, len(xs), step)
        else:
            indices = np.arange(len(xs))
            
        
        indices = np.random.choice(len(xs), min(num_samples, len(xs)), replace=False)
        points = []
        for i in indices:
            u, v = xs[i], ys[i]
            d = depth[v, u]
            if d > 0 and not np.isnan(d):
                x = (u - cx) * d / fx
                y = (v - cy) * d / fy
                z = d
                points.append([x, y, z])
        if len(points) < 5:
            return None
        points = np.array(points).squeeze(axis=-1)
        z_vals = points[:, 2]
        valid_idx = (z_vals >= np.percentile(z_vals, 5)) & (z_vals <= np.percentile(z_vals, 95))
        print("get pose valid idx shape",valid_idx.shape, points.shape)
        points = points[valid_idx]
        if len(points) < 5:
            return None
        mean_xyz = np.mean(points, axis=0)
        cov = np.cov(points.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        x_axis = eigvecs[:, np.argmax(eigvals)]
        z_axis = np.array([0, 0, 1])
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        R_mat = np.column_stack((x_axis, y_axis, z_axis))
        quat = R.from_matrix(R_mat).as_quat()
        
        if not hasattr(self, 'last_quat') or self.last_quat is None:
            self.last_quat = quat
        else:
            alpha = 0.8
            quat = alpha * self.last_quat + (1 - alpha) * quat
            quat /= np.linalg.norm(quat)
            self.last_quat = quat
        
        return {
            "position": [float(round(x, 4)) for x in mean_xyz],
            "orientation": [float(round(q, 6)) for q in quat]
        }
    
    # def get_link_position_in_world(self, link_name, world_frame="odom"):
      #  try:
       #     tf_buffer = tf2_ros.Buffer()
        #    listener = tf2_ros.TransformListener(tf_buffer)
         #   # rospy.loginfo("Waiting for TF between [%s] and [%s]...", world_frame, link_name)
          #  tf_buffer.can_transform(world_frame, link_name, rospy.Time(0), rospy.Duration(5.0))
           # trans = tf_buffer.lookup_transform(world_frame, link_name, rospy.Time(0), rospy.Duration(1.0))
            #pos = trans.transform.translation
            #return np.array([pos.x, pos.y, pos.z])
        #except Exception as e:
         #   rospy.logerr("TF lookup failed: %s", str(e))
          #  return None
    def get_link_position_in_world(self, link_name, world_frame="odom"):
        try:     
            self.tf_buffer.can_transform(world_frame, link_name, rospy.Time(0), rospy.Duration(5.0))
            trans = self.tf_buffer.lookup_transform(world_frame, link_name, rospy.Time(0), rospy.Duration(1.0))
            pos = trans.transform.translation
            return np.array([pos.x, pos.y, pos.z])
        except Exception as e:
            rospy.logerr("TF lookup failed: %s", repr(e))
            return None
        
    def euler_to_quaternion(self, rpy):
        roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
        q = quaternion_from_euler(roll, pitch, yaw)
        return q
    
    def vis_world_pose_in_rviz_callback(self, event=None):
        try:
            if self.current_target_pose_in_world is not None:
                marker = Marker()
                marker.header.frame_id = self.world_link_name
                marker.header.stamp = rospy.Time.now()
                marker.ns = "target_parts"
                marker.id = 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose = self.current_target_pose_in_world
                marker.scale.x = self.marker_size[0]
                marker.scale.y = self.marker_size[1]
                marker.scale.z = self.marker_size[2]
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                self.marker_pub.publish(marker)

                axis_marker = Marker()
                axis_marker.header.frame_id = self.world_link_name
                axis_marker.header.stamp = rospy.Time.now()
                axis_marker.ns = "axis"
                axis_marker.id = 1001
                axis_marker.type = Marker.LINE_LIST
                axis_marker.action = Marker.ADD
                axis_marker.pose = self.current_target_pose_in_world
                axis_marker.scale.x = 0.01  
                axis_marker.color.a = 1.0

                from geometry_msgs.msg import Point
                from std_msgs.msg import ColorRGBA

                def p(x, y, z): return Point(x=x, y=y, z=z)

                axis_marker.points = [
                    p(0, 0, 0), p(0.1, 0, 0), 
                    p(0, 0, 0), p(0, 0.1, 0),  
                    p(0, 0, 0), p(0, 0, 0.1)  
                ]
                axis_marker.colors = [
                    ColorRGBA(1, 0, 0, 1), ColorRGBA(1, 0, 0, 1), 
                    ColorRGBA(0, 1, 0, 1), ColorRGBA(0, 1, 0, 1), 
                    ColorRGBA(0, 0, 1, 1), ColorRGBA(0, 0, 1, 1), 
                ]

                self.marker_pub.publish(axis_marker)
        except Exception:
            rospy.logerr("vis_world_pose_in_rviz_callback exception:\n" + traceback.format_exc())

if __name__ == '__main__':
    try:
        KuavoPoseService()
    except rospy.ROSInterruptException:
        pass
