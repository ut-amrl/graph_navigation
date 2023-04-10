#!/usr/bin/env python
import sys
import time
import numpy as np
import cv2
import roslib
import math
import torchvision.models.segmentation
import torch
import torchvision.transforms as tf
from torch.nn import DataParallel as DP
import rospy
from PIL import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
# include "amrl_msgs/Localization2DMsg.h"

# -----------------------------------------------------------------------
# PARAMETERS
CKPT_PATH = "/home/dynamo/Music/4000.pt"
CAM_IMG_TOPIC = "/camera/rgb/image_raw/compressed"
LOC_PUB_TOPIC = "/contingency/safe_pose"
OVERLAY_PUB_TOPIC = "/contingency/overlay/compressed"
OUT_FPS = 30
# -----------------------------------------------------------------------


class SpotAzureKinectCameraProjection:
    def __init__(self):
        """
        Frames nomenclature:
        (1) World Coordinates: Attached to the robot at the centre of robot's base (Z up, X forward, Y towards robot's left hand side)
        (2) Camera Pose: Attached at the camera centre, basically translate (1) to that point and then rotate about +Y axis so as to align +X such that it comes straight out of the camera lens
        (3) Camera Coordinates: Attached at the camera centre, basically rotate (2 rotations) (2) such that the new +Y is along old -Z and new +X is along old -Y
        (4) Pixel Coordinaes: 3D to 2D intrinsic matrix brings (3) to (4)
        """
        self.M_intrinsic = self.get_M_intrinsic()
        self.kinect_pose_params = self.get_kinect_pose_params()
        self.M_perspective = self.get_M_perspective()
        self.mats = self.get_projection_mats()

    def get_projection_mats(self):
        T_43 = self.M_intrinsic @ self.M_perspective  # 3x4
        T_32 = self.get_std_rot("X", -math.pi/2) @ self.get_std_rot("Z", -math.pi/2)  # 4x4
        T_21 = self.get_std_rot(*self.kinect_pose_params["rot"]) @ self.get_std_trans(*self.kinect_pose_params["trans"])  # 4x4
        T_41 = T_43 @ T_32 @ T_21  # 3x4
        H_41 = T_41[:, [0, 1, 3]]  # homography for Z=0 plane, 3x3
        H_14 = np.linalg.inv(H_41)  # 3x3
        return {"T_43": T_43, "T_32": T_32, "T_21": T_21, "T_41": T_41, "H_41": H_41, "H_14": H_14}

    def get_M_intrinsic(self):
        # 3x3
        mat = [
            [934.6554, 0, 1028.3415],
            [0, 938.6148, 747.5388],
            [0, 0, 1]
        ]
        return np.array(mat)

    def get_kinect_pose_params(self):
        # metres, radians
        # first did translation, then rotation
        params = {}
        params["cx"] = 24.5 / 100
        params["cy"] = -2 / 100   # initially I measured 7, but -2 seems to be the correct one
        params["cz"] = 76 / 100
        params["trans"] = (params["cx"], params["cy"], params["cz"])
        params["rot"] = ("Y", np.deg2rad(15.2))
        return params

    def get_ordinary_from_homo(self, v):
        # Scales so that last coord is 1 and then removes last coord
        o = v.squeeze()
        o = o/o[-1]
        return o[:-1]

    def get_homo_from_ordinary(self, v):
        o = list(v.squeeze())
        o.append(1)
        return np.array(o)

    def get_std_trans(self, cx, cy, cz):
        # cx, cy, cz are the coords of O_M wrt O_F when expressed in F
        mat = [
            [1, 0, 0, -cx],
            [0, 1, 0, -cy],
            [0, 0, 1, -cz],
            [0, 0, 0, 1]
        ]
        return np.array(mat)

    def get_std_rot(self, axis, alpha):
        # axis is either "X", "Y", or "Z" axis of F and alpha is positive acc to right hand thumb rule dirn
        if axis == "X":
            mat = [
                [1, 0, 0, 0],
                [0, math.cos(alpha), math.sin(alpha), 0],
                [0, -math.sin(alpha), math.cos(alpha), 0],
                [0, 0, 0, 1]
            ]
        elif axis == "Y":
            mat = [
                [math.cos(alpha), 0, -math.sin(alpha), 0],
                [0, 1, 0, 0],
                [math.sin(alpha), 0, math.cos(alpha), 0],
                [0, 0, 0, 1]
            ]
        elif axis == "Z":
            mat = [
                [math.cos(alpha), math.sin(alpha), 0, 0],
                [-math.sin(alpha), math.cos(alpha), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        else:
            raise ValueError("Invalid axis!")
        return np.array(mat)

    def get_M_perspective(self):
        # 3x4
        mat = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]
        return np.array(mat)


class ExtractSafeSpot:
    def __init__(self):
        self.latest_image = None
        self.height = 1280//2
        self.width = 1920//2
        self.transformImg = tf.Compose([tf.ToTensor(), tf.Resize((self.height, self.width)), tf.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))])
        self.device = self._load_device()
        self.bmask_model = self._load_bmask_model(CKPT_PATH)
        self.bmask_cmap = self.get_bmask_cmap()
        self.kinect = SpotAzureKinectCameraProjection()

        rospy.Subscriber(CAM_IMG_TOPIC, CompressedImage, self.image_callback, queue_size=1, buff_size=2**32)
        self.loc_pub = rospy.Publisher(LOC_PUB_TOPIC, Float64MultiArray, queue_size=1)
        self.overlay_pub = rospy.Publisher(OVERLAY_PUB_TOPIC, CompressedImage, queue_size=1)
        rospy.Timer(rospy.Duration(1/OUT_FPS), self.processing_callback)

    def _load_device(self):
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        return device

    def _load_bmask_model(self, cpath):
        Net = torchvision.models.segmentation.deeplabv3_resnet50(pretrained=True)  # Load net
        Net.classifier[4] = torch.nn.Conv2d(256, 2, kernel_size=(1, 1), stride=(1, 1))  # Change final layer to 2 classes
        Net.backbone.conv1 = torch.nn.Conv2d(3, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False)
        Net = DP(Net, device_ids=[0])
        Net = Net.to(self.device)
        ckpt = torch.load(cpath)
        Net.load_state_dict(ckpt['state_dict'])
        Net.double()
        Net.eval()
        return Net

    def get_bmask_cmap(self):
        cmap = np.array([[0, 0, 0], [255, 255, 255]], dtype=np.uint8)
        return cmap

    def image_callback(self, msg):
        self.latest_image = msg.data

    def processing_callback(self, _):
        def mask1D_to_rgb(mask1D, cmap):
            if mask1D.shape[-1] == 1:
                mask1D = mask1D.squeeze()
            return cmap[mask1D]

        if self.latest_image is None:
            return
        img1 = np.frombuffer(self.latest_image, np.uint8)
        img2 = cv2.imdecode(img1, cv2.IMREAD_COLOR)
        img3 = np.array(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))

        pred_bmask = self.get_bmask(img3)  # gives 0 1 matrix binary mask
        b1, pixel_x, pixel_y = self.deduce_safe_loc(pred_bmask)
        b2, orient_1_x, orient_1_y, orient_2_x, orient_2_y = self.deduce_safe_angle(pred_bmask, pixel_x, pixel_y)

        if (b1 and b2):
            loc = self.kinect.get_homo_from_ordinary(np.array([pixel_x, pixel_y]))
            orient1 = self.kinect.get_homo_from_ordinary(np.array([orient_1_x, orient_1_y]))
            orient2 = self.kinect.get_homo_from_ordinary(np.array([orient_2_x, orient_2_y]))

            loc_w = self.kinect.get_ordinary_from_homo(self.kinect.mats["H_14"] @ loc).squeeze()
            orient1_w = self.kinect.get_ordinary_from_homo(self.kinect.mats["H_14"] @ orient1).squeeze()
            orient2_w = self.kinect.get_ordinary_from_homo(self.kinect.mats["H_14"] @ orient2).squeeze()

            rel_ang = math.atan((orient2_w[0] - orient1_w[0]) / (orient2_w[1] - orient1_w[1]))
        else:
            loc_w = np.array([0, 0])
            rel_ang = 0

        # publishing 6 floats:
        #       0 -> bool for gotSafePose (1 means true)
        #       1 -> bool for loc (0 -> local, 1 -> ground)
        #       2 -> x
        #       3 -> y
        #       4 -> bool for theta (0 -> local, 1 -> ground)
        #       5 -> theta

        msg = Float64MultiArray()
        msg.data.append(float(b1 and b2))
        msg.data.append(0)
        msg.data.append(loc_w[0])
        msg.data.append(loc_w[1])
        msg.data.append(0)
        msg.data.append(rel_ang)
        self.loc_pub.publish(msg)

        bmask3d = mask1D_to_rgb(pred_bmask, self.bmask_cmap)
        overlay = np.array(cv2.addWeighted(bmask3d, 0.4, img2, 1-0.4, gamma=0))
        overlay = cv2.circle(overlay, (pixel_x, pixel_y), radius=50, color=(0, 0, 255), thickness=-1)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', overlay)[1]).tobytes()
        self.overlay_pub.publish(msg)

    def get_bmask(self, img):
        """
        Outputs a 0, 1 binary mask for safe pixels
        TOIMPLEMENT: Implement other cases, using raw_nosemseg for now
        """
        def prepare_input_model(img):
            img = np.array(img, dtype=np.float64) / 255.0
            img = self.transformImg(img)
            return img

        height_ini, width_ini, depth_ini = img.shape
        inp = prepare_input_model(img)
        inp = torch.autograd.Variable(inp, requires_grad=False).to(self.device).unsqueeze(0)
        with torch.no_grad():
            Prd = self.bmask_model(inp)['out']  # Run net
        Prd = tf.Resize((height_ini, width_ini))(Prd[0])
        seg = torch.argmax(Prd, 0).detach().cpu().numpy().squeeze()
        return seg

    def deduce_safe_loc(self, bmask, pad_=50):
        """
        Outputs pixel locs of safe loc. Scans from bottom to top to find a safe pixel with padding at all 4 dirns
        Return False and garbage values if not possible
        TOIMPLEMENT
        """
        # DUMMY
        bmask = bmask.squeeze()
        for i in range(bmask.shape[0]-1, -1, -1):  # y
            for j in range(bmask.shape[1]-1, -1, -1):  # x
                if bmask[i, j] == 1:
                    return True, j, i

        return False, -1, -1

    def deduce_safe_angle(self, bmask, x, y, min_=50):
        """
        For now (later, do a more sophisticated approach), just scans 8 possible dirns and sees where longest length of safe pixels, and returns the pair of opposite dirns with highest sum (min_ reqd)
        Convention, #1 has lower y (ie, upwards)
        Return False and garbage values if not possible
        TOIMPLEMENT
        """
        # DUMMY
        return True, x, y-1, x, y+1  # just go forward


def setup_ros_node():
    rospy.init_node('contingency_image_extractor', anonymous=False)
    e = ExtractSafeSpot()
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS safe spot extractor module")


setup_ros_node()
