#!/usr/bin/env python
import sys
import time
import numpy as np
import cv2
import roslib
import rospy
from PIL import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
# include "amrl_msgs/Localization2DMsg.h"


class ExtractSafeSpot:
    def __init__(self):
        rospy.Subscriber("/camera/rgb/image_raw/compressed",
                         CompressedImage, self.callback)
        self.loc_pub = rospy.Publisher(
            "/contingency/safe_pose", Float64MultiArray, queue_size=1)

    def callback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np_rgb = np.array(cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB))
        # pil_image = Image.fromarray(image_np_rgb)
        # pil_image.show()
        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(1)
        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "png"
        # msg.data = np.array(cv2.imencode('.png', image_np)[1]).tostring()
        # self.image_pub.publish(msg)

        # pred_bmask = self.get_bmask(image_np_rgb)  # a 0 1 matrix mask
        # pixel_x, pixel_y = self.deduce_safe_loc(pred_bmask)
        # orient_1_x, orient_1_y, orient_2_x, orient_2_y = self.deduce_safe_angle(pred_bmask)

        # publishing 6 floats:
        #       0 -> bool for gotSafePose (1 means true)
        #       1 -> bool for loc (0 -> local, 1 -> ground)
        #       2 -> x
        #       3 -> y
        #       4 -> bool for theta (0 -> local, 1 -> ground)
        #       5 -> theta

        msg = Float64MultiArray()
        msg.data.append(1)
        msg.data.append(1)
        msg.data.append(30.21)
        msg.data.append(21.12)
        msg.data.append(1)
        msg.data.append(-0.64)
        self.loc_pub.publish(msg)


def setup_ros_node():
    rospy.init_node('contingency_image_extractor', anonymous=False)
    tester = ExtractSafeSpot()
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS safe spot extractor module")


setup_ros_node()


# def image_loader(dir, img_name, toFloat=False):  # 1280 x 1920 x 3 image shape is returned
#     out = np.array(Image.open(os.path.join(dir, img_name)))
#     if toFloat:
#         out = np.array(out, dtype=np.float64)
#         out = out / 255.0
#     return out


# def GetInputforInference(idx):
#     img = image_loader(images_dir, all_image_names[idx], toFloat=True)
#     img = transformImg(img)
#     if take_semseg:
#         semseg = rgb_to_semlabel(image_loader(
#             semseg_dir, all_image_names[idx]))
#         semseg = myToTensor(semseg)
#         inp = torch.cat((img, semseg), dim=0)
#     else:
#         inp = img
#     return inp
