import roslib

roslib.load_manifest("graph_navigation")

import rospy
import numpy as np
from amrl_msgs.msg import Localization2DMsg, Pose2Df

import argparse

import cv2
import torch
import numpy as np
from PIL import Image
from math import ceil
from matplotlib import pyplot as plt
from queue import PriorityQueue


class PatchFromImageDataset:
    def __init__(self, img_path: str, patch_size: int = 40) -> None:
        self.image = np.array(Image.open(img_path))
        self.image = np.moveaxis(self.image, [2, 0, 1], [0, 1, 2])[:3]
        self.patch_size = patch_size
        self.h = int(ceil(self.image.shape[1] / self.patch_size))
        self.w = int(ceil(self.image.shape[2] / self.patch_size))
        self.len = self.w * self.h

    def __len__(self):
        return self.len

    def __getitem__(self, idx):
        w = self.patch_size * (idx % self.w)
        h = self.patch_size * (idx // self.w)
        out = self.image[:, h : h + self.patch_size, w : w + self.patch_size]
        if out.shape != [3, 40, 40]:
            k = np.zeros((3, 40, 40))
            k[:, : out.shape[1], : out.shape[2]] = out
            out = k
        return out


class CustomQueue(PriorityQueue):
    def __contains__(self, item):
        with self.mutex:
            for i in self.queue:
                if item == i[1]:
                    return True


def create_costmap(img):
    dataset = PatchFromImageDataset("./eer_lawn_moremasked.jpg")
    loader = torch.utils.data.DataLoader(dataset, batch_size=1)
    model = torch.jit.load("./jit_cost_model_outdoor_6dim.pt")
    model.eval()
    model.cuda()

    costmap = np.zeros((dataset.h, dataset.w))
    i = 0
    with torch.no_grad():
        for batch in loader:
            h = i // dataset.w
            w = i % dataset.w
            if batch.numpy().any():
                preds = list(model(batch.float().to("cuda")).cpu().numpy())
                costmap[h, w] = preds[0]
            else:
                costmap[h, w] = 100
            if i % (len(dataset) // 10) == 0:
                print(i / len(dataset))
            i += 1

    return costmap


def plan(start, goal, costmap):
    def h(p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def make_path(p, c):
        path = [c]
        while c in p:
            c = p[c]
            path.insert(0, c)
        return path

    def get_neighbors(p):
        x, y = p
        out = []
        if x - 1 > 0:
            out.append((x - 1, y))
        if x + 1 < costmap.shape[1]:
            out.append((x + 1, y))
        if y - 1 > 0:
            out.append((x, y - 1))
        if y + 1 < costmap.shape[0]:
            out.append((x, y + 1))
        return out

    def edge_cost(p1, p2):
        # print(p1, p2)
        return max(costmap[p1[1]][p1[0]], costmap[p2[1]][p2[0]])

    start = tuple([int(ceil(i / 40)) for i in start])
    goal = tuple([int(ceil(i / 40)) for i in goal])
    print(costmap.shape, start, goal)

    q = CustomQueue()
    q.put((0, start))
    closed = set()
    prev = {}
    g = {}
    g[start] = 0

    while not q.empty():
        s, curr = q.get()
        closed.add(curr)

        for i in get_neighbors(curr):
            if i not in closed:
                gval = g[curr] + edge_cost(curr, i)
                hval = h(i, goal)
                if i not in q or g[i] > gval:
                    prev[i] = curr
                    g[i] = gval
                    q.put((gval + hval, i))

        if goal in prev:
            return make_path(prev, goal)

    return []


class Namespace:
    pub = None

    delta = False
    x = 0.0
    y = 0.0

    start_loc = None
    fac = 20
    last_carrot = None
    plan = None


def loc_callback(msg: Localization2DMsg):
    pos = (msg.pose.x, msg.pose.y, msg.pose.theta)
    if start_loc is None:
        start_loc = pos

    # first find robot position in image,
    impos = robot_to_image_location(pos)

    # then, using last set carrot, find next carrot that is closest to the robot
    newcarrot = find_next_carrot(impos)
    Namespace.last_carrot = newcarrot

    # convert the carrot to robot local coordinates
    localcarrot = image_to_robot_location(newcarrot)

    # publish the carrot
    m = Pose2Df()
    m.x = localcarrot[0]
    m.y = localcarrot[1]
    m.theta = localcarrot[2]
    Namespace.pub.publish(m)


if __name__ == "__main__":
    mapimg = cv2.imread("./eer_lawn_moremasked.jpg")
    costmap = cv2.imread("./eercost.jpg", cv2.IMREAD_GRAYSCALE)

    start = (2050, 500)
    goal = (580, 790)
    p = plan(start, goal, costmap)
    Namespace.plan = p
    cv2.circle(mapimg, start, 10, 0xFF0000, thickness=-1)
    cv2.circle(mapimg, goal, 10, 0x00FF00, thickness=-1)
    for k in p:
        cv2.circle(mapimg, [i * 40 for i in k], 5, 0x0000FF, thickness=-1)
    plt.imsave("eernav.jpg", mapimg)

    # rospy.init_node("satnav")
    # Namespace.pub = rospy.Publisher("/nav_override", Localization2DMsg, queue_size=1)

    # l = rospy.Subscriber("/localization", Localization2DMsg, loc_callback)

    # rospy.spin()
