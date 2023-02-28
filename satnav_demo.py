from typing import List, Tuple
import roslib

roslib.load_manifest("graph_navigation")

from math import ceil
from queue import PriorityQueue

import cv2
import numpy as np
import rospy
from amrl_msgs.msg import Localization2DMsg, Pose2Df
from matplotlib import pyplot as plt


class CustomQueue(PriorityQueue):
    def __contains__(self, item):
        with self.mutex:
            for i in self.queue:
                if item == i[1]:
                    return True


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

    start_loc = None  # no longer needed bc assuming start of (0, 0, -pi/2)
    fac: int = 20
    last_carrot_idx: int = None
    plan: List[Tuple[int, int]] = None
    carrot_set_dist = 3


def anglemod(a: float) -> float:
    return a - (2 * np.pi) * np.round(a / (2 * np.pi))


def robot_to_image_location(pos: Tuple[float, float, float]) -> Tuple[int, int]:
    x, y, theta = pos
    x = round(-x * Namespace.fac)
    y = round(y * Namespace.fac)
    return x, y


# takes position of robot in image and returns index of next carrot to send
def find_next_carrot(pos: Tuple[int, int]) -> int:
    if Namespace.last_carrot_idx is None:
        return min(len(plan) - 1, 2)
    else:
        dlast = np.sqrt(
            (pos[0] - plan[Namespace.last_carrot_idx][0]) ** 2
            + (pos[1] - plan[Namespace.last_carrot_idx][1]) ** 2
        )
        if dlast > Namespace.carrot_set_dist:
            return Namespace.last_carrot_idx
        else:
            return min(Namespace.last_carrot_idx + 1, len(plan) - 1)


def image_to_robot_location(pos: Tuple[int, int]) -> Tuple[float, float, float]:
    x, y = pos
    x = -x / Namespace.fac
    y = y / Namespace.fac
    return x, y, -np.pi / 2


def loc_callback(msg: Localization2DMsg):
    pos = (msg.pose.x, msg.pose.y, msg.pose.theta)
    if start_loc is None:
        start_loc = pos

    # first find robot position in image,
    impos = robot_to_image_location(pos)

    # then, using last set carrot, find next carrot that is closest to the robot
    newcarroti = find_next_carrot(impos)
    Namespace.last_carrot_idx = newcarroti
    newcarrot = Namespace.plan[newcarroti]

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
        cv2.circle(mapimg, tuple([i * 40 for i in k]), 5, 0x0000FF, thickness=-1)
    plt.imsave("eernav.jpg", mapimg)

    rospy.init_node("satnav")
    Namespace.pub = rospy.Publisher("/nav_override", Localization2DMsg, queue_size=1)

    l = rospy.Subscriber("/localization", Localization2DMsg, loc_callback)

    rospy.spin()
