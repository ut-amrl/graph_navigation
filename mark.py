import cv2
from matplotlib import pyplot as plt

def main():
    start = (895, 110)
    goal = (240, 320)
    mapimg = cv2.cvtColor(cv2.imread("./eerlawn_initial.jpg"), cv2.COLOR_BGR2RGB)
    cv2.circle(mapimg, start, 5, 0xFF0000, thickness=-1)
    cv2.circle(mapimg, goal, 5, 0x00FF00, thickness=-1)
    plt.imsave("marked.jpg", mapimg)

    

if __name__ == "__main__":
    main()