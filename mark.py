import cv2
from matplotlib import pyplot as plt

def main():
    start = (2030, 480)
    goal = (80, 850)
    mapimg = cv2.imread("./eer_lawn_moremasked.jpg")
    cv2.circle(mapimg, start, 10, 0xFF0000, thickness=-1)
    cv2.circle(mapimg, goal, 10, 0x00FF00, thickness=-1)
    plt.imsave("eer_lawn_moremasked_marked.jpg", mapimg)

    

if __name__ == "__main__":
    main()