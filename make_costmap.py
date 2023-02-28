from math import ceil

import cv2
import numpy as np
import torch
from matplotlib import pyplot as plt
from PIL import Image


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


if __name__ == "__main__":
    mapimg = cv2.imread("./eer_lawn_moremasked.jpg")
    mapimg = cv2.cvtColor(mapimg, cv2.COLOR_BGR2RGB)
    costmap = create_costmap(mapimg)
    plt.imsave("eercost.jpg", costmap)
