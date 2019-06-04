from RGBD_XYZ.rgb_xy import *
from RGBD_XYZ.xyd_xyz import *
import numpy as np
import matplotlib.pyplot as plt


def img2xyz(rgb, dep):
    rgb2xy = rgb_xy()
    xy = rgb2xy.detect(rgb)
    x = xy[0].astype(np.int32)
    y = xy[1].astype(np.int32)
    # get xy for image
    xyd = np.concatenate((xy, np.array(dep[x][y][:1])), axis=0)
    # get depth for dep
    plt.imshow(rgb)
    print(xyd)
    xyd2xyz = xyd_xyz()
    xyz = xyd2xyz.detect(xyd)
    # get the xyz in the space
    print(xyz)


if __name__ == "__main__":
    # ----- test ----
    rgb, dep = read_img_dep("[0.5513610201719624, 0.1887571504509108, 0.75]")
    # ----- test ----
    img2xyz(rgb, dep)
