
import os
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transformations import plot_transform
from pytransform3d.rotations import matrix_from_euler_xyz
import matplotlib.pyplot as plt
import numpy as np

# Set cuurent working directory
os.chdir("C:/toy-projects/robotics-final-project/robotics-final-project/bin")

# xyz_key_pts = np.loadtxt("../share/log.txt")
# xyzrpy_pts = np.loadtxt("../share/motion-plan/simulation/test_simulation.txt")[1::200]
# ax = make_3d_axis(ax_s=600, pos=111, n_ticks=6)
# homo = np.empty((4, 4))
# for row in xyzrpy_pts:
#     rot = matrix_from_euler_xyz(row[[5,4,3]])
#     trans = row[0:3]
#     # print(rot)
#     # print(trans)
#     homo[:3, :3] = rot
#     homo[:3, 3] = trans
#     homo[3, :] = [0, 0, 0, 1]
#     # print(homo)
#     plot_transform(ax=ax, A2B=homo, s=50)
#     # plt.axis('off')
#     plt.tight_layout()
#     plt.plot(xyzrpy_pts[:,0], xyzrpy_pts[:,1], xyzrpy_pts[:,2], color="yellow")
# plt.show()

xyzrpy_pts_norm = np.linalg.norm(np.loadtxt(
    "../share/motion-plan/simulation/test_simulation.txt"), axis=1)
vel = np.diff(xyzrpy_pts_norm)
acc = np.diff(vel)
plt.plot(xyzrpy_pts_norm)
plt.show()
plt.plot(vel)
plt.show()
plt.plot(acc)
plt.show()

xyzrpy_pts_1 = np.loadtxt(
    "../share/motion-plan/simulation/test_simulation.txt")[:, 0]
vel_1 = np.diff(xyzrpy_pts_1)
plt.plot(vel_1)
xyzrpy_pts_2 = np.loadtxt(
    "../share/motion-plan/simulation/test_simulation.txt")[:, 1]
vel_2 = np.diff(xyzrpy_pts_2)
plt.plot(vel_2)
xyzrpy_pts_3 = np.loadtxt(
    "../share/motion-plan/simulation/test_simulation.txt")[:, 2]
vel_3 = np.diff(xyzrpy_pts_3)
plt.plot(vel_3)
xyzrpy_pts_4 = np.loadtxt(
    "../share/motion-plan/simulation/test_simulation.txt")[:, 3]
vel_4 = np.diff(xyzrpy_pts_4)
plt.plot(vel_4)
xyzrpy_pts_5 = np.loadtxt(
    "../share/motion-plan/simulation/test_simulation.txt")[:, 4]
vel_5 = np.diff(xyzrpy_pts_5)
plt.plot(vel_5)
xyzrpy_pts_6 = np.loadtxt(
    "../share/motion-plan/simulation/test_simulation.txt")[:, 5]
vel_6 = np.diff(xyzrpy_pts_6)
plt.plot(vel_6)
plt.show()

plt.plot(xyzrpy_pts_1)
plt.plot(xyzrpy_pts_2)
plt.plot(xyzrpy_pts_3)
plt.plot(xyzrpy_pts_4)
plt.plot(xyzrpy_pts_5)
plt.plot(xyzrpy_pts_6)
plt.show()
