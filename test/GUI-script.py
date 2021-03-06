
import os
import time
import tkinter as tk
from tkinter import font
from PIL import ImageTk, Image
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure

from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transformations import plot_transform
from pytransform3d.rotations import matrix_from_euler_xyz
import matplotlib.pyplot as plt

# Set cuurent working directory
os.chdir("C:/toy-projects/robotics-final-project/robotics-final-project/bin")


class Application(tk.Frame):
    def __init__(self, master=None):

        super().__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        self.my_font = tk.font.Font(family='仓耳今楷05-6763 W03', size=15)

        self.disp_img_src_button = tk.Button(self)
        self.disp_img_src_button["text"] = "手眼标定"
        self.disp_img_src_button["command"] = self.eye_in_hand_calibration
        self.disp_img_src_button["font"] = self.my_font
        self.disp_img_src_button.pack(side="left")

        self.disp_img_src_button = tk.Button(self)
        self.disp_img_src_button["text"] = "采集图像"
        self.disp_img_src_button["command"] = self.disp_img_src
        self.disp_img_src_button["font"] = self.my_font
        self.disp_img_src_button.pack(side="left")

        self.disp_img_dst_button = tk.Button(self)
        self.disp_img_dst_button["text"] = "物体识别"
        self.disp_img_dst_button["command"] = self.target_recognition
        self.disp_img_dst_button["font"] = self.my_font
        self.disp_img_dst_button.pack(side="left")

        self.disp_img_dst_button = tk.Button(self)
        self.disp_img_dst_button["text"] = "最优路径"
        self.disp_img_dst_button["command"] = self.strategy
        self.disp_img_dst_button["font"] = self.my_font
        self.disp_img_dst_button.pack(side="left")

        self.disp_motion_plan_button = tk.Button(self)
        self.disp_motion_plan_button["text"] = "运动规划"
        self.disp_motion_plan_button["command"] = self.disp_motion_plan
        self.disp_motion_plan_button["font"] = self.my_font
        self.disp_motion_plan_button.pack(side="left")

        self.disp_motion_plan_button = tk.Button(self)
        self.disp_motion_plan_button["text"] = "执行"
        self.disp_motion_plan_button["fg"] = "red"
        # self.disp_motion_plan_button["command"] = self.disp_motion_plan
        self.disp_motion_plan_button["font"] = self.my_font
        self.disp_motion_plan_button.pack(side="left")

        self.quit = tk.Button(self, text="退出", fg="red", font=self.my_font,
                              command=self.master.destroy)
        self.quit.pack(side="left")

        self.panel = tk.Label(self.master)
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.toolbar = NavigationToolbar2Tk(self.canvas, root)

    def disp_img_src(self):
        self.panel.destroy()
        self.canvas.get_tk_widget().destroy()
        self.toolbar.destroy()

        self.img_src_path = "../share/target-recognition/src/img_src.jpg"
        self.img_src = ImageTk.PhotoImage(Image.open(self.img_src_path))
        self.panel = tk.Label(self.master, image=self.img_src)
        self.panel.pack(side="bottom", fill="both", expand="yes")

    def eye_in_hand_calibration(self):
        self.panel.destroy()
        self.canvas.get_tk_widget().destroy()
        self.toolbar.destroy()

        action_path = "eye-in-hand-calibration.exe"
        os.startfile(action_path)
        time.sleep(0.5)

    def target_recognition(self):
        self.panel.destroy()
        self.canvas.get_tk_widget().destroy()
        self.toolbar.destroy()

        action_path = "brick-construction.exe"
        os.startfile(action_path)
        time.sleep(0.5)

        self.img_dst_path = "../share/target-recognition/dst/img_rectangle_detection.jpg"
        self.img_dst = ImageTk.PhotoImage(Image.open(self.img_dst_path))
        self.panel = tk.Label(self.master, image=self.img_dst)
        self.panel.pack(side="bottom", fill="both", expand="yes")

    def strategy(self):
        self.panel.destroy()
        self.canvas.get_tk_widget().destroy()
        self.toolbar.destroy()
        self.img_dst_path = "../share/target-recognition/dst/img_strategy.jpg"
        self.img_dst = ImageTk.PhotoImage(Image.open(self.img_dst_path))
        self.panel = tk.Label(self.master, image=self.img_dst)
        self.panel.pack(side="bottom", fill="both", expand="yes")

    def disp_motion_plan(self):
        self.panel.destroy()
        self.canvas.get_tk_widget().destroy()
        self.toolbar.destroy()

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.draw()
        self.ax = self.fig.add_subplot(111, projection="3d")

        # simulation_files = os.listdir("../share/motion-plan/simulation")
        # print(simulation_files)
        # for i in range(len(simulation_files)):
        #     self.data = np.loadtxt(
        #         "../share/motion-plan/simulation/"+simulation_files[i])
        #     self.ax.plot(self.data[:,0], self.data[:,1], self.data[:,2], color="c")

        self.xyz_key_pts = np.loadtxt("../share/log.txt")[:, 0:3]
        self.ax.scatter(
            self.xyz_key_pts[0, 0], self.xyz_key_pts[0, 1], self.xyz_key_pts[0, 2], color="black")
        self.xyz_key_pts = np.delete(self.xyz_key_pts, (0), axis=0)
        self.ax.scatter(self.xyz_key_pts[0::2][:, 0], self.xyz_key_pts[0::2]
                        [:, 1], self.xyz_key_pts[0::2][:, 2], color="blue")
        self.ax.scatter(self.xyz_key_pts[1::2][:, 0], self.xyz_key_pts[1::2]
                        [:, 1], self.xyz_key_pts[1::2][:, 2], color="red")


        # self.data = np.loadtxt("../share/motion-plan/simulation/test_simulation.txt")
        # self.x = self.data[:, 0]
        # self.y = self.data[:, 1]
        # self.z = self.data[:, 2]
        # self.ax.plot(self.x, self.y, self.z)

        self.toolbar = NavigationToolbar2Tk(self.canvas, root)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand="yes")


root = tk.Tk()
root.title("test")
# root.geometry("1366x768")
root.geometry("640x570")
root.configure(background='white')

app = Application(master=root)
app.mainloop()
