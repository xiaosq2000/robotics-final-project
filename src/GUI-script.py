import tkinter as tk
from PIL import ImageTk, Image
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure


class Application(tk.Frame):
    def __init__(self, master=None):

        super().__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        self.disp_img_src_button = tk.Button(self)
        self.disp_img_src_button["text"] = "采集图像"
        self.disp_img_src_button["command"] = self.disp_img_src
        self.disp_img_src_button.pack(side="left")

        self.disp_img_dst_button = tk.Button(self)
        self.disp_img_dst_button["text"] = "物体识别与最优中心计算"
        self.disp_img_dst_button["command"] = self.disp_img_dst
        self.disp_img_dst_button.pack(side="left")

        self.disp_motion_plan_button = tk.Button(self)
        self.disp_motion_plan_button["text"] = "运动规划可视化"
        self.disp_motion_plan_button["command"] = self.disp_motion_plan
        self.disp_motion_plan_button.pack(side="left")

        self.quit = tk.Button(self, text="退出", fg="red",
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

        self.img_src_path = "./data/rectangle-detection/img_src.jpg"
        self.img_src = ImageTk.PhotoImage(Image.open(self.img_src_path))
        self.panel = tk.Label(self.master, image=self.img_src)
        self.panel.pack(side="bottom", fill="both", expand="yes")

    def disp_img_dst(self):
        self.panel.destroy()
        self.canvas.get_tk_widget().destroy()
        self.toolbar.destroy()

        self.img_dst_path = "./data/rectangle-detection/img_dst.jpg"
        self.img_dst = ImageTk.PhotoImage(Image.open(self.img_dst_path))
        self.panel = tk.Label(self.master, image=self.img_dst)
        self.panel.pack(side="bottom", fill="both", expand="yes")

    def disp_motion_plan(self):
        self.panel.destroy()
        self.canvas.get_tk_widget().destroy()
        self.toolbar.destroy()

        self.data = np.loadtxt('./data/motion-plan/data_1.txt')

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.draw()
        self.ax = self.fig.add_subplot(111, projection="3d")

        # self.x = self.data[:, 0]
        # self.y = self.data[:, 1]
        # self.z = self.data[:, 2]
        # self.ax.plot(self.x, self.y, self.z)
        self.x = np.arange(0,3,.01)
        # self.ax.plot(self.x, self.y, self.z)

        self.ax.plot(self.x, 2 * np.sin(2 * np.pi * self.x))

        self.toolbar = NavigationToolbar2Tk(self.canvas, root)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand="yes")


root = tk.Tk()
root.title("test")
root.geometry("1600x1200")
root.configure(background='white')

app = Application(master=root)
app.mainloop()
