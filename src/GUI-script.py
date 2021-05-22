
import tkinter as tk
from tkinter import font
from PIL import ImageTk, Image


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
        self.disp_img_dst_button["text"] = "图像识别"
        self.disp_img_dst_button["command"] = self.disp_img_dst
        self.disp_img_dst_button.pack(side="left")

        self.panel = tk.Label(self.master)

        self.quit = tk.Button(self, text="退出", fg="red",
                              command=self.master.destroy)
        self.quit.pack(side="left")

    def disp_img_src(self):
        self.panel.destroy()
        self.img_src_path = "./data/rectangle-detection/img_src.jpg"
        self.img_src = ImageTk.PhotoImage(Image.open(self.img_src_path))
        self.panel = tk.Label(self.master, image=self.img_src)
        self.panel.pack(side="bottom", fill="both", expand="yes")

    def disp_img_dst(self):
        self.panel.destroy()
        self.img_dst_path = "./data/rectangle-detection/img_dst.jpg"
        self.img_dst = ImageTk.PhotoImage(Image.open(self.img_dst_path))
        self.panel = tk.Label(self.master, image=self.img_dst)
        self.panel.pack(side="bottom", fill="both", expand="yes")


root = tk.Tk()
root.title("test")
root.geometry("1600x1200")
root.configure(background='white')

app = Application(master=root)
app.mainloop()
