import tkinter as tk
from tkinter import ttk

root = tk.Tk()
root.geometry('450x250')
root.resizable(False, False)
root.title('PID controller')

k_p = tk.DoubleVar()
k_i = tk.DoubleVar()
k_d = tk.DoubleVar()

robot_x = tk.DoubleVar()
robot_y = tk.DoubleVar()

def init_target_controller():
    slider_label = ttk.Label(
        root,
        text='X: ',
        font=("Roboto", 18),
    )

    slider_label.grid(
        column=2,
        row=0,
        sticky='w',
        padx=15,
        pady=10,
        ipadx=20,
        ipady=10
    )

    slider = ttk.Scale(
        root,
        from_=0,
        to=1,
        orient='horizontal',
        variable=robot_x
    )
    slider.set(0.5)

    slider.grid(
        column=3,
        row=0,
        sticky='we',
        ipadx=10,
        ipady=10
    )

    slider_label = ttk.Label(
        root,
        text='Y: ',
        font=("Roboto", 18),
    )

    slider_label.grid(
        column=2,
        row=1,
        sticky='w',
        padx=15,
        pady=10,
        ipadx=20,
        ipady=10
    )

    slider = ttk.Scale(
        root,
        from_=0,
        to=1,
        orient='horizontal',
        variable=robot_y
    )
    slider.set(0.5)

    slider.grid(
        column=3,
        row=1,
        sticky='we',
        ipadx=10,
        ipady=10
    )

def init_pid_controller():
    slider_label = ttk.Label(
        root,
        text='P: ',
        font = ("Roboto", 18),
    )

    slider_label.grid(
        column=0,
        row=0,
        sticky='w',
        padx=15,
        pady=10,
        ipadx=20,
        ipady=10
    )

    slider = ttk.Scale(
        root,
        from_=0,
        to=0.2,
        orient='horizontal',
        variable=k_p
    )

    slider.grid(
        column=1,
        row=0,
        sticky='we',
        ipadx=10,
        ipady=10
    )


    slider_label = ttk.Label(
        root,
        text='I: ',
        font=("Roboto", 18),
    )

    slider_label.grid(
        column=0,
        row=1,
        sticky='w',
        padx=15,
        pady=10,
        ipadx=20,
        ipady=10
    )
    slider = ttk.Scale(
        root,
        from_=0,
        to=0.2,
        orient='horizontal',
        variable=k_i
    )

    slider.grid(
        column=1,
        row=1,
        sticky='we',
        ipadx=10,
        ipady=10
    )

    slider_label = ttk.Label(
        root,
        text='D: ',
        font=("Roboto", 18),
    )

    slider_label.grid(
        column=0,
        row=2,
        sticky='w',
        padx=15,
        pady=10,
        ipadx=20,
        ipady=10
    )
    slider = ttk.Scale(
        root,
        from_=0,
        to=0.2,
        orient='horizontal',
        variable=k_d
    )

    slider.grid(
        column=1,
        row=2,
        sticky='we',
        ipadx=10,
        ipady=10
    )

def init_controller(target:bool = None, pid:bool = None):
    if target:
        init_target_controller()
    if pid:
        init_pid_controller()

def ui_loop():
    root.mainloop()