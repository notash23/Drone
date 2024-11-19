import matplotlib
from matplotlib import pyplot as plt, animation
from flying.test.controller_ui import ui_loop, init_controller, k_p, k_i, k_d, robot_x, robot_y
from flying.utils.PID import Position, PID

matplotlib.use('TkAgg')
fig, axs = plt.subplots(2, 2)

init_controller(target=True, pid=True)

target_points = []
for r_ax in axs:
    for ax in r_ax:
        ax.set_xlim(0,1)
        ax.set_ylim(0,1)
        target_points.append(ax.plot(0.5, 0.5, 'og')[0])

pids = [
    PID(
        k_p=0.03,
        k_i=0,
        k_d=0,
        robot_position=Position(0.5, 0.5, 0.5)
    ),
    PID(
        k_p=0.5,
        k_i=0.02,
        k_d=0.25,
        robot_position=Position(0.5, 0.5, 0.5)
    ),
    PID(
        k_p=0.2,
        k_i=0.01,
        k_d=0.4,
        robot_position=Position(0.5, 0.5, 0.5)
    ),
    PID(
        k_p=0.2,
        k_i=0.01,
        k_d=0.25,
        robot_position=Position(0.5, 0.5, 0.5)
    )
]

x = []
y = []
for _ in range(4):
    x.append(0)
    y.append(0.5)

lines = [
    axs[0, 0].plot(0, 0, '.r')[0],
    axs[0, 1].plot(0, 0, '.r')[0],
    axs[1, 0].plot(0, 0, '.r')[0],
    axs[1, 1].plot(0, 0, '.r')[0]
]


def animate(i):
    global x, y
    # print(k_p.get(), k_i.get(), k_d.get())
    pids[0].robot_position = Position(robot_x.get(), robot_y.get(), 0.5)
    target_points[0].set_data([robot_x.get()], [robot_y.get()])
    if i==1:
        x = []
        y = []
        for _ in range(4):
            x.append(0)
            y.append(0.5)
            pids[i].reset()
        # pids[0].k_p = k_p.get()
        # pids[0].k_i = k_i.get()
        # pids[0].k_d = k_d.get()
    for i, pid in enumerate(pids):
        dx, dy, dz = pid.get_rc_controls(Position(x[i],y[i],0))
        x[i] += dx
        y[i] += dy

        lines[i].set_data([x[i]], [y[i]])

    return lines + target_points


ani = animation.FuncAnimation(fig=fig, func=animate, frames=500, interval=20, repeat=True, blit=True)
plt.show()
ui_loop()
