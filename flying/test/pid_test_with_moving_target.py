import matplotlib
from matplotlib import pyplot as plt, animation
from flying.test.controller_ui import ui_loop, init_controller, k_p, k_i, k_d, robot_x, robot_y
from flying.utils.PID import Position, PID

matplotlib.use('TkAgg')
fig, ax = plt.subplots()

init_controller(target=True, pid=True)

ax.set_xlim(0,1)
ax.set_ylim(0,1)
target_point = ax.plot(0.5, 0.5, 'og')[0]

pid = PID()

x = 0
y = 0.5

pid_result = ax.plot(0, 0.5, '.r')[0]


def animate(i):
    global x, y, pid
    # print(k_p.get(), k_i.get(), k_d.get())

    # update pid
    pid.robot_position = Position(robot_x.get(), robot_y.get(), 0.5)
    pid.k_p = k_p.get()
    pid.k_i = k_i.get()
    pid.k_d = k_d.get()

    target_point.set_data([robot_x.get()], [robot_y.get()])
    dx, dy, dz = pid.get_rc_controls(Position(x,y,0))
    x += dx
    y += dy

    pid_result.set_data([x], [y])

    return target_point, pid_result


ani = animation.FuncAnimation(fig=fig, func=animate, frames=500, interval=20, repeat=True, blit=True)
plt.show()
ui_loop()
