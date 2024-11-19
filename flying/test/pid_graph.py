import matplotlib
import numpy as np
from matplotlib import pyplot as plt

from flying.utils.PID import Position, PID

matplotlib.use('TkAgg')
fig, ax = plt.subplots()

time = 20
steps = 20000

x = np.linspace(0, time, steps)

# Only P
pid = PID(
    k_p= 1,
    k_i = 0,
    k_d = 0,
    max_windup = Position(10, 10, 10),
    robot_position = Position(0, 1, 0)
)
y = []
y_val = 0
for i in range(steps):
    _, dy, _ = pid.get_rc_controls(Position(0,y_val,0), timestep=time/steps)
    y_val += dy*time/steps
    y.append(y_val)

y = np.array(y)
plt.plot(x,y)

# Only PI
pid = PID(
    k_p= 1,
    k_i = 1,
    k_d = 0,
    max_windup = Position(100, 100, 100),
    robot_position = Position(0, 1, 0)
)
y = []
y_val = 0
for i in range(steps):
    _, dy, _ = pid.get_rc_controls(Position(0,y_val,0), timestep=time/steps)
    y_val += dy*time/steps
    y.append(y_val)

y = np.array(y)
plt.plot(x,y)

# Only PID
pid = PID(
    k_p= 1,
    k_i = 0.3,
    k_d = 0.2,
    max_windup = Position(100, 0.2, 100),
    robot_position = Position(0, 1, 0)
)
y = []
y_val = 0
for i in range(steps):
    _, dy, _ = pid.get_rc_controls(Position(0,y_val,0), timestep=time/steps)
    y_val += dy*time/steps
    y.append(y_val)

y = np.array(y)
plt.plot(x,y)

plt.show()
